#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "bbot_balance_controller/keyboard_reader.h"
#include "bbot_balance_controller/flight_joint_pd.hpp"
#include "bbot_balance_controller/takeoff_detection.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"
#include "bbot_balance_controller/control_timing.hpp"
#include "bbot_balance_controller/ground_joint_pd.hpp"
#include "bbot_balance_controller/torso_pitch_control.hpp"
#include "bbot_balance_controller/world_pose_velocity.hpp"
#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/effort_allocation.hpp"
#include "bbot_balance_controller/rolling_jump_control.hpp"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

namespace bbot_jump
{

// 工具函数
inline double clamp_value(double value, double min_value, double max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

inline double deadband(double value, double threshold)
{
    if (std::abs(value) < threshold) return 0.0;
    return (value > 0.0) ? (value - threshold) : (value + threshold);
}

inline double low_pass_filter(double new_value, double old_value, double alpha)
{
    return alpha * new_value + (1.0 - alpha) * old_value;
}

inline double lerp(double a, double b, double ratio)
{
    return a + (b - a) * ratio;
}

/// @brief 五次多项式轨迹生成器 (C2 连续)
struct QuinticTrajectory
{
    double t0 = 0.0;
    double tf = 0.0;
    double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, a4 = 0.0, a5 = 0.0;

    void init(double t_start, double duration,
              double z0, double v0, double acc0,
              double zf, double vf, double accf)
    {
        t0 = t_start;
        tf = t_start + duration;
        double T = (duration > 1e-4) ? duration : 1e-4;

        a0 = z0;
        a1 = v0 * T;
        a2 = 0.5 * acc0 * T * T;

        double delta_z = zf - (a0 + a1 + a2);
        double delta_v = vf * T - (a1 + 2.0 * a2);
        double delta_a = accf * T * T - 2.0 * a2;

        a3 = 10.0 * delta_z - 4.0 * delta_v + 0.5 * delta_a;
        a4 = -15.0 * delta_z + 7.0 * delta_v - 1.0 * delta_a;
        a5 = 6.0 * delta_z - 3.0 * delta_v + 0.5 * delta_a;
    }

    void evaluate(double t, double & z_out, double & v_out, double & acc_out) const
    {
        double duration = tf - t0;
        if (duration <= 1e-4) {
            z_out = a0;
            v_out = 0.0;
            acc_out = 0.0;
            return;
        }

        double tau = (t - t0) / duration;
        tau = clamp_value(tau, 0.0, 1.0);

        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        z_out = a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
        v_out = (a1 + 2.0 * a2 * tau + 3.0 * a3 * tau2 + 4.0 * a4 * tau3 + 5.0 * a5 * tau4) / duration;
        acc_out = (2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau2 + 20.0 * a5 * tau3) / (duration * duration);
    }

    bool is_finished(double t) const
    {
        return t >= tf;
    }
};

// 有限采样的轨迹准入检查；保留位置裕量，不能只验证端点连续。
inline bool flight_trajectory_admissible(const QuinticTrajectory & traj,
                                         double speed_limit, double acceleration_limit)
{
    for (int i = 0; i <= 256; ++i) {
        double q, v, a;
        traj.evaluate(traj.t0 + (traj.tf - traj.t0) * i / 256.0, q, v, a);
        if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a) ||
            std::abs(q) > 1.45 || std::abs(v) > speed_limit ||
            std::abs(a) > acceleration_limit) return false;
    }
    return true;
}

// 末端行程保护：只预测“离地前仍可能继续保持当前关节速度”的短时间窗口。
// v5.6：上一轮在 knee≈-0.92 rad、-9.7 rad/s 时仍被 45 ms 预瞄提前削到 0.75，
// 实际离地速度仅 1.83 m/s。缩短到 30 ms：仍保留高速末端保护，但不在尚有
// 约 0.5 rad 行程时过早损失竖直冲量。硬关节限幅本身仍保持不变。
inline double joint_extension_scale(double q, double v)
{
    if (std::abs(v) < 1.0) return 1.0;
    const double remaining = v > 0.0 ? 1.45 - q : q + 1.45;
    constexpr double preview_time = 0.030;
    constexpr double ramp_margin = 0.10;
    return clamp_value((remaining - preview_time * std::abs(v)) / ramp_margin, 0.0, 1.0);
}

/// @brief LQR 反馈增益
struct LQRGain
{
    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;
};

/// @brief 跳跃状态机枚举
enum JumpState
{
    // 0~7 数值保持不变，避免破坏现有 CSV/边界测试对 state 数字的解释。
    STATE_BALANCE = 0,            // 0: 变高度 LQR 自平衡状态
    STATE_SQUAT = 1,              // 1: 下蹲蓄力阶段 (L -> L_SQUAT_)
    STATE_THRUST = 2,             // 2: 爆发推地阶段 (L_SQUAT_ -> H_TAKEOFF_)
    STATE_FLIGHT = 3,             // 3: 腾空相阶段 (冲顶 -> 收腿 0.30m -> 预展腿 0.40m)
    STATE_TOUCHDOWN_BUFFER = 4,   // 4: 触地缓冲阻抗控制
    STATE_RECOVERY = 5,           // 5: 平稳沉降与消除反弹
    STATE_STANDUP = 6,            // 6: 倒地起立自恢复
    STATE_EMERGENCY = 7,          // 7: 紧急停机
    STATE_PRE_JUMP = 8            // 8: 滚动起跳准备：建立前向速度与前倾工作点
};

/// @brief 腾空相内部子阶段枚举
enum FlightSubphase
{
    FLIGHT_SUBPHASE_ATTITUDE_ARREST = 0,  // 0: 姿态刹车阶段
    FLIGHT_SUBPHASE_TUCK = 1,             // 1: 五次平滑收腿阶段
    FLIGHT_SUBPHASE_EXTEND = 2,           // 2: 顶点展腿阶段
    FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY = 3 // 3: 保护展腿阶段
};

inline const char* flight_subphase_to_string(FlightSubphase s)
{
    switch (s) {
        case FLIGHT_SUBPHASE_ATTITUDE_ARREST: return "ATTITUDE_ARREST";
        case FLIGHT_SUBPHASE_TUCK: return "TUCK";
        case FLIGHT_SUBPHASE_EXTEND: return "EXTEND";
        case FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY: return "PROTECTIVE_DEPLOY";
        default: return "UNKNOWN";
    }
}

/// @brief 恢复阶段内部交接子阶段枚举
enum RecoverySubphase
{
    RECOVERY_EFFORT_RAISE = 0,      // 0: 五次轨迹站高
    RECOVERY_EFFORT_STABILIZE = 1,  // 1: 连续 0.50s 检查稳态
    RECOVERY_POSITION_PRELOAD = 2,  // 2: 锁存实际构型并预发 0.10s
    RECOVERY_SWITCHING = 3,         // 3: STRICT 原子切换进行中
    RECOVERY_POSITION_HOLD = 4,     // 4: 保持锁存构型 0.25s
    RECOVERY_POSITION_RETURN = 5,   // 5: 0.8s 五次轨迹回到 L_STAND 对应 IK
    RECOVERY_COMPLETE = 6,          // 6: 切换完成进入 BALANCE
    // v6.0：THRUST 失败且轮子仍接地时，不允许直接保持长腿/站高。
    // 先缩腿到安全高度，待姿态和角速度缓和后再重新站起。
    RECOVERY_FAIL_CROUCH = 7,       // 7: 失败接地缩腿
    RECOVERY_FAIL_STABILIZE = 8     // 8: 低位姿态稳定
};

inline const char* recovery_subphase_to_string(RecoverySubphase s)
{
    switch (s) {
        case RECOVERY_EFFORT_RAISE: return "EFFORT_RAISE";
        case RECOVERY_EFFORT_STABILIZE: return "EFFORT_STABILIZE";
        case RECOVERY_POSITION_PRELOAD: return "POSITION_PRELOAD";
        case RECOVERY_SWITCHING: return "SWITCHING";
        case RECOVERY_POSITION_HOLD: return "POSITION_HOLD";
        case RECOVERY_POSITION_RETURN: return "POSITION_RETURN";
        case RECOVERY_COMPLETE: return "COMPLETE";
        case RECOVERY_FAIL_CROUCH: return "FAIL_CROUCH";
        case RECOVERY_FAIL_STABILIZE: return "FAIL_STABILIZE";
        default: return "UNKNOWN";
    }
}

inline const char* state_to_string(JumpState s)
{
    switch (s) {
        case STATE_BALANCE: return "BALANCE";
        case STATE_PRE_JUMP: return "PRE_JUMP";
        case STATE_SQUAT: return "SQUAT";
        case STATE_THRUST: return "THRUST";
        case STATE_FLIGHT: return "FLIGHT";
        case STATE_TOUCHDOWN_BUFFER: return "TOUCHDOWN_BUFFER";
        case STATE_RECOVERY: return "RECOVERY";
        case STATE_STANDUP: return "STANDUP";
        case STATE_EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

} // namespace bbot_jump

class BBotVelocityJumpController : public rclcpp::Node
{
public:
    BBotVelocityJumpController()
        : Node("bbot_velocity_jump_controller")
    {
        gain_low_ = {-6.1624, -45.8436, -179.6985, -42.8109};
        gain_high_ = {-6.3650, -49.5719, -233.4004, -62.6391};
        current_gain_ = gain_high_;

        balance_offset_ = 0.038;
        cmd_scale_ = 0.043;
        wheel_radius_ = 0.07;
        max_cmd_x_ = 10.0;

        walk_speed_ = 0.50;
        turn_speed_ = 0.60;
        speed_ramp_time_ = 1.0;

        // ── jump-thrust-v5：滚动起跳准备参数 ──
        // 这些是首轮 Gazebo 试验值，不是辨识后的最优参数；全部可通过 ROS 参数覆盖。
        // PRE_JUMP 只建立稳定的接近速度；SQUAT 末段和 THRUST 再把速度推到离地目标。
        jump_forward_speed_ = this->declare_parameter<double>("jump_forward_speed", 0.50);
        jump_forward_speed_ = bbot_jump::clamp_value(jump_forward_speed_, 0.0, 0.70);
        jump_takeoff_forward_speed_ = this->declare_parameter<double>("jump_takeoff_forward_speed", 0.45);
        jump_takeoff_forward_speed_ = bbot_jump::clamp_value(
            jump_takeoff_forward_speed_, jump_forward_speed_, 0.85);
        thrust_forward_velocity_kp_ = this->declare_parameter<double>(
            "thrust_forward_velocity_kp", 0.80);
        thrust_forward_velocity_kp_ = bbot_jump::clamp_value(
            thrust_forward_velocity_kp_, 0.0, 2.0);

        // 机身前倾角仍作为几何工作点；真正的“斜前方”离地由下面的正 pitch_rate 目标建立。
        jump_pitch_offset_ = this->declare_parameter<double>("jump_pitch_offset", 0.075);
        jump_pitch_offset_ = bbot_jump::clamp_value(jump_pitch_offset_, 0.0, 0.16);
        jump_takeoff_pitch_rate_ = this->declare_parameter<double>(
            "jump_takeoff_pitch_rate", 0.45);
        jump_takeoff_pitch_rate_ = bbot_jump::clamp_value(
            jump_takeoff_pitch_rate_, 0.0, 0.80);
        jump_takeoff_pitch_rate_tolerance_ = this->declare_parameter<double>(
            "jump_takeoff_pitch_rate_tolerance", 0.18);
        jump_takeoff_pitch_rate_tolerance_ = bbot_jump::clamp_value(
            jump_takeoff_pitch_rate_tolerance_, 0.05, 0.35);
        thrust_pitch_rate_lead_time_ = this->declare_parameter<double>(
            "thrust_pitch_rate_lead_time", 0.07);
        thrust_pitch_rate_lead_time_ = bbot_jump::clamp_value(
            thrust_pitch_rate_lead_time_, 0.0, 0.12);

        // ── jump-thrust-v5.9：真实 CAD 水平落点规划 ──
        // 当前工程的 Kinematics 公开接口只有 inverse_kinematics(target_z, body_pitch)。
        // 本文件在控制器内部复用相同 CAD 几何，精确加入纵向 target_x，
        // 不再使用“等效腿长 + 只减髋角”的近似，也不修改 bbot_kinematics API。
        // target_x>0 表示机身位于轮子前方，等价于轮子相对机身向后布置。
        landing_wheel_back_bias_ = this->declare_parameter<double>(
            "landing_wheel_back_bias", 0.085);
        landing_wheel_back_bias_ = bbot_jump::clamp_value(
            landing_wheel_back_bias_, 0.0, 0.12);
        // 水平速度越大，允许轮子适度向前以承担一部分 capture；但该项只做修正，
        // 默认仍保持轮子在机身后方，触地后的主动轮控继续完成剩余捕获。
        landing_capture_gain_ = this->declare_parameter<double>(
            "landing_capture_gain", 0.08);
        landing_capture_gain_ = bbot_jump::clamp_value(
            landing_capture_gain_, 0.0, 0.60);
        landing_target_x_max_ = this->declare_parameter<double>(
            "landing_target_x_max", 0.10);
        landing_target_x_max_ = bbot_jump::clamp_value(
            landing_target_x_max_, 0.03, 0.14);
        landing_capture_height_ = this->declare_parameter<double>(
            "landing_capture_height", 0.40);
        landing_capture_height_ = bbot_jump::clamp_value(
            landing_capture_height_, 0.30, 0.50);
        landing_capture_speed_deadband_ = this->declare_parameter<double>(
            "landing_capture_speed_deadband", 0.08);
        landing_capture_speed_deadband_ = bbot_jump::clamp_value(
            landing_capture_speed_deadband_, 0.0, 0.25);

        // ── jump-thrust-v5.9：wheel-first 落地几何约束 ──
        // 小腿绝对角以“轮轴→膝、竖直向上=0”为定义。落地接近时禁止小腿接近水平，
        // 否则膝关节中心会降到接近轮轴高度，出现轮子和膝盖同时触地。
        landing_shank_abs_max_ = this->declare_parameter<double>(
            "landing_shank_abs_max", 0.75);
        landing_shank_abs_max_ = bbot_jump::clamp_value(
            landing_shank_abs_max_, 0.45, 1.05);
        // 膝关节中心相对轮轴至少保持此竖直高度；与上面的角度约束取更严格者。
        landing_knee_axis_clearance_min_ = this->declare_parameter<double>(
            "landing_knee_axis_clearance_min", 0.20);
        landing_knee_axis_clearance_min_ = bbot_jump::clamp_value(
            landing_knee_axis_clearance_min_, 0.10, 0.28);
        // 保护展腿必须在预计触地前预留一段稳定时间，避免固定 0.24 s 导致触地时轨迹尚未完成。
        landing_deploy_ready_margin_ = this->declare_parameter<double>(
            "landing_deploy_ready_margin", 0.055);
        landing_deploy_ready_margin_ = bbot_jump::clamp_value(
            landing_deploy_ready_margin_, 0.03, 0.10);
        landing_protective_deploy_min_ = this->declare_parameter<double>(
            "landing_protective_deploy_min", 0.10);
        landing_protective_deploy_min_ = bbot_jump::clamp_value(
            landing_protective_deploy_min_, 0.07, 0.16);

        // v5.6：FLIGHT 不再把起跳前倾姿态一步拉回静态 balance_offset。
        // 目标着陆姿态保留少量前倾，并用平滑参考在整个腾空前半段完成回正。
        flight_landing_pitch_bias_ = this->declare_parameter<double>(
            "flight_landing_pitch_bias", 0.055);
        flight_landing_pitch_bias_ = bbot_jump::clamp_value(
            flight_landing_pitch_bias_, 0.0, 0.10);
        flight_pitch_transition_duration_ = this->declare_parameter<double>(
            "flight_pitch_transition_duration", 0.28);
        flight_pitch_transition_duration_ = bbot_jump::clamp_value(
            flight_pitch_transition_duration_, 0.15, 0.40);
        // 试验/调参值：短腾空时不能用过大的负参考角速度强行把起跳前倾拉回，
        // 否则与保护展腿的内部反作用叠加，会在触地前反向后仰。
        flight_landing_pitch_rate_max_ = this->declare_parameter<double>(
            "flight_landing_pitch_rate_max", 0.30);
        flight_landing_pitch_rate_max_ = bbot_jump::clamp_value(
            flight_landing_pitch_rate_max_, 0.10, 0.60);

        pre_jump_timeout_ = this->declare_parameter<double>("pre_jump_timeout", bbot_jump::kDefaultPreJumpTimeout);
        pre_jump_timeout_ = bbot_jump::clamp_value(pre_jump_timeout_, 0.5, 4.0);
        pre_jump_stable_duration_ = this->declare_parameter<double>("pre_jump_stable_duration", 0.08);
        pre_jump_stable_duration_ = bbot_jump::clamp_value(pre_jump_stable_duration_, 0.03, 0.30);
        pre_jump_speed_tolerance_ = this->declare_parameter<double>("pre_jump_speed_tolerance", 0.06);
        pre_jump_speed_tolerance_ = bbot_jump::clamp_value(pre_jump_speed_tolerance_, 0.01, 0.20);
        pre_jump_pitch_tolerance_ = this->declare_parameter<double>("pre_jump_pitch_tolerance", 0.035);
        pre_jump_pitch_tolerance_ = bbot_jump::clamp_value(pre_jump_pitch_tolerance_, 0.01, 0.10);
        pre_jump_rate_limit_ = this->declare_parameter<double>("pre_jump_rate_limit", 0.30);
        pre_jump_rate_limit_ = bbot_jump::clamp_value(pre_jump_rate_limit_, 0.05, 0.80);
        jump_pitch_ref_ = balance_offset_ + jump_pitch_offset_;
        active_jump_pitch_ref_ = balance_offset_;

        L_MIN_ = 0.30;
        L_MAX_ = 0.50;
        L_STAND_ = 0.50;

        target_height_ = L_STAND_;
        current_height_ = target_height_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0; // 0.05 m/s

        // ── 跳跃核心参数 ──
        // 不做过深、过快的下蹲：位置控制器切到 Effort 的短暂过渡期间，
        // 0.30 m / 0.15 s 会让机身先失去支撑再来不及推地。
        L_SQUAT_ = 0.34;          // 下蹲蓄力高度 [m]
        T_SQUAT_ = 0.50;          // 下蹲过渡时间 [s]，与旧稳定控制器一致

        T_THRUST_ = 0.10;        // 推地规划时间 [s]
        H_TAKEOFF_ = 0.475;       // 离地目标高度 [m]
        V_TAKEOFF_ = 2.30;        // 离地初速度 [m/s]
        // 当前推地主要由膝关节承担，实测膝力矩约 57 Nm 时会产生很大的
        // 后仰反作用。髋部必须快速提供足够的基座姿态力矩，而不是等倾角
        // 已经扩大后才缓慢纠正。
        K_BODY_P_THRUST_ = 70.0;  // 推地姿态刚度 [Nm/rad]
        K_BODY_D_THRUST_ = 12.0;  // 推地姿态阻尼 [Nm*s/rad]
        TAU_HIP_BODY_MAX_ = 20.0; // 髋关节姿态补偿力矩限幅 [Nm]
        // 试验/调参值：膝关节快速伸展会通过内部反作用力矩把机身推向后仰。
        // 这里依据推地期望膝速度提前给髋关节少量反作用补偿，不能替代
        // 后仰门控，也不能继续增大到让髋关节主导腿部构型。
        K_LEG_REACTION_FF_THRUST_ = this->declare_parameter<double>(
            "thrust_leg_reaction_ff_gain", 0.8);
        K_LEG_REACTION_FF_THRUST_ = bbot_jump::clamp_value(
            K_LEG_REACTION_FF_THRUST_, 0.0, 3.0);
        TAU_LEG_REACTION_FF_MAX_ = this->declare_parameter<double>(
            "thrust_leg_reaction_ff_max", 8.0);
        TAU_LEG_REACTION_FF_MAX_ = bbot_jump::clamp_value(
            TAU_LEG_REACTION_FF_MAX_, 0.0, 12.0);
        K_BODY_P_BUFFER_ = 55.0;  // 缓冲阶段姿态刚度 [Nm/rad]
        K_BODY_D_BUFFER_ = 15.0;  // 缓冲阶段姿态阻尼 [Nm*s/rad]

        L_RETRACT_ = 0.30;        // v6.2：保持明显收腿目标；关键修复是强制真正执行TUCK
        L_TOUCH_ = 0.50;         // 腾空展腿着陆高度 [m]
        L_BUFFER_SETTLE_ = this->declare_parameter<double>("landing_buffer_height", 0.34);
        L_BUFFER_SETTLE_ = bbot_jump::clamp_value(
            L_BUFFER_SETTLE_, L_SQUAT_ + 0.01, L_TOUCH_ - 0.02);
        // 试验值：触地时先保持空中末帧的实际关节构型，再平滑交给缓冲 IK。
        // 这避免保护展腿尚未结束时，缓冲控制器又给髋关节一个位置阶跃。
        landing_joint_handoff_duration_ = this->declare_parameter<double>(
            "landing_joint_handoff_duration", 0.16);
        landing_joint_handoff_duration_ = bbot_jump::clamp_value(
            landing_joint_handoff_duration_, 0.08, 0.30);
        // 放慢收腿，避免腿部反作用角动量把箱体继续推向后仰。
        T_FLIGHT_TUCK_ = 0.10;   // v6.2：低跳中快速完成明显收腿 [s]
        T_FLIGHT_APEX_ = 0.30;    // 高跳时仍可在顶点附近开始展腿；低跳由剩余时间提前触发
        T_FLIGHT_EXTEND_ = 0.090; // v6.2：给收腿腾出时间，同时保留wheel-first展腿 [s]
        T_PROTECTIVE_DEPLOY_ = 0.24; // 超标离地后，从离地初速度连续过渡到着陆构型
        T_FLIGHT_TIMEOUT_ = 0.60; // 腾空超时保护阈值 [s]
        PITCH_FLIGHT_GUARD_ = 0.45; // 腾空姿态保护阈值 [rad]

        // 落地速度约 1.5 m/s 时，原 160 N/腿上限与缓慢建力不足以在
        // 有效腿程内吸收动能。允许用户已放宽的关节力矩用于触地承重。
        K_Z_BUFFER_ = 450.0;      // 单腿垂直刚度 [N/m]
        D_Z_BUFFER_ = 75.0;       // 单腿垂直阻尼 [N*s/m]
        F_Z_BUFFER_MAX_ = 240.0;  // 单腿最大缓冲支撑力 [N]
        body_mass_ = this->declare_parameter<double>("body_mass", 9.5);
        TOTAL_MASS_ = 4.00 + 1.60 + 2.40 + body_mass_; // 17.5 kg
        auto runtime_robot_params = kinematics_.get_params();
        runtime_robot_params.m3 = body_mass_;
        runtime_robot_params.M_total = TOTAL_MASS_;
        kinematics_.set_params(runtime_robot_params);
        height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;

        position_proportional_gain_ = this->declare_parameter<double>("position_proportional_gain", 0.3);
        enable_position_handoff_ = this->declare_parameter<bool>("enable_position_handoff", true);
        handoff_joint_error_limit_ = this->declare_parameter<double>("handoff_joint_error_limit", 0.12);
        handoff_height_drop_limit_ = this->declare_parameter<double>("handoff_height_drop_limit", 0.04);
        handoff_pitch_error_limit_ = this->declare_parameter<double>("handoff_pitch_error_limit", 0.12);
        handoff_z_dot_limit_ = this->declare_parameter<double>("handoff_z_dot_limit", 0.20);

        // ── jump-thrust-v6.0：失败接地恢复 + 可恢复推地姿态门控 ──
        // THRUST 失败但轮子仍接地时，先把长腿缩到较低、可恢复的工作高度，
        // 而不是直接进入 L_STAND=0.50m 的稳态支撑。
        failed_thrust_crouch_height_ = this->declare_parameter<double>(
            "failed_thrust_crouch_height", 0.38);
        failed_thrust_crouch_height_ = bbot_jump::clamp_value(
            failed_thrust_crouch_height_, L_SQUAT_ + 0.02, L_STAND_ - 0.05);
        failed_thrust_crouch_duration_ = this->declare_parameter<double>(
            "failed_thrust_crouch_duration", 0.32);
        failed_thrust_crouch_duration_ = bbot_jump::clamp_value(
            failed_thrust_crouch_duration_, 0.20, 0.55);
        failed_thrust_settle_duration_ = this->declare_parameter<double>(
            "failed_thrust_settle_duration", 0.18);
        failed_thrust_settle_duration_ = bbot_jump::clamp_value(
            failed_thrust_settle_duration_, 0.08, 0.40);
        // 只有真正的“硬姿态失稳”才进入 block；单纯 gyro 短暂转负只降推力、不锁死轨迹。
        thrust_hard_block_timeout_ = this->declare_parameter<double>(
            "thrust_hard_block_timeout", 0.12);
        thrust_hard_block_timeout_ = bbot_jump::clamp_value(
            thrust_hard_block_timeout_, 0.06, 0.20);

        // 目标离地速度推地轨迹参数；可由 ROS 参数或批量优化器覆盖。
        jump_height_target_ = this->declare_parameter<double>("jump_height", 0.20);
        takeoff_velocity_override_ = this->declare_parameter<double>("takeoff_velocity", 0.0);
        thrust_duration_ = this->declare_parameter<double>("thrust_duration", 0.24);
        thrust_peak_ratio_ = this->declare_parameter<double>("thrust_peak_ratio", 2.30);
        // 试验值：名义伸腿轨迹结束仍未离地时保留的每腿额外推力比例。
        // 该脉冲只在姿态门控允许、且离地速度尚未达到目标时生效。
        grounded_launch_boost_ratio_ = this->declare_parameter<double>(
            "grounded_launch_boost_ratio", 0.75);
        thrust_shape_early_ = this->declare_parameter<double>("thrust_shape_early", 0.95);
        thrust_shape_late_ = this->declare_parameter<double>("thrust_shape_late", 0.75);
        thrust_velocity_kp_ = this->declare_parameter<double>("thrust_velocity_kp", 8.0);
        thrust_torque_margin_ = this->declare_parameter<double>("thrust_torque_margin", 0.95);
        // 仅 Gazebo 启动文件默认开启；离开 THRUST 或关闭仿真时钟后恢复75/60。
        sim_relax_thrust_limits_ = this->declare_parameter<bool>("sim_relax_thrust_limits", false);
        // 推地轨迹结束不等于轮子已离地。留出额外时间给速度闭环维持
        // 有效接地推力，直到失重确认；该超时仍是未离地时的保护上限。
        thrust_timeout_ = this->declare_parameter<double>("thrust_timeout", 0.60);
        target_takeoff_velocity_ = takeoff_velocity_override_ > 0.0 ?
            takeoff_velocity_override_ : std::sqrt(2.0 * 9.81 * std::max(0.01, jump_height_target_));
        thrust_duration_ = bbot_jump::clamp_value(thrust_duration_, 0.08, 0.32);
        thrust_timeout_ = bbot_jump::clamp_value(thrust_timeout_, thrust_duration_ + 0.12, 0.75);
        thrust_peak_ratio_ = bbot_jump::clamp_value(thrust_peak_ratio_, 1.0, 4.0);
        grounded_launch_boost_ratio_ = bbot_jump::clamp_value(
            grounded_launch_boost_ratio_, 0.0, 1.50);
        thrust_shape_early_ = bbot_jump::clamp_value(thrust_shape_early_, 0.05, 2.0);
        thrust_shape_late_ = bbot_jump::clamp_value(thrust_shape_late_, 0.05, 2.0);
        thrust_velocity_kp_ = bbot_jump::clamp_value(thrust_velocity_kp_, 0.0, 30.0);
        thrust_torque_margin_ = bbot_jump::clamp_value(thrust_torque_margin_, 0.80, 1.0);
        // 实测正轮速会把机体带向后方；腾空姿态反馈必须使用同向符号，
        // 才能在负俯仰/负角速度时给出负轮速进行回正。
        air_wheel_sign_ = this->declare_parameter<double>("air_wheel_sign", 1.0);

        RCLCPP_INFO(this->get_logger(),
                    "[jump-thrust-v6.2] 强制实际状态TUCK / 去除round-trip误拦截 / 低跳立即再展腿");
        RCLCPP_INFO(this->get_logger(),
                    "[rolling-jump-config] approach_v=%.3f takeoff_v=%.3f m/s pitch_ref=%.3f rad "
                    "takeoff_pitch_rate=%.3f rad/s prepare_timeout=%.2fs",
                    jump_forward_speed_, jump_takeoff_forward_speed_, jump_pitch_ref_,
                    jump_takeoff_pitch_rate_, pre_jump_timeout_);
        RCLCPP_INFO(this->get_logger(),
                    "[landing-placement-config] wheel_back_bias=%.3fm capture_gain=%.2f target_x_max=%.3fm height=%.3fm deadband=%.3fm/s",
                    landing_wheel_back_bias_, landing_capture_gain_, landing_target_x_max_,
                    landing_capture_height_, landing_capture_speed_deadband_);
        RCLCPP_INFO(this->get_logger(),
                    "[flight-landing-attitude] pitch_ref=balance+%.3f rad, transition=%.2fs, rate_limit=%.2f rad/s; 当前关节安全时25ms后可直接TUCK",
                    flight_landing_pitch_bias_, flight_pitch_transition_duration_,
                    flight_landing_pitch_rate_max_);
        RCLCPP_INFO(this->get_logger(),
                    "[flight-tuck-config] retract=%.3fm tuck=%.3fs extend=%.3fs apex=%.3fs",
                    L_RETRACT_, T_FLIGHT_TUCK_, T_FLIGHT_EXTEND_, T_FLIGHT_APEX_);
        RCLCPP_INFO(this->get_logger(),
                    "[v6-recovery-config] failed_crouch=%.3fm / %.2fs settle=%.2fs hard_block_timeout=%.2fs",
                    failed_thrust_crouch_height_, failed_thrust_crouch_duration_,
                    failed_thrust_settle_duration_, thrust_hard_block_timeout_);

        RCLCPP_INFO(this->get_logger(),"[takeoff-sync-v6.3] 同时刻轮底几何 / 独立采样确认 / 12mm进入6mm退出");

        RCLCPP_INFO(this->get_logger(),"[ground-handoff-v6.4] 有限前倾参考 / 全程推地姿态轮控 / 着陆轮控提前衔接");

        RCLCPP_INFO(this->get_logger(),"[ground-sampling-v6.5] 仿真时钟门控 / 持续接触确认 / 地面离散反馈 / 世界速度修正");

        RCLCPP_INFO(this->get_logger(),"[launch-capture-v6.6] 推地收尾锁存 / 明确净空确认 / 落地持续捕获与再捕获");

        RCLCPP_INFO(this->get_logger(),"[centroidal-v6.7] 整机质心速度推地 / 重心相对轮轴捕获 / 保留空中离散关节反馈");

        RCLCPP_INFO(this->get_logger(),
            "[effort-allocation-v6.8] 有符号推力预算 / 地面腿部重力补偿 / 仿真推地放宽=%d (150Nm)",
            (sim_relax_thrust_limits_ && this->get_parameter("use_sim_time").as_bool()) ? 1 : 0);

        RCLCPP_INFO(this->get_logger(),
            "[com-drive-v6.12] Effort稳态可行驶/再次跳跃 / 推地单向构型P保留D");

        // 日志路径初始化
        const char * home_dir = getenv("HOME");
        data_path_ = std::string(home_dir ? home_dir : "/home/admin") + "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/";
        open_log_files();

        // ── 订阅话题 ──
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BBotVelocityJumpController::imu_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/bbot/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                if (!std::isfinite(msg->pose.pose.position.x) ||
                    !std::isfinite(msg->pose.pose.position.y) ||
                    !std::isfinite(msg->pose.pose.position.z)) return;
                odom_base_position_ = {msg->pose.pose.position.x,
                    msg->pose.pose.position.y,msg->pose.pose.position.z};
                gazebo_world_z_ = msg->pose.pose.position.z;
                last_world_odom_time_ = this->now().seconds();
                takeoff_odom_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
                const auto & orientation = msg->pose.pose.orientation;
                tf2::Quaternion body_q(orientation.x, orientation.y, orientation.z, orientation.w);
                takeoff_odom_pose_valid_ = std::isfinite(body_q.length2()) && body_q.length2()>1e-8;
                if (takeoff_odom_pose_valid_) {
                    body_q.normalize();
                    const tf2::Matrix3x3 rotation(body_q);
                    for (int row=0;row<3;++row) for (int col=0;col<3;++col)
                        odom_body_rotation_(row,col)=rotation[row][col];
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(body_q).getRPY(roll,pitch,yaw);
                    takeoff_odom_pitch_ = -roll;
                    const auto vertical_axis = tf2::Matrix3x3(body_q).getRow(2);
                    odom_world_z_in_body_ = {vertical_axis.x(),vertical_axis.y(),vertical_axis.z()};
                }
                // Gazebo的3D twist在机身系且内部已滤波；不能直接把z当世界vz。
                // 按世界位置差分只保留一个odom区间延迟，不再串联0.25慢低通。
                odom_twist_z_diag_ = msg->twist.twist.linear.z;
                world_pose_velocity_.update(takeoff_odom_stamp_,
                    {msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z});
                const bool velocity_valid = world_pose_velocity_.valid();
                world_xy_dot_filter_initialized_ = velocity_valid;
                world_z_dot_filter_initialized_ = velocity_valid;
                const auto & world_velocity = world_pose_velocity_.velocity();
                gazebo_world_x_dot_ = velocity_valid ? world_velocity[0] : 0.0;
                gazebo_world_y_dot_ = velocity_valid ? world_velocity[1] : 0.0;
                gazebo_world_z_dot_ = velocity_valid ? world_velocity[2] : 0.0;
                odom_received_ = true;
                if (current_state_ != bbot_jump::STATE_BALANCE) {
                    if (!world_height_valid_for_jump_ &&
                        current_state_ == bbot_jump::STATE_SQUAT) {
                        thrust_start_world_z_ = gazebo_world_z_;
                        max_world_z_during_jump_ = gazebo_world_z_;
                        world_height_valid_for_jump_ = true;
                    }
                    max_world_z_during_jump_ = std::max(
                        max_world_z_during_jump_, gazebo_world_z_);
                }
            });

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&BBotVelocityJumpController::joint_state_callback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                target_speed_const_ = msg->linear.x;
                target_yaw_rate_ = msg->angular.z;
                if (std::abs(msg->linear.x) < 0.001 && std::abs(msg->angular.z) < 0.001) {
                    if (was_moving_) {
                        target_x_ = x_;
                        was_moving_ = false;
                    }
                }
            });

        target_height_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target_height", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                if (current_state_ == bbot_jump::STATE_BALANCE) {
                    target_height_ = bbot_jump::clamp_value(msg->data, L_MIN_, L_MAX_);
                }
            });

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handle_mode_command(msg->data);
            });

        jump_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/jump_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "jump" || msg->data == "J" || msg->data == "j") {
                    trigger_jump();
                }
            });

        // ── 发布话题 ──
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);

        leg_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        leg_effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_effort_controller/commands", 10);

        // ── 控制器动态切换服务客户端 ──
        switch_ctrl_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        effort_mode_active_ = false;

        // 主控制循环：200Hz (5ms)
        timer_ = this->create_wall_timer(5ms, std::bind(&BBotVelocityJumpController::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=====================================================");
        RCLCPP_INFO(this->get_logger(), "  BBot 跳跃与 LQR 复合控制器已启动 (200Hz)");
        RCLCPP_INFO(this->get_logger(), "  支持按键: J (跳跃), W/A/S/D (遥控), Q/E (变高度), R (起立), X (停机)");
        RCLCPP_INFO(this->get_logger(), "=====================================================");
    }

    ~BBotVelocityJumpController()
    {
        close_log_files();
    }

private:
    int num_ = 0;
    // ── 状态机相关 ──
    bbot_jump::JumpState current_state_ = bbot_jump::STATE_BALANCE;
    double state_start_time_ = 0.0;
    bbot_jump::QuinticTrajectory quintic_traj_;

    // ── 传感器与里程计数据 ──
    bool imu_received_ = false;
    bool wheel_origin_set_ = false;
    bool was_moving_ = false;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double pitch_rate_raw_ = 0.0;
    double pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.15;
    bbot_jump::TorsoImuObserver torso_imu_;
    bbot_jump::TorsoPitchTorque torso_torque_;
    double torso_control_rate_ = 0.0;
    double torso_control_horizon_ = 0.030;
    double hip_common_before_allocation_ = 0.0;
    double hip_differential_torque_ = 0.0;
    bool torso_imu_fresh_ = false;

    double acc_z_raw_ = 0.0;
    double acc_z_filt_ = 9.81;
    double acc_z_prev_ = 9.81;
    bool acc_z_filter_init_ = false;

    double x_ = 0.0;
    double x_dot_ = 0.0;
    double x_dot_raw_ = 0.0;
    double x_dot_filt_ = 0.0;
    bool x_dot_filter_init_ = false;
    double x_dot_alpha_ = 0.08;

    double left_wheel_pos_ = 0.0, right_wheel_pos_ = 0.0;
    double left_wheel_vel_ = 0.0, right_wheel_vel_ = 0.0;
    double left_wheel_pos_origin_ = 0.0, right_wheel_pos_origin_ = 0.0;

    double hip_pos_left_ = 0.0, knee_pos_left_ = 0.0;
    double hip_vel_left_ = 0.0, knee_vel_left_ = 0.0;
    double hip_pos_right_ = 0.0, knee_pos_right_ = 0.0;
    double hip_vel_right_ = 0.0, knee_vel_right_ = 0.0;
    double hip_effort_left_ = 0.0, knee_effort_left_ = 0.0;
    double hip_effort_right_ = 0.0, knee_effort_right_ = 0.0;
    double hip_cmd_left_ = 0.0, knee_cmd_left_ = 0.0;
    double hip_cmd_right_ = 0.0, knee_cmd_right_ = 0.0;

    // 位置控制器目标缓存
    double hip_pos_cmd_left_ = 0.0, knee_pos_cmd_left_ = 0.0;
    double hip_pos_cmd_right_ = 0.0, knee_pos_cmd_right_ = 0.0;

    // 垂直方向状态估计 (正运动学)
    double current_z_ = 0.40;
    double current_z_dot_ = 0.0;
    double current_z_dot_raw_ = 0.0;
    double prev_z_ = 0.40;
    rclcpp::Time prev_z_time_;
    bool z_dot_filter_init_ = false;
    double prev_q_hip_des_ = 0.0;
    double prev_q_knee_des_ = 0.0;
    double state_start_z_ = 0.30;
    double flight_start_z_ = 0.40;
    bool thrust_trajectory_initialized_ = false;
    double thrust_force_per_leg_ = 0.0;
    double thrust_force_command_per_leg_ = 0.0;
    double last_thrust_force_request_ = 0.0;
    double last_thrust_force_limit_ = 0.0;
    double last_tau_body_per_hip_ = 0.0;
    bbot_jump::JointVector ground_support_previous_ = bbot_jump::JointVector::Zero();
    bbot_jump::JointVector last_support_feedforward_ = bbot_jump::JointVector::Zero();
    bool ground_support_initialized_ = false;
    double ground_pd_horizon_ = 0.0;
    std::array<double,4> ground_pd_feedback_{};
    bool last_wheels_airborne_ = false;
    double flight_ff_force_start_ = 0.0;
    bool flight_trajectory_initialized_ = false;
    double last_effort_tau_hip_left_ = 0.0;
    double last_effort_tau_knee_left_ = 0.0;
    double last_effort_tau_hip_right_ = 0.0;
    double last_effort_tau_knee_right_ = 0.0;
    double last_effort_time_ = -1.0;
    bool effort_slew_initialized_ = false;
    // 离地候选需要连续多个周期成立，避免腿部快速伸展造成单帧误判。
    int airborne_confidence_count_ = 0;
    int velocity_reached_count_ = 0;
    int balance_settle_count_ = 0;
    bool touchdown_buffer_initialized_ = false;
    bool protective_landing_ = false;

    // ── THRUST 姿态门控与推力调控变量 ──
    bbot_jump::ThrustRelease thrust_release_;
    double touchdown_catch_stable_time_ = 0.0;
    double thrust_motion_elapsed_ = 0.0;
    double thrust_extension_scale_ = 1.0;
    double joint_sample_time_ = -1.0;
    double joint_sample_period_ = 0.010;
    double air_pd_horizon_ = 0.025;
    std::array<double, 4> air_pd_raw_{};
    std::array<double, 4> air_pd_discrete_{};
    bool thrust_attitude_blocked_ = false;
    bool thrust_gate_has_opened_ = false;
    int thrust_attitude_stable_count_ = 0;
    int thrust_block_recovery_count_ = 0;
    double thrust_block_elapsed_ = 0.0;
    double thrust_hard_block_timeout_ = 0.12;

    // ── FLIGHT 内部子阶段与空中轮速修正 ──
    bbot_jump::FlightSubphase flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST;
    double attitude_arrest_start_time_ = 0.0;
    int attitude_arrest_stable_count_ = 0;
    double tuck_start_time_ = 0.0;
    double extend_start_time_ = 0.0;
    double tuck_start_z_ = 0.40;
    bbot_jump::QuinticTrajectory tuck_traj_;
    bbot_jump::QuinticTrajectory extend_traj_;
    bbot_jump::QuinticTrajectory protective_deploy_traj_;
    bbot_jump::QuinticTrajectory arrest_hip_left_traj_;
    bbot_jump::QuinticTrajectory arrest_knee_left_traj_;
    bbot_jump::QuinticTrajectory arrest_hip_right_traj_;
    bbot_jump::QuinticTrajectory arrest_knee_right_traj_;
    std::array<bbot_jump::QuinticTrajectory, 4> normal_flight_joint_traj_;
    std::array<double, 4> joint_velocity_cmd_{};
    double ground_height_offset_ = 0.0;
    bbot_jump::JointPoseHistory takeoff_joint_history_;
    bbot_jump::TakeoffConfirmation takeoff_confirmation_;
    bbot_jump::TouchdownConfirmation touchdown_confirmation_;
    double takeoff_odom_stamp_ = -1.0;
    double takeoff_odom_pitch_ = 0.0;
    Eigen::Vector3d odom_world_z_in_body_ = Eigen::Vector3d::UnitZ();
    bbot_jump::CentroidalHeightObserver centroidal_height_;
    bbot_jump::CentroidalWorldObserver centroidal_world_;
    Eigen::Vector3d odom_base_position_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d odom_body_rotation_ = Eigen::Matrix3d::Identity();
    double capture_com_velocity_ = 0.0;
    bool capture_world_valid_ = false;
    bool capture_world_active_ = false;
    bool recovery_ready_ = false;
    bool effort_jump_cycle_ = false;
    double recovery_drive_ref_ = 0.0;
    double recovery_yaw_ref_ = 0.0;
    double capture_world_target_ = 0.0;
    bbot_jump::CentroidalBalanceState centroidal_balance_;
    bool centroidal_velocity_valid_ = false;
    double thrust_vertical_velocity_ = 0.0;
    bool takeoff_odom_pose_valid_ = false;
    bool takeoff_rise_observed_ = false;
    bool takeoff_geometry_aligned_ = false;
    double takeoff_clearance_left_ = -1.0;
    double takeoff_clearance_right_ = -1.0;
    double last_world_descent_time_ = -1.0;
    double last_world_odom_time_ = -1.0;
    double wheel_clearance_ = 0.0;
    bool contact_window_ = false;
    bbot_jump::QuinticTrajectory protective_hip_left_traj_;
    bbot_jump::QuinticTrajectory protective_knee_left_traj_;
    bbot_jump::QuinticTrajectory protective_hip_right_traj_;
    bbot_jump::QuinticTrajectory protective_knee_right_traj_;
    bool tuck_started_ = false;
    double tuck_start_timestamp_ = 0.0;
    bool protective_deploy_initialized_ = false;
    double protective_deploy_start_time_ = 0.0;
    double protective_deploy_q_hip_left_ = 0.0;
    double protective_deploy_q_knee_left_ = 0.0;
    double protective_deploy_q_hip_right_ = 0.0;
    double protective_deploy_q_knee_right_ = 0.0;
    double air_wheel_sign_ = 1.0;
    double air_wheel_cmd_raw_ = 0.0;

    double recovery_x_ref_ = 0.0;        // 落地恢复阶段位置参考
    double recovery_hold_height_ = 0.40; // 恢复初期承重保持高度
    bool recovery_descent_started_ = false;
    // v6.0：失败接地恢复参数。FAIL_CROUCH 阶段允许 hip/knee 同时跟随高度 IK，
    // 避免旧 RECOVERY 固定 hip reference 导致“膝在缩、髋仍顶着长腿”的构型冲突。
    double failed_thrust_crouch_height_ = 0.38;
    double failed_thrust_crouch_duration_ = 0.32;
    double failed_thrust_settle_duration_ = 0.18;
    double fail_recovery_stable_timer_ = 0.0;
    bool recovery_follow_height_ik_ = false;
    double buffer_force_per_leg_ = 0.0;  // 触地缓冲滤波支撑力 [N]
    double height_force_per_leg_ = 0.0;  // 恢复/稳态单腿滤波支撑力 [N]
    bool height_force_initialized_ = false;
    int recovery_stable_count_ = 0;      // 恢复连续稳定计数
    bool post_landing_balance_soft_start_ = false; // 落地后平衡软接管标志
    bool post_landing_gyro_reduced_ = false;       // 落地后陀螺仪增益缩放标志
    bool post_landing_effort_support_ = false;     // 跳后保持力矩控制模式
    bool post_landing_position_handoff_requested_ = false;
    int post_landing_position_settle_count_ = 0;
    double post_landing_position_hold_until_ = 0.0;
    double post_landing_position_hip_cmd_left_ = 0.0;
    double post_landing_position_knee_cmd_left_ = 0.0;
    double post_landing_position_hip_cmd_right_ = 0.0;
    double post_landing_position_knee_cmd_right_ = 0.0;
    double position_switch_time_ = -1.0;
    bool post_landing_translation_feedback_ = false; // 落地平移反馈保护标志
    double balance_entry_time_ = 0.0;
    double last_wheel_cmd_x_ = 0.0;

    // ── 触地检测计数器 / 捕获状态 ──
    int touchdown_knee_effort_count_ = 0;
    int touchdown_stable_count_ = 0;
    bool touchdown_catch_active_ = true;
    int touchdown_catch_stable_count_ = 0;
    double touchdown_settle_start_time_ = -1.0;  // CATCH 释放后 PREPARE/BRAKE 阶段计时起点
    bool touchdown_brake_active_ = false;        // false: PREPARE_BRAKE, true: BRAKE
    int touchdown_brake_ready_count_ = 0;
    double touchdown_brake_start_time_ = -1.0;
    double touchdown_brake_cmd_ref_ = 0.0;       // BRAKE 阶段逐步下降的后退轮速目标幅值
    double recovery_hip_reference_ = 0.0;
    std::string touchdown_phase_diag_ = "";
    double touchdown_capture_diag_ = 0.0;
    double touchdown_cmd_target_diag_ = 0.0;
    double touchdown_x_error_diag_ = 0.0;
    double touchdown_joint_handoff_start_time_ = -1.0;
    double touchdown_hip_left_start_ = 0.0;
    double touchdown_knee_left_start_ = 0.0;
    double touchdown_hip_right_start_ = 0.0;
    double touchdown_knee_right_start_ = 0.0;

    // ── 控制参数 ──
    bbot_jump::LQRGain gain_low_;
    bbot_jump::LQRGain gain_high_;
    bbot_jump::LQRGain current_gain_;

    double balance_offset_;
    double post_landing_pitch_ref_ = 0.034; // 落地后静态俯仰平衡参考角 [rad]
    double cmd_scale_;
    double wheel_radius_;
    double max_cmd_x_;

    double target_speed_const_ = 0.0;
    double target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_;
    double turn_speed_;
    double speed_ramp_time_;
    double target_x_ = 0.0;
    double vel_integral_ = 0.0;

    // ── jump-thrust-v5：滚动起跳工作点 ──
    double jump_forward_speed_ = 0.30;          // PRE_JUMP / SQUAT 接近速度 [m/s]
    double jump_takeoff_forward_speed_ = 0.45;  // THRUST 离地前向速度目标 [m/s]
    double thrust_forward_velocity_kp_ = 0.80;  // THRUST 前向速度误差补偿
    double jump_pitch_offset_ = 0.075;
    double jump_pitch_ref_ = 0.113;
    double active_jump_pitch_ref_ = 0.038;
    double jump_takeoff_pitch_rate_ = 0.45;     // 正值=继续向前旋转 [rad/s]
    double jump_takeoff_pitch_rate_tolerance_ = 0.18;
    double thrust_pitch_rate_lead_time_ = 0.07;
    double active_jump_pitch_rate_ref_ = 0.0;

    // 空中落点规划。inverse_kinematics 的 target_x>0 表示“机身在轮子前方”，
    // 即轮子相对机身后移。速度 capture 项只会减少这部分后移量。
    double landing_wheel_back_bias_ = 0.085;
    double landing_capture_gain_ = 0.08;
    double landing_target_x_max_ = 0.10;
    double landing_capture_height_ = 0.40;
    double landing_capture_speed_deadband_ = 0.08;
    double landing_shank_abs_max_ = 0.75;
    double landing_knee_axis_clearance_min_ = 0.20;
    double landing_deploy_ready_margin_ = 0.055;
    double landing_protective_deploy_min_ = 0.10;
    double landing_target_shank_abs_ = 0.0;
    double landing_target_knee_axis_clearance_ = 0.0;
    double landing_capture_vx_ = 0.0;
    double landing_capture_raw_offset_ = 0.0;
    double landing_capture_offset_ = 0.0;      // 速度导致的前置修正量
    double landing_target_x_ = 0.0;            // 真正传给 IK 的第三参数
    double landing_capture_comp_ = 0.0;        // 仅诊断：atan2(target_x, L_TOUCH)
    double landing_capture_omega_ = 0.0;
    double landing_forward_axis_x_ = 1.0;
    double landing_forward_axis_y_ = 0.0;
    double landing_forward_direction_sign_ = 1.0;
    bool landing_forward_axis_valid_ = false;
    bool landing_capture_planned_ = false;

    // v5.6 空中姿态参考：从真实离地角平滑回到略前倾的着陆工作点。
    double flight_landing_pitch_bias_ = 0.055;
    double flight_pitch_transition_duration_ = 0.28;
    double flight_landing_pitch_rate_max_ = 0.30;
    double flight_pitch_ref_start_ = 0.038;
    double flight_air_pitch_ref_diag_ = 0.038;
    double flight_air_pitch_rate_ref_diag_ = 0.0;
    bool protective_deploy_rate_limited_ = false;
    int protective_deploy_replan_count_ = 0; // v6.1：限幅保护展腿允许继续追剩余着陆构型

    double pre_jump_timeout_ = bbot_jump::kDefaultPreJumpTimeout;
    double pre_jump_stable_duration_ = 0.08;
    double pre_jump_speed_tolerance_ = 0.06;
    double pre_jump_pitch_tolerance_ = 0.035;
    double pre_jump_rate_limit_ = 0.30;
    double pre_jump_stable_timer_ = 0.0;
    double pre_jump_hold_height_ = 0.50;
    double squat_pitch_start_ref_ = 0.038;
    bool pre_jump_balance_handoff_ = false;
    double takeoff_forward_speed_ = 0.0;
    double landing_wheel_ground_blend_ = 0.0;
    double air_wheel_baseline_ = 0.0;

    double current_height_ = 0.40;
    double target_height_ = 0.40;
    double leg_transition_speed_;
    double L_MIN_;
    double L_MAX_;
    double L_STAND_;

    // 跳跃规划参数
    double L_SQUAT_;
    double T_SQUAT_;
    double T_THRUST_;
    double H_TAKEOFF_;
    double V_TAKEOFF_;
    double K_BODY_P_THRUST_;
    double K_BODY_D_THRUST_;
    double TAU_HIP_BODY_MAX_;
    double K_LEG_REACTION_FF_THRUST_;
    double TAU_LEG_REACTION_FF_MAX_;
    double last_thrust_reaction_ff_ = 0.0;
    double K_BODY_P_BUFFER_;
    double K_BODY_D_BUFFER_;
    double L_RETRACT_;
    double L_TOUCH_;
    double L_BUFFER_SETTLE_;
    double landing_joint_handoff_duration_ = 0.16;
    double T_FLIGHT_TUCK_;
    double T_FLIGHT_APEX_;
    double T_FLIGHT_EXTEND_;
    double T_PROTECTIVE_DEPLOY_;
    double protective_deploy_duration_active_ = 0.24;
    double T_FLIGHT_TIMEOUT_;
    double PITCH_FLIGHT_GUARD_;
    double K_Z_BUFFER_;
    double D_Z_BUFFER_;
    double F_Z_BUFFER_MAX_;
    double TOTAL_MASS_;

    // ── 目标速度 THRUST 参数与试验统计 ──
    double jump_height_target_ = 0.20;
    double takeoff_velocity_override_ = 0.0;
    double target_takeoff_velocity_ = 1.98;
    double thrust_duration_ = 0.24;
    double thrust_peak_ratio_ = 2.30;
    double grounded_launch_boost_ratio_ = 0.75;
    double thrust_shape_early_ = 0.95;
    double thrust_shape_late_ = 0.75;
    double thrust_velocity_kp_ = 8.0;
    double thrust_torque_margin_ = 0.95;
    bool sim_relax_thrust_limits_ = false;
    bbot_jump::JointEffortLimits active_effort_limits_{75.0,60.0};
    std::array<double,4> ground_gravity_torque_{};
    double thrust_force_unlimited_request_ = 0.0;
    double thrust_knee_pd_left_ = 0.0;
    bool thrust_knee_position_yield_ = false;
    double thrust_timeout_ = 0.48;
    double thrust_start_z_ = 0.40;
    bool velocity_takeoff_reached_ = false;
    double actual_takeoff_velocity_ = 0.0;
    double max_z_during_jump_ = 0.0;
    double max_abs_pitch_during_jump_ = 0.0;
    double max_abs_hip_torque_during_jump_ = 0.0;
    double max_abs_knee_torque_during_jump_ = 0.0;
    double thrust_mechanical_work_ = 0.0;
    double last_thrust_force_ = 0.0;
    double last_summary_time_ = -1.0;
    bool jump_summary_written_ = false;
    std::string jump_failure_reason_;
    std::uint64_t jump_id_ = 0;
    std::ofstream jump_summary_file_;

    // ── 真实世界高度与落地点单参考 ──
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    bbot_jump::WorldPoseVelocity world_pose_velocity_;
    double odom_twist_z_diag_ = 0.0;
    double gazebo_world_z_ = 0.40;
    double gazebo_world_z_dot_ = 0.0;
    double gazebo_world_x_dot_ = 0.0;
    double gazebo_world_y_dot_ = 0.0;
    bool world_z_dot_filter_initialized_ = false;
    bool world_xy_dot_filter_initialized_ = false;
    bool odom_received_ = false;
    bool world_height_valid_for_jump_ = false;
    double max_world_z_during_jump_ = 0.0;
    double thrust_start_world_z_ = 0.40;
    double touchdown_x_ref_ = 0.0;
    bool touchdown_x_latched_ = false;

    // ── 离地构型锁存与动态诊断指标 ──
    double takeoff_q_hip_left_ = 0.0;
    double takeoff_q_knee_left_ = 0.0;
    double takeoff_q_hip_right_ = 0.0;
    double takeoff_q_knee_right_ = 0.0;
    double takeoff_pitch_rate_ = 0.0;
    double landing_pitch_err_ = 0.0;
    double touchdown_drift_x_ = 0.0;

    // ── 执行器硬件限幅输出记录 ──
    double actual_tau_hip_left_ = 0.0;
    double actual_tau_knee_left_ = 0.0;
    double actual_tau_hip_right_ = 0.0;
    double actual_tau_knee_right_ = 0.0;

    // ── Position 交接配置与状态 ──
    double body_mass_ = 9.5;
    double position_proportional_gain_ = 0.3;
    bool enable_position_handoff_ = true;
    double handoff_joint_error_limit_ = 0.12;
    double handoff_height_drop_limit_ = 0.04;
    double handoff_pitch_error_limit_ = 0.12;
    double handoff_z_dot_limit_ = 0.20;

    bbot_jump::RecoverySubphase recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
    double recovery_stable_timer_ = 0.0;
    double position_preload_timer_ = 0.0;
    double position_hold_timer_ = 0.0;
    double position_return_timer_ = 0.0;
    double position_switch_request_time_ = -1.0;
    bool position_handoff_suppressed_for_jump_ = false;
    double latched_pos_hip_left_ = 0.0;
    double latched_pos_knee_left_ = 0.0;
    double latched_pos_hip_right_ = 0.0;
    double latched_pos_knee_right_ = 0.0;
    bbot_jump::QuinticTrajectory traj_return_hip_l_;
    bbot_jump::QuinticTrajectory traj_return_knee_l_;
    bbot_jump::QuinticTrajectory traj_return_hip_r_;
    bbot_jump::QuinticTrajectory traj_return_knee_r_;
    double max_position_error_ = 0.0;
    std::string handoff_fallback_reason_;
    std::string controller_mode_str_ = "EFFORT";

    // ── 节点组件 ──
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jump_cmd_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_effort_pub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_client_;
    bool effort_mode_active_ = false;
    bool leg_mode_switch_pending_ = false;
    rclcpp::TimerBase::SharedPtr timer_;

    KeyboardReader keyboard_;
    bbot_kinematics::Kinematics kinematics_;
    rclcpp::Time last_time_;
    rclcpp::Time start_time_;

    std::string data_path_;
    std::ofstream jump_log_file_;

    bool effort_jump_preparation() const {
        return effort_jump_cycle_ && effort_mode_active_ &&
            (current_state_==bbot_jump::STATE_PRE_JUMP || current_state_==bbot_jump::STATE_SQUAT);
    }

    // ── 模式与跳跃触发 ──
    void trigger_jump()
    {
        const bool from_recovery = current_state_ == bbot_jump::STATE_RECOVERY &&
            recovery_ready_ && effort_mode_active_ && capture_world_valid_;
        if (current_state_ != bbot_jump::STATE_BALANCE && !from_recovery) {
            RCLCPP_WARN(this->get_logger(), "[跳跃请求忽略] 需要 BALANCE 或已就绪的 Effort RECOVERY (当前: %s)",
                        bbot_jump::state_to_string(current_state_));
            return;
        }

        if (!odom_received_ || this->now().seconds() - last_world_odom_time_ > 0.10) {
            RCLCPP_WARN(this->get_logger(), "[跳跃请求忽略] 世界里程计未就绪或过期，无法判定离地/触地");
            return;
        }

        // PRE_JUMP 会主动建立前向速度与前倾工作点；触发瞬间只拒绝已经明显失稳的状态。
        const bool takeoff_safe =
            std::abs(pitch_ - balance_offset_) < 0.25 &&
            std::abs(pitch_rate_) < 2.0 &&
            std::abs(from_recovery ? capture_com_velocity_ : x_dot_) < 0.30 &&
            (!from_recovery || (std::abs(ground_balance_angle())<0.06 &&
                                std::abs(ground_balance_rate())<0.35));
        if (!takeoff_safe) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[跳跃请求忽略] 请先空格停车并等待姿态稳定: pitch=%.3f gyro=%.3f v=%.3f",
                                 pitch_, pitch_rate_, from_recovery?capture_com_velocity_:x_dot_);
            return;
        }

        double now_sec = this->now().seconds();
        effort_jump_cycle_ = effort_mode_active_;
        recovery_ready_ = false;
        recovery_drive_ref_ = 0.0;
        recovery_yaw_ref_ = 0.0;
        jump_pitch_ref_ = balance_offset_ + jump_pitch_offset_;
        active_jump_pitch_ref_ = balance_offset_;
        active_jump_pitch_rate_ref_ = 0.0;
        pre_jump_hold_height_ = bbot_jump::clamp_value(current_z_, L_SQUAT_, L_MAX_);
        pre_jump_stable_timer_ = 0.0;
        pre_jump_balance_handoff_ = false;
        takeoff_forward_speed_ = 0.0;
        air_wheel_baseline_ = last_wheel_cmd_x_;
        landing_capture_vx_ = 0.0;
        landing_capture_raw_offset_ = 0.0;
        landing_capture_offset_ = 0.0;
        landing_target_x_ = 0.0;
        landing_capture_comp_ = 0.0;
        landing_capture_omega_ = 0.0;
        landing_forward_axis_x_ = 1.0;
        landing_forward_axis_y_ = 0.0;
        landing_forward_direction_sign_ = 1.0;
        landing_forward_axis_valid_ = false;
        landing_capture_planned_ = false;
        landing_target_shank_abs_ = 0.0;
        landing_target_knee_axis_clearance_ = 0.0;
        protective_deploy_duration_active_ = T_PROTECTIVE_DEPLOY_;
        flight_pitch_ref_start_ = pitch_;
        flight_air_pitch_ref_diag_ = pitch_;
        flight_air_pitch_rate_ref_diag_ = 0.0;
        protective_deploy_rate_limited_ = false;
        protective_deploy_replan_count_ = 0;

        RCLCPP_INFO(this->get_logger(),
                    ">>> 收到跳跃指令！PRE_JUMP 建立 vx=%.3f；SQUAT 末段建立 pitch_rate=%.3f；"
                    "THRUST 目标 vx=%.3f m/s，pitch 从 %.3f 推向 %.3f rad <<<",
                    jump_forward_speed_, jump_takeoff_pitch_rate_, jump_takeoff_forward_speed_,
                    balance_offset_, jump_pitch_ref_);
        current_state_ = bbot_jump::STATE_PRE_JUMP;
        state_start_time_ = now_sec;
        protective_landing_ = false;
        post_landing_effort_support_ = false;
        post_landing_position_handoff_requested_ = false;
        post_landing_position_settle_count_ = 0;
        post_landing_translation_feedback_ = false;
        post_landing_gyro_reduced_ = false;
        post_landing_pitch_ref_ = balance_offset_;
        position_switch_time_ = -1.0;
        thrust_start_z_ = current_z_;
        thrust_start_world_z_ = gazebo_world_z_;
        max_world_z_during_jump_ = gazebo_world_z_;
        world_height_valid_for_jump_ = odom_received_;
        touchdown_x_latched_ = false;
        touchdown_x_ref_ = x_;
        recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
        recovery_stable_timer_ = 0.0;
        position_preload_timer_ = 0.0;
        position_hold_timer_ = 0.0;
        position_return_timer_ = 0.0;
        position_switch_request_time_ = -1.0;
        position_handoff_suppressed_for_jump_ = false;
        max_position_error_ = 0.0;
        handoff_fallback_reason_.clear();
        controller_mode_str_ = effort_mode_active_ ? "EFFORT" : "POSITION";
        velocity_takeoff_reached_ = false;
        actual_takeoff_velocity_ = 0.0;
        takeoff_pitch_rate_ = 0.0;
        landing_pitch_err_ = 0.0;
        touchdown_drift_x_ = 0.0;
        max_z_during_jump_ = current_z_;
        max_abs_pitch_during_jump_ = 0.0;
        max_abs_hip_torque_during_jump_ = 0.0;
        max_abs_knee_torque_during_jump_ = 0.0;
        thrust_mechanical_work_ = 0.0;
        last_thrust_force_ = 0.0;
        last_thrust_force_request_ = 0.0;
        last_thrust_force_limit_ = 0.0;
        last_tau_body_per_hip_ = 0.0;
        last_thrust_reaction_ff_ = 0.0;
        last_wheels_airborne_ = false;
        landing_wheel_ground_blend_ = 0.0;
        touchdown_confirmation_.reset();
        flight_ff_force_start_ = 0.0;
        flight_trajectory_initialized_ = false;
        effort_slew_initialized_ = false;
        airborne_confidence_count_ = 0;
        thrust_release_.reset();
        touchdown_catch_stable_time_=0.0;
        thrust_motion_elapsed_ = 0.0;
        thrust_attitude_blocked_ = false;
        thrust_gate_has_opened_ = false;
        thrust_attitude_stable_count_ = 0;
        thrust_block_recovery_count_ = 0;
        thrust_block_elapsed_ = 0.0;
        fail_recovery_stable_timer_ = 0.0;
        recovery_follow_height_ik_ = false;
        flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST;
        attitude_arrest_start_time_ = 0.0;
        attitude_arrest_stable_count_ = 0;
        tuck_started_ = false;
        tuck_start_timestamp_ = 0.0;
        protective_deploy_initialized_ = false;
        protective_deploy_start_time_ = 0.0;
        air_wheel_cmd_raw_ = 0.0;
        last_summary_time_ = -1.0;
        jump_summary_written_ = false;
        jump_failure_reason_.clear();
        ++jump_id_;

        // PRE_JUMP 从当前位置开始积分移动参考，避免继承 BALANCE 的旧位置误差。
        target_speed_const_ = jump_forward_speed_;
        target_speed_smoothed_ = x_dot_;
        target_yaw_rate_ = 0.0;
        target_x_ = x_;
        was_moving_ = true;
        vel_integral_ = 0.0;
    }

    void handle_mode_command(const std::string & cmd)
    {
        if (cmd == "jump" || cmd == "j" || cmd == "J") {
            trigger_jump();
        } else if (cmd == "standup" || cmd == "r" || cmd == "R") {
            current_state_ = bbot_jump::STATE_STANDUP;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
        } else if (cmd == "emergency" || cmd == "x" || cmd == "X") {
            current_state_ = bbot_jump::STATE_EMERGENCY;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
        } else if (cmd == "balance") {
            current_state_ = bbot_jump::STATE_BALANCE;
        }
    }

    void process_keyboard()
    {
        std::string seq = keyboard_.read_sequence();
        if (seq.empty()) return;

        if (seq == "j" || seq == "J") {
            trigger_jump();
        } else if (seq == "w" || seq == "W") {
            target_speed_const_ = walk_speed_;
            target_yaw_rate_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 前进  speed=%.2f", target_speed_const_);
        } else if (seq == "s" || seq == "S") {
            target_speed_const_ = -walk_speed_;
            target_yaw_rate_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 后退  speed=%.2f", target_speed_const_);
        } else if (seq == "a" || seq == "A") {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = turn_speed_;
            RCLCPP_INFO(this->get_logger(), "[键盘] 左转  yaw=%.2f", target_yaw_rate_);
        } else if (seq == "d" || seq == "D") {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = -turn_speed_;
            RCLCPP_INFO(this->get_logger(), "[键盘] 右转  yaw=%.2f", target_yaw_rate_);
        } else if (seq == " ") {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            RCLCPP_INFO(this->get_logger(), "[键盘] 刹车停止");
        } else if (seq == "q" || seq == "Q") {
            if (current_state_ == bbot_jump::STATE_BALANCE) {
                target_height_ = bbot_jump::clamp_value(target_height_ + 0.01, L_MIN_, L_MAX_);
                RCLCPP_INFO(this->get_logger(), "[键盘] 升高  目标高度 → %.3f m", target_height_);
            }
        } else if (seq == "e" || seq == "E") {
            if (current_state_ == bbot_jump::STATE_BALANCE) {
                target_height_ = bbot_jump::clamp_value(target_height_ - 0.01, L_MIN_, L_MAX_);
                RCLCPP_INFO(this->get_logger(), "[键盘] 降低  目标高度 → %.3f m", target_height_);
            }
        } else if (seq == "r" || seq == "R") {
            current_state_ = bbot_jump::STATE_STANDUP;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 触发自适应起立恢复模式！");
        } else if (seq == "x" || seq == "X") {
            current_state_ = bbot_jump::STATE_EMERGENCY;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "[键盘] 紧急停机！");
        }
    }

    // ── 传感器回调 ──
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 机器人 CAD 约定：前倾对应负 roll，取负号使前倾为正
        pitch_ = -roll;
        pitch_rate_raw_ = -msg->angular_velocity.x;
        torso_imu_.update(rclcpp::Time(msg->header.stamp).seconds(),pitch_rate_raw_,
            msg->linear_acceleration.y,msg->linear_acceleration.z);

        if (!pitch_rate_filter_init_) {
            pitch_rate_filt_ = pitch_rate_raw_;
            pitch_rate_filter_init_ = true;
        } else {
            pitch_rate_filt_ = bbot_jump::low_pass_filter(pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
        }
        pitch_rate_ = pitch_rate_filt_;

        acc_z_raw_ = msg->linear_acceleration.z;
        if (!acc_z_filter_init_) {
            acc_z_filt_ = acc_z_raw_;
            acc_z_filter_init_ = true;
        } else {
            acc_z_prev_ = acc_z_filt_;
            acc_z_filt_ = bbot_jump::low_pass_filter(acc_z_raw_, acc_z_filt_, 0.20);
        }

        imu_received_ = true;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 使用消息时间戳估计采样周期；控制定时器200 Hz不等于传感器200 Hz。
        const double sample_time = rclcpp::Time(msg->header.stamp).seconds();
        const double stamp = sample_time > 0.0 ? sample_time : this->now().seconds();
        if (joint_sample_time_ >= 0.0 && stamp > joint_sample_time_) {
            joint_sample_period_ = bbot_jump::low_pass_filter(
                bbot_jump::clamp_value(stamp - joint_sample_time_, 0.005, 0.030),
                joint_sample_period_, 0.2);
        }
        joint_sample_time_ = stamp;
        bool has_left = false, has_right = false;
        std::array<bool, 4> has_leg_position{};
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "link_004_joint") {
                if (i < msg->position.size()) left_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size()) left_wheel_vel_ = msg->velocity[i];
                has_left = true;
            } else if (msg->name[i] == "link_007_joint") {
                if (i < msg->position.size()) right_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size()) right_wheel_vel_ = msg->velocity[i];
                has_right = true;
            } else if (msg->name[i] == "link_002_joint") {
                if (i < msg->position.size()) { hip_pos_left_ = msg->position[i]; has_leg_position[0]=true; }
                if (i < msg->velocity.size()) hip_vel_left_ = msg->velocity[i];
                if (i < msg->effort.size()) hip_effort_left_ = msg->effort[i];
            } else if (msg->name[i] == "link_003_joint") {
                if (i < msg->position.size()) { knee_pos_left_ = msg->position[i]; has_leg_position[1]=true; }
                if (i < msg->velocity.size()) knee_vel_left_ = msg->velocity[i];
                if (i < msg->effort.size()) knee_effort_left_ = msg->effort[i];
            } else if (msg->name[i] == "link_005_joint") {
                if (i < msg->position.size()) { hip_pos_right_ = msg->position[i]; has_leg_position[2]=true; }
                if (i < msg->velocity.size()) hip_vel_right_ = msg->velocity[i];
                if (i < msg->effort.size()) hip_effort_right_ = msg->effort[i];
            } else if (msg->name[i] == "link_006_joint") {
                if (i < msg->position.size()) { knee_pos_right_ = msg->position[i]; has_leg_position[3]=true; }
                if (i < msg->velocity.size()) knee_vel_right_ = msg->velocity[i];
                if (i < msg->effort.size()) knee_effort_right_ = msg->effort[i];
            }
        }

        if (std::all_of(has_leg_position.begin(),has_leg_position.end(),[](bool value){return value;})) {
            // 零时间戳不假装同步；仅真实消息时间戳可进入几何历史。
            if (sample_time>0.0) takeoff_joint_history_.push(sample_time,
                {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_});
        }

        if (has_left && has_right) {
            if (!wheel_origin_set_) {
                left_wheel_pos_origin_ = left_wheel_pos_;
                right_wheel_pos_origin_ = right_wheel_pos_;
                wheel_origin_set_ = true;
                prev_z_time_ = this->now();
            }
            x_dot_raw_ = -wheel_radius_ * 0.5 * (left_wheel_vel_ + right_wheel_vel_);
            if (!x_dot_filter_init_) {
                x_dot_filt_ = x_dot_raw_;
                x_dot_filter_init_ = true;
            } else {
                x_dot_filt_ = bbot_jump::low_pass_filter(x_dot_raw_, x_dot_filt_, x_dot_alpha_);
            }
            x_dot_ = x_dot_filt_;

            double left_delta = left_wheel_pos_ - left_wheel_pos_origin_;
            double right_delta = right_wheel_pos_ - right_wheel_pos_origin_;
            x_ = -wheel_radius_ * 0.5 * (left_delta + right_delta);
        }

        // 计算当前腿长与竖直速度：综合左右双腿平均高度，消除单腿盲区与不对称塌软
        double z_left = kinematics_.calculate_com_height(pitch_, hip_pos_left_, knee_pos_left_);
        double z_right = kinematics_.calculate_com_height(pitch_, hip_pos_right_, knee_pos_right_);
        double z_calc = 0.5 * (z_left + z_right);
        rclcpp::Time now_t = sample_time > 0.0 ?
            rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) : this->now();
        double dt_z = (now_t - prev_z_time_).seconds();
        if (dt_z > 0.001) {
            current_z_dot_raw_ = (z_calc - prev_z_) / dt_z;
            current_z_dot_raw_ = bbot_jump::clamp_value(current_z_dot_raw_, -3.0, 3.0);
            if (!z_dot_filter_init_) {
                current_z_dot_ = current_z_dot_raw_;
                z_dot_filter_init_ = true;
            } else {
                current_z_dot_ = bbot_jump::low_pass_filter(
                    current_z_dot_raw_, current_z_dot_, 0.22);
            }
            prev_z_ = z_calc;
            prev_z_time_ = now_t;
        }
        current_z_ = z_calc;
    }

    // ── 主控制循环 (200Hz) ──
    void control_loop()
    {
        if (!imu_received_ || !wheel_origin_set_) return;

        process_keyboard();

        rclcpp::Time now = this->now();
        double now_sec = now.seconds();
        double previous_control_sec = last_time_.seconds();
        double dt = 0.0;
        const bool control_due = bbot_jump::advance_control_time(now_sec, previous_control_sec, dt);
        if (!control_due) {
            if (now < last_time_) last_time_ = now;
            return;
        }
        last_time_ = now;

        // v6.7：机身姿态与整机重心是不同状态。腿在落地压缩时会改变
        // 重心相对轮轴的位置和速度；所有落地轮控阶段共用这一状态。
        centroidal_balance_ = bbot_jump::centroidal_balance_state(
            {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_},
            {hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_},
            pitch_,pitch_rate_,body_mass_);
        if (odom_received_ && takeoff_odom_pose_valid_)
            centroidal_height_.update(takeoff_odom_stamp_,now_sec,gazebo_world_z_,
                odom_world_z_in_body_,takeoff_joint_history_,body_mass_);
        centroidal_velocity_valid_ = takeoff_odom_pose_valid_ && centroidal_height_.valid(now_sec);

        if (odom_received_ && takeoff_odom_pose_valid_)
            centroidal_world_.update(takeoff_odom_stamp_,now_sec,odom_base_position_,
                odom_body_rotation_,takeoff_joint_history_,body_mass_);
        Eigen::Vector3d heading=odom_body_rotation_.col(1);
        heading.z()=0.0;
        capture_world_valid_=takeoff_odom_pose_valid_ && centroidal_world_.valid(now_sec) &&
            heading.norm()>0.5 && centroidal_balance_.valid &&
            torso_imu_.fresh(now_sec) && now_sec>=joint_sample_time_ &&
            now_sec-joint_sample_time_<=0.080;
        if (capture_world_valid_)
            capture_com_velocity_=centroidal_world_.forward_velocity(heading.normalized());
        capture_world_active_=false;
        thrust_knee_position_yield_=false;

        // 执行当前状态机分支
        switch (current_state_)
        {
            case bbot_jump::STATE_BALANCE:
                run_state_balance(dt);
                break;
            case bbot_jump::STATE_PRE_JUMP:
                run_state_pre_jump(now_sec, dt);
                break;
            case bbot_jump::STATE_SQUAT:
                run_state_squat(now_sec, dt);
                break;
            case bbot_jump::STATE_THRUST:
                run_state_thrust(now_sec, dt);
                break;
            case bbot_jump::STATE_FLIGHT:
                run_state_flight(now_sec);
                break;
            case bbot_jump::STATE_TOUCHDOWN_BUFFER:
                run_state_touchdown_buffer(now_sec, dt);
                break;
            case bbot_jump::STATE_RECOVERY:
                run_state_recovery(now_sec, dt);
                break;
            case bbot_jump::STATE_STANDUP:
                run_state_standup();
                break;
            case bbot_jump::STATE_EMERGENCY:
            publish_wheel_cmd(0.0, 0.0);
                if (effort_mode_active_) {
                    publish_effort_leg_control(hip_pos_left_, knee_pos_left_, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                } else {
                    publish_position_leg_control(hip_pos_left_, knee_pos_left_);
                }
                break;
        }

        num_++;
    }

    // ── 阶段 0：变高度 LQR 自平衡 ──
    void run_state_balance(double dt)
    {
        // 1. 平滑过渡高度
        update_leg_height_by_dt(dt);

        // 2. 动态插值 LQR 增益
        interpolate_lqr_gain();

        // 失衡保护
        // if (std::abs(pitch_err) > 1.80) {
        //     current_state_ = bbot_jump::STATE_STANDUP;
        //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //                          "[平衡控制器] 倾角过大失衡 (pitch=%.3f)，进入起立恢复模式...", pitch_);
        //     return;
        // }

        // 3. 目标速度平滑过渡
        double target_speed_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < target_speed_const_) {
            target_speed_smoothed_ += target_speed_step;
            if (target_speed_smoothed_ > target_speed_const_)
                target_speed_smoothed_ = target_speed_const_;
        } else if (target_speed_smoothed_ > target_speed_const_) {
            target_speed_smoothed_ -= target_speed_step;
            if (target_speed_smoothed_ < target_speed_const_)
                target_speed_smoothed_ = target_speed_const_;
        }
        double target_speed = target_speed_smoothed_;

        // 4. 位置参考积分
        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005) {
            if (was_moving_) {
                target_x_ = x_;
                was_moving_ = false;
            }
        } else {
            target_x_ += target_speed * dt;
            was_moving_ = true;
        }

        // 状态误差计算 (实际值 - 目标值)
        double pos_error = x_ - target_x_;
        double vel_error = x_dot_ - target_speed;
        double gyro_val = pitch_rate_;
        double dynamic_target_pitch = post_landing_gyro_reduced_ ?
            post_landing_pitch_ref_ : balance_offset_;
        double theta_error = 0.0;
        double u_pitch = 0.0;
        double cmd_x = 0.0;
        const double gyro_gain_scale =
            post_landing_gyro_reduced_ ? 0.50 : 1.0;
        const double translation_gain_scale = 1.0;

        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005) {
            theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(translation_gain_scale * current_gain_.k_x * pos_error +
                        translation_gain_scale * current_gain_.k_x_dot * vel_error +
                        current_gain_.k_theta * theta_error +
                        gyro_gain_scale * current_gain_.k_theta_dot * gyro_val);
            cmd_x = -u_pitch * cmd_scale_;
            vel_integral_ = 0.0;
        } else {
            double vel_error_v = target_speed - x_dot_;
            vel_integral_ += vel_error_v * dt;
            vel_integral_ = bbot_jump::clamp_value(vel_integral_, -0.5, 0.5);

            double kp_v = 0.25;
            double ki_v = 0.05;
            dynamic_target_pitch = (post_landing_gyro_reduced_ ? post_landing_pitch_ref_ : balance_offset_) +
                                   (kp_v * vel_error_v + ki_v * vel_integral_);
            dynamic_target_pitch = bbot_jump::clamp_value(dynamic_target_pitch, -0.2, 0.2);

            theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_theta * theta_error +
                        gyro_gain_scale * current_gain_.k_theta_dot * gyro_val);
            cmd_x = -u_pitch * cmd_scale_ - target_speed;
        }

        cmd_x = bbot_jump::clamp_value(cmd_x, -max_cmd_x_, max_cmd_x_);
        // 落地后平移速度与变化率限制
        if (post_landing_translation_feedback_) {
            cmd_x = bbot_jump::clamp_value(cmd_x, -1.0, 1.0);
            const double max_cmd_step = 10.0 * dt;
            cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                cmd_x - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        }

        if (pre_jump_balance_handoff_) {
            const double requested_cmd = cmd_x;
            cmd_x = bbot_jump::rolling_abort_wheel_command(cmd_x, last_wheel_cmd_x_, dt);
            const bool stopped = std::abs(target_speed) < 0.005 &&
                std::abs(x_dot_) < 0.04 && std::abs(pitch_rate_) < 0.15;
            if (stopped && std::abs(requested_cmd - cmd_x) < 1e-6)
                pre_jump_balance_handoff_ = false;
        }

        // 落地软接管计时
        if (post_landing_balance_soft_start_) {
            const double balance_elapsed = this->now().seconds() - balance_entry_time_;
            if (balance_elapsed >= 1.0) {
                post_landing_balance_soft_start_ = false;
            }
        }
        publish_wheel_cmd(cmd_x, target_yaw_rate_);

        // 8Hz 遥测打印
        if (num_ % 25 == 0) {
            const double term_x = translation_gain_scale * current_gain_.k_x * pos_error;
            const double term_xdot = translation_gain_scale * current_gain_.k_x_dot * vel_error;
            const double term_theta = current_gain_.k_theta * theta_error;
            const double term_theta_dot = gyro_gain_scale * current_gain_.k_theta_dot * gyro_val;
            std::cout << "[BALANCE]"
                      << " x=" << x_
                      << " target_x=" << target_x_
                      << " x_err=" << pos_error
                      << " v=" << x_dot_
                      << " v_err=" << vel_error
                      << " pitch=" << pitch_
                      << " pitch_err=" << theta_error
                      << " pitch_rate=" << pitch_rate_ << '\n'
                      << "  terms: x=" << term_x
                      << " v=" << term_xdot
                      << " pitch=" << term_theta
                      << " gyro=" << term_theta_dot
                      << " cmd_x=" << cmd_x << std::endl;
        }

        // 腿部逆运动学与重力矩计算
        bbot_kinematics::IKSolution ik_bal =
            kinematics_.inverse_kinematics(current_height_, 0.0);
        bbot_kinematics::JointTorques g_torques =
            kinematics_.compute_gravity_torques(
                0.0, ik_bal.theta_hip, ik_bal.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;

        if (post_landing_effort_support_) {
            // 跳后稳态力矩支撑
            request_effort_controller();
            if (effort_mode_active_) {
                support_force_total = publish_effort_balance_control(current_height_);
            }
        } else {
            // 起跳前默认位置控制模式
            const double q_hip_cmd = ik_bal.theta_hip + 0.008;
            const double q_knee_cmd = ik_bal.theta_knee - 0.040;
            publish_position_leg_control(q_hip_cmd, q_knee_cmd);
        }

        log_data(cmd_x,
                 g_torques.hip_torque * 0.5,
                 g_torques.knee_torque * 0.5,
                 support_force_total);

        const bool balance_quiet = std::abs(pitch_ - balance_offset_) < 0.22 &&
            std::abs(pitch_rate_) < 1.5 && std::abs(x_dot_) < 0.25 &&
            std::abs(x_ - target_x_) < 0.25;
        balance_settle_count_ = balance_quiet ? std::min(balance_settle_count_ + 1, 1000) : 0;
    }

    // ── jump-thrust-v5：滚动起跳准备 (PRE_JUMP) ──
    void run_state_pre_jump(double now_sec, double dt)
    {
        const double elapsed = now_sec - state_start_time_;
        current_height_ = pre_jump_hold_height_;

        // 腿保持触发瞬间高度，PRE_JUMP 只建立水平速度与机身工作姿态。
        const auto ik_hold = kinematics_.inverse_kinematics(pre_jump_hold_height_, 0.0);
        const auto g_torques = kinematics_.compute_gravity_torques(
            0.0, ik_hold.theta_hip, ik_hold.theta_knee);
        if (effort_mode_active_) {
            // Repeat jump stays on the same ground effort law as recovery.
            publish_effort_height_control(
                pre_jump_hold_height_, 0.0,
                K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                25.0, 3.5, 45.0, 6.0,
                true);
        } else {
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);
        }

        // PRE_JUMP 直接复用 BALANCE 已有的“速度外环 -> 俯仰参考 -> 姿态内环”结构。
        // 不再把 k_x / k_x_dot 直接作用到持续移动的 target_x_ 上：上一版中
        // 位置/速度状态反馈会压过 -target_speed 前馈，使 cmd 长时间保持正值，
        // 实测 x_dot 因而一直停留在 0 附近甚至反向，永远到不了 +0.20 m/s。
        const double target_speed_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < jump_forward_speed_) {
            target_speed_smoothed_ = std::min(
                target_speed_smoothed_ + target_speed_step, jump_forward_speed_);
        } else if (target_speed_smoothed_ > jump_forward_speed_) {
            target_speed_smoothed_ = std::max(
                target_speed_smoothed_ - target_speed_step, jump_forward_speed_);
        }
        const double target_speed = target_speed_smoothed_;

        // target_x_ 仅保留为诊断参考，不参与 PRE_JUMP 轮控反馈。
        target_x_ += target_speed * dt;
        interpolate_lqr_gain();
        const double pos_error = x_ - target_x_;
        const double vel_error_v = target_speed - x_dot_;
        vel_integral_ += vel_error_v * dt;
        vel_integral_ = bbot_jump::clamp_value(vel_integral_, -0.5, 0.5);

        // 与 run_state_balance() 的移动分支使用完全相同的速度外环参数。
        constexpr double kp_v = 0.25;
        constexpr double ki_v = 0.05;
        active_jump_pitch_ref_ = balance_offset_ +
            kp_v * vel_error_v + ki_v * vel_integral_;
        active_jump_pitch_ref_ = bbot_jump::clamp_value(
            active_jump_pitch_ref_, -0.2, 0.2);
        if (effort_jump_preparation()) active_jump_pitch_ref_=balance_offset_;
        // PRE_JUMP 仍要求近似零角速度，避免过早把机器人推入持续前倒。
        active_jump_pitch_rate_ref_ = 0.0;

        const double pitch_error = pitch_ - active_jump_pitch_ref_;
        const double pitch_rate_error = pitch_rate_ - active_jump_pitch_rate_ref_;
        const double u_pitch = -(
            current_gain_.k_theta * pitch_error +
            current_gain_.k_theta_dot * pitch_rate_error);

        double cmd_target = -u_pitch * cmd_scale_ - target_speed;
        cmd_target = bbot_jump::clamp_value(cmd_target, -1.20, 1.20);
        if (effort_jump_preparation()) cmd_target=landing_capture_target(cmd_target);
        const double max_cmd_step = (effort_jump_preparation()?8.0:5.0) * std::max(dt, 0.001);
        const double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

        const bool ready_now = (!effort_jump_preparation() || capture_world_valid_) &&
            bbot_jump::rolling_prepare_ready(
            effort_jump_preparation() ? capture_com_velocity_ : x_dot_,
            jump_forward_speed_, target_speed_smoothed_,
            pitch_ - active_jump_pitch_ref_, pitch_rate_,
            pre_jump_speed_tolerance_, pre_jump_pitch_tolerance_, pre_jump_rate_limit_);
        pre_jump_stable_timer_ = ready_now ?
            (pre_jump_stable_timer_ + dt) : 0.0;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[PRE_JUMP] t=%.3f vx=%.3f/%.3f pitch=%.3f/%.3f gyro=%.3f xerr=%.3f stable=%.3f/%.3f cmd=%.3f",
            elapsed, x_dot_, jump_forward_speed_, pitch_, active_jump_pitch_ref_, pitch_rate_,
            pos_error, pre_jump_stable_timer_, pre_jump_stable_duration_, cmd_x);

        log_data(cmd_x, g_torques.hip_torque * 0.5,
                 g_torques.knee_torque * 0.5, TOTAL_MASS_ * 9.81);

        if (pre_jump_stable_timer_ >= pre_jump_stable_duration_) {
            // 进入 SQUAT 时重新锚定移动位置参考，只保留速度/姿态工作点，避免位置误差阶跃。
            target_x_ = x_;
            state_start_time_ = now_sec;
            current_state_ = bbot_jump::STATE_SQUAT;
            // 锁存速度外环已经建立的俯仰参考，不能在SQUAT首帧退回静态零点。
            squat_pitch_start_ref_ = active_jump_pitch_ref_;
            const double squat_start_height = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, L_MAX_);
            current_height_ = squat_start_height;
            quintic_traj_.init(now_sec, T_SQUAT_,
                               squat_start_height, 0.0, 0.0,
                               L_SQUAT_, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(),
                        ">>> PRE_JUMP 达标：vx=%.3f, pitch=%.3f, gyro=%.3f。进入 moving-SQUAT <<<",
                        x_dot_, pitch_, pitch_rate_);
            return;
        }

        if (elapsed >= pre_jump_timeout_) {
            // PRE_JUMP 仍处于 Position/常规平衡域，不进入 RECOVERY，也不切换 Effort。
            jump_failure_reason_ = "PRE_JUMP未能建立滚动起跳工作点";
            RCLCPP_WARN(this->get_logger(),
                        "[PRE_JUMP中止] %.2fs 内未稳定到目标 (vx=%.3f/%.3f pitch=%.3f/%.3f gyro=%.3f)，返回地面支撑",
                        elapsed, x_dot_, jump_forward_speed_, pitch_, active_jump_pitch_ref_, pitch_rate_);
            if (effort_jump_cycle_) {
                abort_jump_to_recovery(now_sec,"Effort PRE_JUMP准备超时");
                return;
            }
            current_state_ = bbot_jump::STATE_BALANCE;
            target_speed_const_ = 0.0;
            // 保留当前平滑速度和PI状态，让BALANCE移动分支逐步减速。
            // 不能同一帧清零速度、积分并切到静态位置LQR。
            pre_jump_balance_handoff_ = true;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = true;
            current_height_ = pre_jump_hold_height_;
            target_height_ = pre_jump_hold_height_;
            write_jump_summary(false, jump_failure_reason_);
            return;
        }
    }

    // ── 阶段 1：下蹲蓄力 (SQUAT) ──
    void run_state_squat(double now_sec, double dt)
    {
        double elapsed = now_sec - state_start_time_;
        double des_z, des_v, des_acc;
        quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);
        (void)des_acc;

        current_height_ = des_z;

        // 下蹲重力补偿力矩 + 高刚度轨迹跟踪 (Kp=350, Kd=14)
        bbot_kinematics::IKSolution ik_sq = kinematics_.inverse_kinematics(des_z, 0.0);
        bbot_kinematics::JointTorques g_torques = kinematics_.compute_gravity_torques(0.0, ik_sq.theta_hip, ik_sq.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;
        if (effort_mode_active_) {
            support_force_total = publish_effort_height_control(
                des_z, des_v,
                K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                25.0, 3.5, 45.0, 6.0,
                true);
        } else {
            publish_position_leg_control(ik_sq.theta_hip, ik_sq.theta_knee);
        }

        // moving-SQUAT：PRE_JUMP 已经建立前向速度，此阶段只让起跳俯仰参考
        // 从准备末帧的俯仰参考平滑过渡到 jump_pitch_ref_。target_x_ 继续积分只用于诊断；
        // 不再使用 k_x / k_x_dot 反馈，否则持续移动参考会再次与速度前馈相互对抗。
        target_x_ += jump_forward_speed_ * dt;
        interpolate_lqr_gain();
        const double pos_error = x_ - target_x_;
        const double squat_progress = bbot_jump::clamp_value(elapsed / T_SQUAT_, 0.0, 1.0);
        active_jump_pitch_ref_ = bbot_jump::rolling_reference_blend(
            squat_pitch_start_ref_, jump_pitch_ref_, squat_progress);

        // v5.3：前 65% 下蹲只建立几何前倾；最后 35% 才平滑建立正的俯仰角速度。
        // 这样 THRUST 接管时机身已经“向前转”，而不是又被 D 项刹到 pitch_rate≈0。
        constexpr double rate_build_start = 0.65;
        const double rate_phase = bbot_jump::clamp_value(
            (squat_progress - rate_build_start) / (1.0 - rate_build_start), 0.0, 1.0);
        const double rate_blend = rate_phase * rate_phase * (3.0 - 2.0 * rate_phase);
        active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_ * rate_blend;

        const double pitch_err = pitch_ - active_jump_pitch_ref_;
        const double pitch_rate_err = pitch_rate_ - active_jump_pitch_rate_ref_;
        const double u_pitch = -(
            current_gain_.k_theta * pitch_err +
            current_gain_.k_theta_dot * pitch_rate_err);
        // PRE_JUMP末帧使用cmd_scale_；尺度也连续过渡到原下蹲值。
        const double squat_cmd_scale = bbot_jump::rolling_reference_blend(
            cmd_scale_, 0.037, squat_progress);
        double cmd_target = -u_pitch * squat_cmd_scale - jump_forward_speed_;
        cmd_target = bbot_jump::clamp_value(cmd_target, -1.30, 1.30);
        if (effort_jump_preparation()) cmd_target=landing_capture_target(cmd_target);
        const double max_cmd_step = (effort_jump_preparation()?8.0:6.0) * std::max(dt, 0.001);
        const double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[SQUAT_MOVING] effort=%d z=%.3f pitch=%.3f/%.3f perr=%.3f gyro=%.3f/%.3f "
            "xerr=%.3f vx=%.3f/%.3f cmd=%.3f",
            effort_mode_active_ ? 1 : 0, current_z_, pitch_, active_jump_pitch_ref_, pitch_err,
            pitch_rate_, active_jump_pitch_rate_ref_, pos_error, x_dot_, jump_forward_speed_, cmd_x);

        log_data(cmd_x, g_torques.hip_torque * 0.5, g_torques.knee_torque * 0.5,
                 support_force_total);

        // 下蹲蓄力完成条件：到达目标附近、速度处于可推地范围即可。
        // |z_dot|<0.04 对仿真接触振动过于苛刻，会无限卡在 SQUAT。
        const bool traj_done = (elapsed >= T_SQUAT_);
        const bool settled = (std::abs(current_z_dot_) < 0.25 && std::abs(current_z_ - L_SQUAT_) < 0.060);
        const bool motion_ready =
            (!effort_jump_preparation() || capture_world_valid_) &&
            std::abs((effort_jump_preparation()?capture_com_velocity_:x_dot_) - jump_forward_speed_) <= 0.12 &&
            std::abs(pitch_ - jump_pitch_ref_) <= 0.080 &&
            std::abs(pitch_rate_ - jump_takeoff_pitch_rate_) <=
                jump_takeoff_pitch_rate_tolerance_;
        const bool forced_ready = (elapsed >= T_SQUAT_ + 0.15 &&
                                   std::abs(current_z_ - L_SQUAT_) < 0.10 &&
                                   motion_ready);
        const bool squat_timeout = (elapsed >= T_SQUAT_ + 0.35);

        if (((traj_done && settled) && motion_ready) || forced_ready) {
            RCLCPP_INFO(this->get_logger(), ">>> 蓄力完成%s (t=%.3fs, z=%.3f, v=%.2f)！启动阶段 2：全力爆发弹射推地 (THRUST)... <<<",
                        forced_ready && !settled ? "（使用可控速度兜底）" : "",
                        elapsed, current_z_, current_z_dot_);
            current_state_ = bbot_jump::STATE_THRUST;
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            thrust_trajectory_initialized_ = false;
            thrust_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
            thrust_force_command_per_leg_ = thrust_force_per_leg_;
            velocity_reached_count_ = 0;
            thrust_release_.reset();
            thrust_motion_elapsed_ = 0.0;
            thrust_extension_scale_ = 1.0;
            thrust_attitude_blocked_ = true;
            thrust_gate_has_opened_ = false;
            thrust_attitude_stable_count_ = 0;
            thrust_block_recovery_count_ = 0;
            thrust_block_elapsed_ = 0.0;
            active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_;
            // 保留 moving-SQUAT 的最后轮速命令，THRUST 不得人为制造水平速度阶跃。

            // 切换前预写重力支撑力矩
            if (!effort_mode_active_) {
                publish_effort_leg_control(ik_sq.theta_hip, ik_sq.theta_knee, 0.0, 0.0,
                                           -g_torques.hip_torque * 0.5, -g_torques.knee_torque * 0.5,
                                           80.0, 8.0, 80.0, 8.0);
            } // Repeat jump keeps the last supporting effort until THRUST starts.
            request_effort_controller();

            prev_q_hip_des_ = ik_sq.theta_hip;
            prev_q_knee_des_ = ik_sq.theta_knee;
        } else if (squat_timeout) {
            // 高度尚未接近下蹲目标，继续推地会造成冲击；此时安全返回。
            abort_jump_to_recovery(now_sec, "下蹲后腿长或垂直速度未稳定");
        }
    }

    // ── 阶段 2：爆发推地 (THRUST) ──
    void run_state_thrust(double now_sec, double dt)
    {
        if (!effort_mode_active_) {
            // 控制器异步切换期间不能把刚建立的前倾角速度耗掉。
            // 腿仍保持下蹲位置，但轮控继续追踪“前倾角 + 正俯仰角速度”，并开始向离地前向速度加速。
            request_effort_controller();
            bbot_kinematics::IKSolution ik_hold = kinematics_.inverse_kinematics(L_SQUAT_, 0.0);
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);

            interpolate_lqr_gain();
            active_jump_pitch_ref_ = jump_pitch_ref_;
            active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_;
            const double pitch_err_wait = pitch_ - active_jump_pitch_ref_;
            const double pitch_rate_err_wait = pitch_rate_ - active_jump_pitch_rate_ref_;
            const double u_pitch_wait = -(
                current_gain_.k_theta * pitch_err_wait +
                current_gain_.k_theta_dot * pitch_rate_err_wait);
            double cmd_target_wait =
                -u_pitch_wait * 0.037 - jump_takeoff_forward_speed_;
            cmd_target_wait = bbot_jump::clamp_value(cmd_target_wait, -1.50, 1.50);
            const double max_wait_step = 7.0 * std::max(dt, 0.001);
            const double cmd_wait = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                cmd_target_wait - last_wheel_cmd_x_, -max_wait_step, max_wait_step);
            publish_wheel_cmd(cmd_wait, 0.0);
            return;
        }

        if (!thrust_trajectory_initialized_) {
            double ground_geom_left=0.0, ground_geom_right=0.0;
            if (!aligned_takeoff_geometry(now_sec,ground_geom_left,ground_geom_right)) {
                // 保持切换前已经写入的支撑力矩，等待最近odom时刻的关节样本。
                // 不能用不同时间的几何标定地面，否则整个跳跃带着固定偏差。
                if (now_sec-state_start_time_>0.20)
                    abort_jump_to_recovery(now_sec,"起跳缺少可对齐的里程计/关节时间戳");
                return;
            }
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            thrust_start_z_ = current_z_;
            thrust_start_world_z_ = gazebo_world_z_;
            // 接地时标定世界高度与 FK 的常量偏差；FK 只用于几何间隙。
            ground_height_offset_ = gazebo_world_z_ - 0.5*(ground_geom_left+ground_geom_right);
            takeoff_confirmation_.reset();
            takeoff_rise_observed_ = false;
            airborne_confidence_count_ = 0;
            last_world_descent_time_ = -1.0;
            contact_window_ = false;
            max_world_z_during_jump_ = gazebo_world_z_;
            quintic_traj_.init(0.0, thrust_duration_, current_z_, 0.0, 0.0,
                               H_TAKEOFF_, 0.8, 0.0);
            thrust_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
            thrust_force_command_per_leg_ = thrust_force_per_leg_;
            velocity_reached_count_ = 0;
            thrust_motion_elapsed_ = 0.0;
            thrust_extension_scale_ = 1.0;
            thrust_attitude_blocked_ = true;
            // 这里仍是 THRUST 初始姿态 gate，而不是运行中的硬故障 block。
            thrust_gate_has_opened_ = false;
            thrust_attitude_stable_count_ = 0;
            thrust_block_recovery_count_ = 0;
            thrust_block_elapsed_ = 0.0;
            thrust_trajectory_initialized_ = true;
        }
        double elapsed = now_sec - state_start_time_;

        // v6.4：前倾角速度只用于有限的起跳准备，不可在角度目标停止后
        // 仍持续要求 +0.45 rad/s。角度参考为该平滑减速曲线的积分。
        const auto thrust_pitch_ref = bbot_jump::thrust_pitch_reference(
            jump_pitch_ref_, jump_takeoff_pitch_rate_,
            thrust_pitch_rate_lead_time_, thrust_motion_elapsed_);
        active_jump_pitch_ref_ = thrust_pitch_ref.angle;
        active_jump_pitch_rate_ref_ =
            (thrust_gate_has_opened_ && thrust_attitude_blocked_) ? 0.0 : thrust_pitch_ref.rate;
        const double pitch_err = pitch_ - active_jump_pitch_ref_;
        const double pitch_rate_err = pitch_rate_ - active_jump_pitch_rate_ref_;
        const double balance_pitch_err = pitch_ - balance_offset_;

        // 门控围绕“目标正俯仰角速度”判断，而不是要求角速度接近 0。
        constexpr double thrust_gate_pitch_limit = 0.08;
        constexpr double thrust_gate_duration = 0.020;
        const bool attitude_stable =
            (std::abs(pitch_err) <= thrust_gate_pitch_limit &&
             std::abs(pitch_rate_err) <= jump_takeoff_pitch_rate_tolerance_);
        if (attitude_stable) {
            thrust_attitude_stable_count_++;
        } else {
            thrust_attitude_stable_count_ = 0;
        }
        const bool stable_duration_met =
            (thrust_attitude_stable_count_ * dt >= thrust_gate_duration);

        // v6.0：把“短暂负角速度”和“真正后仰失稳”拆开。
        // 上一轮 pitch=0.152、jump_err=+0.008、balance_err=+0.114 都仍是明显前倾，
        // 仅 gyro=-0.804 就进入 block，随后又要求 gyro 回到 +0.45 附近才能解锁，
        // 结果推地轨迹永久冻结直到 0.20s 超时。
        //
        // soft_backward_rate：角度仍安全，只是角速度短暂向后。此时继续推进轨迹，
        // 由下方姿态衰减减少额外推力，同时髋姿态 D 项继续把 rate 拉回。
        const bool soft_backward_rate =
            (pitch_rate_ < -0.65 && pitch_rate_ >= -1.35 &&
             pitch_err > -0.06 && balance_pitch_err > 0.00);
        // hard_backward_tendency：角度已经掉到参考后方，或负角速度真的很大。
        // 只有这种情况才允许冻结推地轨迹。
        const bool hard_backward_tendency =
            (pitch_err < -0.10 || balance_pitch_err < -0.06 || pitch_rate_ < -1.35);
        const bool forward_tendency_excess =
            (pitch_err > 0.12 || balance_pitch_err > 0.26 || pitch_rate_ > 1.20);
        // 运行中 hard block 的解锁条件不再要求重新追到 +0.45 rad/s；
        // 只要回到可恢复的近零角速度区间即可继续，避免“一旦 block 就永远解不开”。
        const bool hard_block_recovery_ok =
            (std::abs(pitch_err) <= 0.10 &&
             pitch_rate_ > -0.35 && pitch_rate_ < 0.80 &&
             balance_pitch_err > -0.03 && balance_pitch_err < 0.24);

        if (thrust_attitude_blocked_) {
            thrust_block_elapsed_ += dt;
            if (!thrust_gate_has_opened_) {
                // THRUST 首次启动仍使用原来的动态起跳工作点 gate。
                if (thrust_block_elapsed_ > 0.20) {
                    abort_jump_to_recovery(now_sec, "THRUST初始姿态门控超过0.20s");
                    return;
                }
                if (stable_duration_met) {
                    thrust_attitude_blocked_ = false;
                    thrust_gate_has_opened_ = true;
                    thrust_block_elapsed_ = 0.0;
                    thrust_block_recovery_count_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                                "[推地门控] 动态起跳姿态达标 (pitch=%.3f ref=%.3f err=%.3f "
                                "gyro=%.3f/%.3f)，推进推地轨迹 (motion_t=%.3f s)",
                                pitch_, active_jump_pitch_ref_, pitch_err,
                                pitch_rate_, active_jump_pitch_rate_ref_, thrust_motion_elapsed_);
                }
            } else {
                // 运行中的硬阻塞采用更宽松、物理上可恢复的解锁条件。
                if (hard_block_recovery_ok) {
                    thrust_block_recovery_count_++;
                } else {
                    thrust_block_recovery_count_ = 0;
                }
                if (thrust_block_recovery_count_ * dt >= 0.015) {
                    thrust_attitude_blocked_ = false;
                    thrust_block_elapsed_ = 0.0;
                    thrust_block_recovery_count_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                                "[推地门控] 硬姿态阻塞已恢复 (pitch=%.3f err=%.3f gyro=%.3f)，继续推地",
                                pitch_, pitch_err, pitch_rate_);
                } else if (thrust_block_elapsed_ > thrust_hard_block_timeout_) {
                    abort_jump_to_recovery(now_sec, "推地硬姿态阻塞超时");
                    return;
                }
            }
        } else {
            if (hard_backward_tendency || forward_tendency_excess) {
                thrust_attitude_blocked_ = true;
                thrust_block_elapsed_ = 0.0;
                thrust_attitude_stable_count_ = 0;
                thrust_block_recovery_count_ = 0;
                RCLCPP_WARN(this->get_logger(),
                            "[推地门控] 硬姿态趋势超限 backward=%d forward=%d "
                            "(pitch=%.3f jump_err=%.3f balance_err=%.3f gyro=%.3f)，短时冻结推地",
                            hard_backward_tendency ? 1 : 0, forward_tendency_excess ? 1 : 0,
                            pitch_, pitch_err, balance_pitch_err, pitch_rate_);
            } else {
                // soft backward 不冻结 motion time。它只是降低当前额外推力，
                // 让姿态控制获得时间纠正，同时避免机器人在接地状态“卡死成一条长腿”。
                thrust_motion_elapsed_ += dt;
                if (soft_backward_rate) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 80,
                        "[推地门控] soft-backward: pitch=%.3f err=%.3f gyro=%.3f，继续轨迹但降低额外推力",
                        pitch_, pitch_err, pitch_rate_);
                }
            }
        }

        // 目标速度闭环：以平滑的额外支撑力脉冲产生所需动量，而不是
        // 强制腿端跟踪一个可能与实际动力学不一致的高度五次轨迹。
        const double s = bbot_jump::clamp_value(thrust_motion_elapsed_ / thrust_duration_, 0.0, 1.0);
        const double one_minus_s = 1.0 - s;
        // 三次 Bezier 形状，端点额外推力为零；early/late 控制前后段形状。
        const double shape = 3.0 * one_minus_s * one_minus_s * s * thrust_shape_early_ +
                             3.0 * one_minus_s * s * s * thrust_shape_late_;
        const double mass_per_leg = TOTAL_MASS_ * 0.5;
        const double base_force = mass_per_leg * 9.81;
        // odom pose 是 base_link 原点，不是整机质心。轮子仍贴地时，
        // 机身已达 1.94 m/s，但其余 8 kg 部件没有同等向上速度。
        // 用同时间戳的质量加权世界 COM 速度闭环，避免提前卸力刹腿。
        // 估计缺失时仅保留有界开环推力，不用机身/FK速度冒充达标证据。
        const double vertical_velocity = centroidal_velocity_valid_ ?
            centroidal_height_.velocity() : 0.0;
        thrust_vertical_velocity_ = vertical_velocity;
        const double velocity_error = target_takeoff_velocity_ - vertical_velocity;
        const bool grounded_after_nominal_stroke =
            thrust_motion_elapsed_ >= thrust_duration_;
        const double feedback_force = centroidal_velocity_valid_ ?
            mass_per_leg * thrust_velocity_kp_ * velocity_error : 0.0;
        double extra_force = base_force * (thrust_peak_ratio_ - 1.0) * shape;
        const double feedback_ramp = bbot_jump::clamp_value(thrust_motion_elapsed_ / 0.06, 0.0, 1.0);
        double fb_force = feedback_ramp * feedback_force;

        // 推地末段仅在真实大后仰时平滑削减额外推力，避免为追速度恶化倾角，同时防止正常伸腿小扰动误清推力
        if (s > 0.4) {
            double penalty = 0.0;
            if (pitch_err < -0.04) penalty += (-pitch_err - 0.04) / 0.08;
            if (pitch_rate_ < -0.50) penalty += (-pitch_rate_ - 0.50) / 0.80;
            // 前倾同样需要削减额外冲量，不能等到硬阻塞阈值才处理。
            if (pitch_err > 0.06) penalty += (pitch_err - 0.06) / 0.08;
            if (pitch_rate_err > 0.40) penalty += (pitch_rate_err - 0.40) / 0.80;
            double thrust_pitch_attenuation = bbot_jump::clamp_value(1.0 - penalty, 0.2, 1.0);
            // soft-backward 只把额外推力降到约 45~70%，绝不把推进状态机锁住。
            if (soft_backward_rate) {
                const double soft_u = bbot_jump::clamp_value(
                    (-pitch_rate_ - 0.65) / 0.70, 0.0, 1.0);
                thrust_pitch_attenuation = std::min(
                    thrust_pitch_attenuation, bbot_jump::lerp(0.70, 0.45, soft_u));
            }
            extra_force *= thrust_pitch_attenuation;
            fb_force *= bbot_jump::clamp_value(0.55 + 0.45 * thrust_pitch_attenuation, 0.55, 1.0);
            // 离地速度反馈不能在轨迹末端被姿态衰减完全清零；真正的硬后仰
            // 才由上方 hard block 短时冻结并进入失败缩腿恢复。
        }

        // v5.8：terminal brake 不能只按轨迹相位启动。
        // v5.7 在 s≈0.63、真实世界 vz 仅约 1.0/1.98 m/s 时就开始收腿，
        // 结果还没形成足够竖直动量就进入“收尾”。现在只有当世界竖直速度
        // 已接近目标且轨迹进入后段时才允许刹腿。
        const double takeoff_speed_ratio = vertical_velocity /
            std::max(0.20, target_takeoff_velocity_);
        const double speed_brake_u = bbot_jump::clamp_value(
            (takeoff_speed_ratio - 0.82) / 0.16, 0.0, 1.0);
        const double speed_brake_blend =
            speed_brake_u * speed_brake_u * (3.0 - 2.0 * speed_brake_u);
        const double knee_speed_abs = std::max(
            std::abs(knee_vel_left_), std::abs(knee_vel_right_));
        const double knee_speed_u = bbot_jump::clamp_value(
            (knee_speed_abs - 5.0) / 4.0, 0.0, 1.0);
        const double phase_u = bbot_jump::clamp_value(
            (s - 0.65) / 0.20, 0.0, 1.0);
        const double phase_blend = phase_u * phase_u * (3.0 - 2.0 * phase_u);
        if (centroidal_velocity_valid_)
            thrust_release_.update(now_sec,s,vertical_velocity,target_takeoff_velocity_,thrust_force_per_leg_);
        const double terminal_brake_blend = thrust_release_.brake_blend(
            now_sec,speed_brake_blend * phase_blend * knee_speed_u);

        double F_z_request = base_force;
        if (!thrust_attitude_blocked_) {
            // 即使末段开始刹腿，也只小幅削减推进力；先保证跳起来。
            // 轮子真正离地后再由 FLIGHT 处理剩余关节速度。
            const double terminal_propulsive_scale =
                bbot_jump::clamp_value(1.0 - 0.25 * terminal_brake_blend, 0.75, 1.0);
            F_z_request += terminal_propulsive_scale * (extra_force + fb_force);
            // 在最新实测中，腿仍接地时 s 到 1 后 Bezier 额外推力归零，
            // 每腿请求力从约 200 N 降到 120 N，机身在 1.69 m/s 尚未
            // 离地便失去最后冲量。若尚未达到目标速度，保留一个受力矩
            // 预算限制的短促推力脉冲；离地确认后该状态立即退出。
            if (grounded_after_nominal_stroke && centroidal_velocity_valid_ &&
                velocity_error > 0.0 && !thrust_release_.active()) {
                const double speed_deficit = bbot_jump::clamp_value(
                    velocity_error / std::max(0.20, target_takeoff_velocity_),
                    0.70, 1.0);
                F_z_request += base_force * grounded_launch_boost_ratio_ *
                    speed_deficit;
            }
        } else {
            // 出现后仰趋势阻塞时：若机身已经在高速上升 (z_dot > 0.8)，
            // 立即削减支撑力，促使干净离地，切断地面反作用力对机身持续注入的后仰力矩
            if (vertical_velocity > 0.60) {
                const double unload_factor = bbot_jump::clamp_value(1.0 - thrust_block_elapsed_ / 0.03, 0.0, 1.0);
                F_z_request *= unload_factor;
            }
        }

        // 速度达到目标后撤去额外推力；保留小于重力的短暂支撑，避免末端
        // 力矩突变，下一状态由腾空腿部轨迹接管。
        if (centroidal_velocity_valid_ && velocity_error <= 0.0) {
            if (!velocity_takeoff_reached_) actual_takeoff_velocity_ = vertical_velocity;
            velocity_takeoff_reached_ = true;
            F_z_request = base_force;
        }

        // 轨迹完成后不再按时间卸载。若轮子仍接地，维持速度反馈推力；
        // 只有连续失重确认后才能退出 THRUST，避免产生“假腾空”。

        const double travel_scale = std::min({
            bbot_jump::joint_extension_scale(hip_pos_left_, hip_vel_left_),
            bbot_jump::joint_extension_scale(knee_pos_left_, knee_vel_left_),
            bbot_jump::joint_extension_scale(hip_pos_right_, hip_vel_right_),
            bbot_jump::joint_extension_scale(knee_pos_right_, knee_vel_right_)});
        if (travel_scale < thrust_extension_scale_) {
            if (thrust_extension_scale_ == 1.0) {
                RCLCPP_WARN(this->get_logger(),
                    "[末端行程保护] 提前卸力 scale=%.2f hip=%.3f/%.2f knee=%.3f/%.2f",
                    travel_scale, hip_pos_left_, hip_vel_left_, knee_pos_left_, knee_vel_left_);
            }
            thrust_extension_scale_ = travel_scale;
        }
        F_z_request *= thrust_extension_scale_;
        if (thrust_release_.active())
            F_z_request=std::min({F_z_request,thrust_release_.force_limit(now_sec),thrust_force_per_leg_});

        // 1. 固定的下蹲→伸腿轨迹只用于构型保持，按 thrust_motion_elapsed_ 推进
        double des_z, des_v, des_acc;
        quintic_traj_.evaluate(thrust_motion_elapsed_, des_z, des_v, des_acc);
        (void)des_acc;
        // v5.7：禁止再用 max(des_z,current_z_) 追着“已经超前伸出的实际腿长”继续伸。
        // 一旦实际腿超过规划，目标仍停在规划高度，让地面阶段的关节 PD 开始减速。
        const double target_ik_z = bbot_jump::clamp_value(
            des_z, L_SQUAT_, H_TAKEOFF_);
        current_height_ = target_ik_z;
        const auto ik = kinematics_.inverse_kinematics(target_ik_z, 0.0);
        // 名义推地轨迹已经走完却仍未离地时，不能再用固定 H_TAKEOFF
        // 的 IK 把已伸开的腿强行拉回去。上一轮中 current_z 已到 0.67 m，
        // 规划却停在 0.475 m，位置 PD 会撤掉尚未形成离地所需的最后冲量。
        // 此时保持当前构型，仅继续施加受力矩约束的竖直推力；一旦真正
        // 离地仍会立即转入原有 FLIGHT 流程。
        const bool grounded_thrust_hold = grounded_after_nominal_stroke;
        const double q_hip_des = grounded_thrust_hold ? hip_pos_left_ : ik.theta_hip;
        const double q_knee_des = grounded_thrust_hold ? knee_pos_left_ : ik.theta_knee;
        const double kp_hip_thrust = 18.0;
        const double kd_hip_thrust = 2.5;
        const double kp_knee_thrust = 22.0;
        const double kd_knee_thrust = 3.0;
        // 使用构造函数中配置的推地姿态增益。
        // 伸腿时髋关节抵消腿部反作用俯仰力矩；轮子同时按滚动起跳目标
        // 提供受限水平加速，以形成斜前方离地速度。
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);
        const double qdh_nominal = grounded_thrust_hold ?
            bbot_jump::thrust_extension_velocity(hip_vel_left_,3.0,1.0,thrust_extension_scale_) :
            thrust_extension_scale_ * bbot_jump::clamp_value(
                (dt > 1e-4) ? (q_hip_des - prev_q_hip_des_) / dt : 0.0, -3.0, 3.0);
        const double qdk_nominal = grounded_thrust_hold ?
            bbot_jump::thrust_extension_velocity(knee_vel_left_,6.0,-1.0,thrust_extension_scale_) :
            thrust_extension_scale_ * bbot_jump::clamp_value(
                (dt > 1e-4) ? (q_knee_des - prev_q_knee_des_) / dt : 0.0, -6.0, 6.0);
        // v5.8：末段只做“减速”而不是把期望速度直接拉到 0。
        // 这样在真正离地前仍保留足够的腿端伸展速度。
        const double qdh = (1.0 - 0.35 * terminal_brake_blend) * qdh_nominal;
        const double qdk = (1.0 - 0.75 * terminal_brake_blend) * qdk_nominal;
        const double q_hip_des_r = grounded_thrust_hold ? hip_pos_right_ : q_hip_des;
        const double q_knee_des_r = grounded_thrust_hold ? knee_pos_right_ : q_knee_des;
        const double qdh_r = grounded_thrust_hold ? (1.0-0.35*terminal_brake_blend)*
            bbot_jump::thrust_extension_velocity(hip_vel_right_,3.0,1.0,thrust_extension_scale_) : qdh;
        const double qdk_r = grounded_thrust_hold ? (1.0-0.75*terminal_brake_blend)*
            bbot_jump::thrust_extension_velocity(knee_vel_right_,6.0,-1.0,thrust_extension_scale_) : qdk;
        prev_q_hip_des_ = q_hip_des;
        prev_q_knee_des_ = q_knee_des;

        // 试验/调参前馈：当前模型中推地伸展对应 qdk<0。仅依据“将要发生的”
        // 膝伸展速度提前提供小幅正向髋补偿，抵消腿部内部反作用；有独立限幅，
        // 且在姿态门控阻塞时不允许它替代减推力保护。
        const double knee_extension_speed = std::max(0.0, -qdk);
        last_thrust_reaction_ff_ = bbot_jump::clamp_value(
            K_LEG_REACTION_FF_THRUST_ * knee_extension_speed,
            0.0, TAU_LEG_REACTION_FF_MAX_);
        const double tau_body_per_hip = bbot_jump::clamp_value(
            -0.5 * (K_BODY_P_THRUST_ * pitch_err +
                    K_BODY_D_THRUST_ * pitch_rate_err) +
                last_thrust_reaction_ff_,
            -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        last_tau_body_per_hip_ = tau_body_per_hip;

        // 髋关节在推地阶段的核心任务是姿态稳定(tau_body_per_hip)，构型保持只做低增益补偿；
        // 尤其在后仰趋势时，禁止髋关节PD产生负向拉扯力矩加剧后仰。
        const double pd_h_min = (pitch_rate_ < 0.0 || pitch_err < 0.0) ? 0.0 : -3.0;
        const double pd_h_l = bbot_jump::clamp_value(
            kp_hip_thrust * (q_hip_des - hip_pos_left_) +
            kd_hip_thrust * (qdh - hip_vel_left_), pd_h_min, 3.0);
        const double terminal_knee_brake_l = terminal_brake_blend * bbot_jump::clamp_value(
            -1.6 * knee_vel_left_, -12.0, 12.0);
        const bool thrust_position_yield = centroidal_velocity_valid_ &&
            vertical_velocity < 0.95*target_takeoff_velocity_ &&
            !thrust_attitude_blocked_ && !thrust_release_.active() &&
            terminal_brake_blend==0.0 && thrust_extension_scale_>=1.0;
        thrust_knee_position_yield_=thrust_position_yield;
        const double pd_k_l = bbot_jump::clamp_value(
            kp_knee_thrust * bbot_jump::thrust_knee_position_error(
                q_knee_des,knee_pos_left_,thrust_position_yield) +
            kd_knee_thrust * (qdk - knee_vel_left_) + terminal_knee_brake_l,
            -22.0, 22.0);
        const double pd_h_r = bbot_jump::clamp_value(
            kp_hip_thrust * (q_hip_des_r - hip_pos_right_) +
            kd_hip_thrust * (qdh_r - hip_vel_right_), pd_h_min, 3.0);
        const double terminal_knee_brake_r = terminal_brake_blend * bbot_jump::clamp_value(
            -1.6 * knee_vel_right_, -12.0, 12.0);
        const double pd_k_r = bbot_jump::clamp_value(
            kp_knee_thrust * bbot_jump::thrust_knee_position_error(
                q_knee_des_r,knee_pos_right_,thrust_position_yield) +
            kd_knee_thrust * (qdk_r - knee_vel_right_) + terminal_knee_brake_r,
            -22.0, 22.0);
        // 推地任务解耦：膝关节负责竖直冲量，髋关节只负责机身姿态。
        // 本机构的 J^T Fz 髋力矩方向会抵消后仰纠姿力矩；即使只分配 25%，
        // 实测离地前也会把 +15 N*m 左右的纠姿命令抵消掉大半，使机器人
        // 带着持续负 pitch_rate 离地。因此推地阶段不再把竖直力映射到髋。
        constexpr double thrust_hip_force_share = 0.0;
        const double tau_attitude_reserve = 8.0;
        const auto effort_limits = current_effort_limits();
        const double hip_budget = thrust_torque_margin_*effort_limits.hip-tau_attitude_reserve;
        const double knee_budget = thrust_torque_margin_*effort_limits.knee;
        // 对最终合力矩求预算。膝伸展 J<0 时，正向 PD 是反向制动，
        // 不能既从推力预算扣除一次，又在最终力矩上再抵消一次。
        const double force_limit_left = std::min(
            bbot_jump::signed_force_limit(thrust_hip_force_share*jh_left,tau_body_per_hip+pd_h_l,hip_budget),
            bbot_jump::signed_force_limit(jk_left,pd_k_l,knee_budget));
        const double force_limit_right = std::min(
            bbot_jump::signed_force_limit(thrust_hip_force_share*jh_right,tau_body_per_hip+pd_h_r,hip_budget),
            bbot_jump::signed_force_limit(jk_right,pd_k_r,knee_budget));
        thrust_force_unlimited_request_ = F_z_request;
        thrust_knee_pd_left_ = pd_k_l;
        const double force_limit = std::max(0.0, std::min(force_limit_left, force_limit_right));
        F_z_request = bbot_jump::clamp_value(F_z_request, 0.0, force_limit);
        last_thrust_force_request_ = F_z_request;
        last_thrust_force_limit_ = force_limit;
        const double max_force_step = 1600.0 * std::max(0.001, dt);
        thrust_force_per_leg_ += bbot_jump::clamp_value(
            F_z_request - thrust_force_per_leg_, -max_force_step, max_force_step);
        // 预算突然收紧时，斜率限制不能让旧推力继续超过当前可用力矩。
        thrust_force_per_leg_ = std::min(thrust_force_per_leg_, force_limit);
        if (thrust_extension_scale_ < 1.0 || thrust_release_.active())
            thrust_force_per_leg_ = std::min(thrust_force_per_leg_, F_z_request);
        double F_z_thrust = thrust_force_per_leg_;
        // 竖直推力主要由膝关节产生；髋部只承担少量支撑并优先控制机身姿态。
        // 完整 J^T Fz 髋力矩在本模型上约 -50 Nm，其基座反力矩会把机身持续向后掀。
        const double tau_ff_hip_left = thrust_hip_force_share * F_z_thrust * jh_left;
        const double tau_ff_knee_left = F_z_thrust * jk_left;
        const double tau_ff_hip_right = thrust_hip_force_share * F_z_thrust * jh_right;
        const double tau_ff_knee_right = F_z_thrust * jk_right;

        if (terminal_brake_blend > 0.05) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 80,
                "[THRUST_TERMINAL] s=%.2f blend=%.2f vz=%.2f/%.2f "
                "knee_v=%.2f/%.2f qdk=%.2f brake_tau=%.1f/%.1f F=%.1f",
                s, terminal_brake_blend, vertical_velocity, target_takeoff_velocity_,
                knee_vel_left_, knee_vel_right_, qdk,
                terminal_knee_brake_l, terminal_knee_brake_r, F_z_thrust);
        }

        // 2. 髋关节姿态稳定前馈补偿；PD 只做低增益构型保持。
        publish_effort_leg_control_lr(
            q_hip_des, q_knee_des, q_hip_des_r, q_knee_des_r,
            qdh, qdk, qdh_r, qdk_r,
            tau_ff_hip_left + tau_body_per_hip + pd_h_l, tau_ff_knee_left + pd_k_l,
            tau_ff_hip_right + tau_body_per_hip + pd_h_r, tau_ff_knee_right + pd_k_r,
            // 使用上方与力矩预算一致的限幅 PD，通用下发层不得重复叠加。
            0.0, 0.0, 0.0, 0.0,
            tau_body_per_hip, tau_body_per_hip);

        // rolling-THRUST：从接近速度继续加速到更高的离地前向速度。
        // 当前符号链中负 cmd 对应正 x_dot，因此速度不足时让命令进一步变负。
        const double forward_speed_error = jump_takeoff_forward_speed_ - x_dot_;
        const double thrust_forward_cmd_mag = bbot_jump::clamp_value(
            jump_takeoff_forward_speed_ +
                thrust_forward_velocity_kp_ * forward_speed_error,
            0.0, 0.85);
        // v6.4：名义伸腿时间结束不代表可以反向制动车轮。带前倾时
        // 撤回支撑点会加剧前倒；直到离地都保留与 PRE_JUMP 同符号的姿态反馈。
        interpolate_lqr_gain();
        const double thrust_wheel_target = bbot_jump::thrust_ground_wheel_target(
            thrust_forward_cmd_mag, pitch_err, pitch_rate_err,
            current_gain_.k_theta, current_gain_.k_theta_dot);
        const double max_thrust_wheel_step = 8.0 * std::max(dt, 0.001);
        const double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            thrust_wheel_target - last_wheel_cmd_x_,
            -max_thrust_wheel_step, max_thrust_wheel_step);
        publish_wheel_cmd(cmd_x, 0.0);

        last_thrust_force_ = F_z_thrust;
        thrust_mechanical_work_ += std::abs(F_z_thrust * vertical_velocity) * dt * 2.0;


        // v6.3：同一odom时间戳的世界高度、机身姿态、插值关节构型。
        // 接地快速伸腿时，旧高度减新FK会制造厘米级负间隙，掩盖真实离地。
        double geom_left=0.0, geom_right=0.0;
        takeoff_geometry_aligned_ = aligned_takeoff_geometry(now_sec,geom_left,geom_right);
        takeoff_clearance_left_ = takeoff_geometry_aligned_ ?
            gazebo_world_z_-ground_height_offset_-geom_left : -1.0;
        takeoff_clearance_right_ = takeoff_geometry_aligned_ ?
            gazebo_world_z_-ground_height_offset_-geom_right : -1.0;
        wheel_clearance_ = std::min(takeoff_clearance_left_,takeoff_clearance_right_);
        if (vertical_velocity>0.35 && gazebo_world_z_-thrust_start_world_z_>0.012)
            takeoff_rise_observed_=true;
        const bool accel_unloaded = acc_z_filt_<8.5 && acc_z_raw_<7.5;
        // 已观察到起跳上升后，不因临近顶点vz降低而永久禁止确认离地。
        // 普通净空仍需失重证据；两轮均超过20mm时可用连续几何确认，排除单次IMU冲击。
        const bool wheels_airborne = takeoff_confirmation_.update(
            takeoff_odom_stamp_,now_sec,takeoff_geometry_aligned_ && elapsed>0.10,
            takeoff_clearance_left_,takeoff_clearance_right_,accel_unloaded,takeoff_rise_observed_);
        airborne_confidence_count_=takeoff_confirmation_.count();
        last_wheels_airborne_=wheels_airborne;
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),70,
            "[TAKEOFF_SYNC] t=%.3f aligned=%d stamp=%.3f age=%.3f clrL=%.4f clrR=%.4f "
            "vz=%.2f az=%.1f/%.1f rise=%d conf=%d airborne=%d",
            elapsed,takeoff_geometry_aligned_?1:0,takeoff_odom_stamp_,now_sec-takeoff_odom_stamp_,
            takeoff_clearance_left_,takeoff_clearance_right_,vertical_velocity,acc_z_filt_,acc_z_raw_,
            takeoff_rise_observed_?1:0,airborne_confidence_count_,wheels_airborne?1:0);
        log_data(cmd_x, 0.5 * (tau_ff_hip_left + tau_ff_hip_right),
                 0.5 * (tau_ff_knee_left + tau_ff_knee_right), F_z_thrust * 2.0);

        if (velocity_takeoff_reached_ && vertical_velocity >= target_takeoff_velocity_ * 0.98) {
            velocity_reached_count_ = std::min(velocity_reached_count_ + 1, 20);
        } else {
            velocity_reached_count_ = 0;
        }
        bool velocity_takeoff = velocity_reached_count_ >= 3;
        // 速度或腿长变化均不能替代失重确认。只有轮子连续确认失重后，
        // 才允许进入 FLIGHT 并执行收腿/落地时序。
        const bool airborne_takeoff = wheels_airborne;
        bool leg_collapsing = (elapsed > 0.04 && current_z_ < state_start_z_ - 0.025);
        bool thrust_timeout = (elapsed >= thrust_timeout_);
        bool attitude_ready = (std::abs(pitch_err) < 0.35 && std::abs(pitch_rate_) < 2.5);

        if (leg_collapsing) {
            abort_jump_to_recovery(now_sec, "推地期间腿长反向缩短");
            return;
        }

        if (thrust_timeout && !wheels_airborne) {
            abort_jump_to_recovery(now_sec, "推地超时且轮子未确认离地");
            return;
        }

        // 姿态失稳保护。确认已经离地后立即停止推地；如果姿态不满足正常
        // 腾空门槛，则以 protective landing 进入 FLIGHT，优先展腿保命，
        // 不能继续在接地推力下等待姿态“变好”。
        if (thrust_timeout && !attitude_ready) {
            transition_to_protective_landing(now_sec, "离地前姿态或角速度未稳定");
            return;
        }

        if (airborne_takeoff) {
            takeoff_q_hip_left_ = hip_pos_left_;
            takeoff_q_knee_left_ = knee_pos_left_;
            takeoff_q_hip_right_ = hip_pos_right_;
            takeoff_q_knee_right_ = knee_pos_right_;
            takeoff_pitch_rate_ = pitch_rate_;
            takeoff_forward_speed_ = x_dot_;
            // Gazebo 世界速度的“前进方向”不一定是 world X。本次日志中
            // 离地 vx(wheel)=0.45 m/s，而 odom.linear.x≈0，说明机器人实际沿
            // world Y 或其它水平航向运动。离地瞬间锁存水平速度向量作为前向轴，
            // FLIGHT 中用 dot([vx,vy], forward_axis) 得到真实机身前向速度。
            const double world_horizontal_speed = std::hypot(
                gazebo_world_x_dot_, gazebo_world_y_dot_);
            if (world_xy_dot_filter_initialized_ &&
                world_horizontal_speed > 0.05 &&
                std::abs(takeoff_forward_speed_) > 0.05) {
                landing_forward_axis_x_ = gazebo_world_x_dot_ / world_horizontal_speed;
                landing_forward_axis_y_ = gazebo_world_y_dot_ / world_horizontal_speed;
                landing_forward_direction_sign_ =
                    (takeoff_forward_speed_ >= 0.0) ? 1.0 : -1.0;
                landing_forward_axis_valid_ = true;
            }
            // 滚动离地时轮子已经旋转。空中轮控必须围绕此基准做增量，
            // 不能把“目标0”解释成离地后立即刹轮，否则会额外注入俯仰冲量。
            air_wheel_baseline_ = last_wheel_cmd_x_;
            target_speed_const_ = 0.0;
            target_speed_smoothed_ = 0.0;
            // 从这一帧起，正俯仰角速度任务完成；FLIGHT 负责把姿态平滑过渡到着陆工作点。
            active_jump_pitch_rate_ref_ = 0.0;
            flight_pitch_ref_start_ = pitch_;
            flight_air_pitch_ref_diag_ = pitch_;
            flight_air_pitch_rate_ref_diag_ = 0.0;

            // v5.8：空中仍不把高速腿强行刹到 0；真正离地由轮间隙确认后才进入这里。
            // 地面 THRUST 已先做 terminal brake；若仍有残余速度，空中这里只在
            // 60 ms 内降到“可规划速度”(髋±2、膝±4 rad/s)，随后用整段落地轨迹消掉。
            // 这样显著降低内部角动量瞬间转移到机身的峰值。
            constexpr double arrest_duration = 0.060;
            const double vh_l = hip_vel_left_;
            const double vk_l = knee_vel_left_;
            const double vh_r = hip_vel_right_;
            const double vk_r = knee_vel_right_;
            const auto end_v_hip = [](double v) {
                return bbot_jump::clamp_value(v, -2.0, 2.0);
            };
            const auto end_v_knee = [](double v) {
                return bbot_jump::clamp_value(v, -4.0, 4.0);
            };
            const auto end_q = [&](double q, double v0, double vf) {
                return bbot_jump::clamp_value(
                    q + 0.5 * (v0 + vf) * arrest_duration, -1.36, 1.36);
            };
            const double vh_l_end = end_v_hip(vh_l);
            const double vk_l_end = end_v_knee(vk_l);
            const double vh_r_end = end_v_hip(vh_r);
            const double vk_r_end = end_v_knee(vk_r);
            arrest_hip_left_traj_.init(now_sec, arrest_duration,
                takeoff_q_hip_left_, vh_l, 0.0, end_q(takeoff_q_hip_left_, vh_l, vh_l_end), vh_l_end, 0.0);
            arrest_knee_left_traj_.init(now_sec, arrest_duration,
                takeoff_q_knee_left_, vk_l, 0.0, end_q(takeoff_q_knee_left_, vk_l, vk_l_end), vk_l_end, 0.0);
            arrest_hip_right_traj_.init(now_sec, arrest_duration,
                takeoff_q_hip_right_, vh_r, 0.0, end_q(takeoff_q_hip_right_, vh_r, vh_r_end), vh_r_end, 0.0);
            arrest_knee_right_traj_.init(now_sec, arrest_duration,
                takeoff_q_knee_right_, vk_r, 0.0, end_q(takeoff_q_knee_right_, vk_r, vk_r_end), vk_r_end, 0.0);

            const char * takeoff_reason = velocity_takeoff ? "达到目标离地速度并确认失重" : "已确认失重（速度不足）";
            if (!velocity_takeoff) {
                jump_failure_reason_ = "已离地但未达到目标速度";
                actual_takeoff_velocity_ = vertical_velocity;
            }
            const double flight_entry_pitch_err = pitch_ - balance_offset_;
            const double takeoff_ref_pitch_err = pitch_ - active_jump_pitch_ref_;
            // 斜向起跳的绝对前倾本来就是计划状态，不能再用静态 balance_offset
            // 的 0.15 rad 阈值把正常斜跳直接判成 protective。优先看相对动态
            // 起跳参考的误差，同时保留更宽的绝对姿态硬保护。
            const bool severe_attitude =
                std::abs(takeoff_ref_pitch_err) > 0.14 ||
                std::abs(flight_entry_pitch_err) > 0.30 ||
                std::abs(pitch_rate_) > 1.20;
            // v6.1：protective 判定只看“当前实际关节状态”，不能使用
            // thrust_extension_scale_ 的历史最小值。上一轮离地前曾触发 scale=0.40，
            // 但真正离地时 hip_v≈1.1、knee_v≈-1.3 已经安全；若仍把历史 scale
            // 当作 unsafe，会永久跳过 TUCK，直接进入保护展腿，造成长腿保持到触地。
            const bool unsafe_leg_motion = !flight_leg_motion_ready();
            const bool protective_takeoff = severe_attitude || unsafe_leg_motion;
            if (protective_takeoff) {
                jump_failure_reason_ = severe_attitude ? "离地姿态超标" : "离地关节速度/行程超标";
                RCLCPP_WARN(this->get_logger(),
                            "[离地保护] 已确认离地，姿态或腿部运动超标 "
                            "(pitch=%.3f, flight_err=%.3f, gyro=%.3f)，先执行ATTITUDE_ARREST再规划落地构型",
                            pitch_, flight_entry_pitch_err, pitch_rate_);
            }
            RCLCPP_INFO(this->get_logger(),
                        ">>> 离地爆发完成 [%s] (t=%.3fs, z=%.3f, vz=%.2f, vx=%.2f, pitch=%.3f, gyro=%.3f, acc_z=%.1f, conf=%d, normal=%d)！"
                        "切入腾空相 (FLIGHT)... <<<",
                        takeoff_reason, elapsed, current_z_, vertical_velocity, takeoff_forward_speed_,
                        pitch_, pitch_rate_, acc_z_filt_, airborne_confidence_count_,
                        (!protective_takeoff) ? 1 : 0);
            current_state_ = bbot_jump::STATE_FLIGHT;
            state_start_time_ = now_sec;
            flight_start_z_ = current_z_;
            flight_ff_force_start_ = 0.0; // 空中阶段不再保留 30 ms 的推地支撑力前馈
            last_thrust_force_ = 0.0;
            last_thrust_force_request_ = 0.0;
            last_thrust_force_limit_ = 0.0;
            last_tau_body_per_hip_ = 0.0;
            last_thrust_reaction_ff_ = 0.0;
            flight_trajectory_initialized_ = false;
            touchdown_knee_effort_count_ = 0;
            touchdown_stable_count_ = 0;
            protective_landing_ = protective_takeoff;
            current_height_ = flight_start_z_;

            // 初始化空中子阶段。
            // v5.6：即使因“离地关节速度高”被标记为 protective，也绝不能从 THRUST
            // 直接跳到 PROTECTIVE_DEPLOY。上一轮就是这样绕过了已初始化的 0.10s
            // arrest 轨迹，高速腿在整个展腿轨迹里持续向机身注入反向角动量。
            attitude_arrest_start_time_ = now_sec;
            attitude_arrest_stable_count_ = 0;
            tuck_started_ = false;
            tuck_start_timestamp_ = 0.0;
            protective_deploy_initialized_ = false;
            protective_deploy_start_time_ = 0.0;
            flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST;
            if (protective_takeoff) {
                RCLCPP_WARN(this->get_logger(),
                            "[FLIGHT接管] protective_takeoff=1，但先执行 ATTITUDE_ARREST；"
                            "当前 hip_v=%.2f/%.2f knee_v=%.2f/%.2f，禁止高速腿直接展到落地构型",
                            hip_vel_left_, hip_vel_right_, knee_vel_left_, knee_vel_right_);
            }

            // 保持力矩控制模式
            request_effort_controller();
        }
    }

    // v5.9：当前 bbot_kinematics 公开接口仍只有 inverse_kinematics(target_z, body_pitch)，
    // 但其内部不是理想对称二连杆，而是包含真实 CAD 初始角和轮轴/髋部纵向偏置。
    // 因此不能再用“等效腿长 + 只旋转髋角”的近似。这里直接复用同一套 CAD 几何，
    // 增加 longitudinal target_x，同时保持 bbot_kinematics API 不变。
    //
    // target_x > 0：机身/髋在轮轴前方，也就是轮子相对机身向后布置。
    bbot_kinematics::IKSolution inverse_kinematics_with_target_x(
        double target_z, double body_pitch, double target_x) const
    {
        const auto & p = kinematics_.get_params();
        constexpr double kHipBodyVerticalOffset = 0.07;
        constexpr double kCadWheelToHipLongitudinal = -0.01137221;
        const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
        const double phi2_0 = std::atan2( 0.28210870, 0.19553796);

        // 与 bbot_kinematics::inverse_kinematics() 完全相同的 target_z 定义：
        // target_z 为机身离地高度，先扣除髋->机身竖直偏置和轮半径。
        double dZ_down = target_z - (kHipBodyVerticalOffset + p.wheel_radius);
        dZ_down = bbot_jump::clamp_value(dZ_down, 0.10, 0.60);

        // 正 target_x 代表 wheel behind body，因此 wheel 相对 hip 的纵向坐标更负。
        const double x = bbot_jump::clamp_value(target_x, -0.12, 0.14);
        const double target_dY = kCadWheelToHipLongitudinal - x;

        const double d_sq = target_dY * target_dY + dZ_down * dZ_down;
        const double d = std::max(1e-6, std::sqrt(d_sq));
        const double l_thigh = p.l2;
        const double l_shank = p.l1;

        double cos_gamma =
            (l_thigh * l_thigh + l_shank * l_shank - d_sq) /
            (2.0 * l_thigh * l_shank);
        cos_gamma = bbot_jump::clamp_value(cos_gamma, -1.0, 1.0);
        const double gamma = std::acos(cos_gamma);

        const double theta_d = std::atan2(target_dY, dZ_down);

        double cos_psi =
            (l_thigh * l_thigh + d_sq - l_shank * l_shank) /
            (2.0 * l_thigh * d);
        cos_psi = bbot_jump::clamp_value(cos_psi, -1.0, 1.0);
        const double psi = std::acos(cos_psi);

        const double phi1 = theta_d - psi;
        const double phi2 = phi1 + (M_PI - gamma);

        bbot_kinematics::IKSolution ik;
        ik.theta_shank = phi2;
        ik.theta_hip = phi1 - phi1_0 + body_pitch;
        ik.theta_knee = (phi2 - phi1) - (phi2_0 - phi1_0);
        return ik;
    }

    double landing_shank_limit() const
    {
        const auto & p = kinematics_.get_params();
        double limit = landing_shank_abs_max_;
        if (p.l1 > 1e-6) {
            const double ratio = bbot_jump::clamp_value(
                landing_knee_axis_clearance_min_ / p.l1, 0.0, 0.999);
            const double clearance_limit = std::acos(ratio);
            limit = std::min(limit, clearance_limit);
        }
        return bbot_jump::clamp_value(limit, 0.35, 1.20);
    }

    // 落地接近阶段的最后一道几何保护：
    // theta_shank = phi2_0 + q_hip + q_knee - body_pitch。
    // 若期望小腿已经接近水平，直接调整膝目标把小腿拉回安全锥内；
    // 同时将期望小腿角速度限制为不继续向危险方向增大。
    bool enforce_wheel_first_target(
        double & q_hip, double & q_knee,
        double & qd_hip, double & qd_knee) const
    {
        const double phi2_0 = std::atan2(0.28210870, 0.19553796);
        const double limit = landing_shank_limit();
        const double shank = phi2_0 + q_hip + q_knee - pitch_;
        const double shank_safe = bbot_jump::clamp_value(shank, -limit, limit);
        const bool clamped = std::abs(shank_safe - shank) > 1e-6;
        if (clamped) {
            q_knee += (shank_safe - shank);
            q_knee = bbot_jump::clamp_value(q_knee, -1.36, 1.36);

            // 目标绝对小腿角速度 = qd_hip + qd_knee - pitch_rate。
            // 在几何边界上不允许继续向外旋转。
            const double shank_rate = qd_hip + qd_knee - pitch_rate_;
            const double shank_after = phi2_0 + q_hip + q_knee - pitch_;
            if ((shank_after >= limit - 1e-3 && shank_rate > 0.0) ||
                (shank_after <= -limit + 1e-3 && shank_rate < 0.0)) {
                qd_knee = pitch_rate_ - qd_hip;
            }
        }
        return clamped;
    }

    double body_height_above_wheel_ground(double hip, double knee) const
    {
        // URDF: base_link -> hip = (x, 0.125, -0.07), pitch = -roll。
        // FK 把机身偏移简化为固定 0.07；计算触地间隙时必须旋转此偏移。
        return kinematics_.calculate_com_height(pitch_, hip, knee) +
            0.125 * std::sin(pitch_) + 0.07 * (std::cos(pitch_) - 1.0);
    }

    bool aligned_takeoff_geometry(double now_sec, double & left, double & right) const
    {
        std::array<double,4> q{};
        if (!takeoff_odom_pose_valid_ || takeoff_odom_stamp_<=0.0 ||
            now_sec-takeoff_odom_stamp_>0.080 || takeoff_odom_stamp_>now_sec+0.001 ||
            !takeoff_joint_history_.interpolate(takeoff_odom_stamp_,q)) return false;
        const auto height = [&](double hip, double knee) {
            return kinematics_.calculate_com_height(takeoff_odom_pitch_,hip,knee) +
                0.125*std::sin(takeoff_odom_pitch_) + 0.07*(std::cos(takeoff_odom_pitch_)-1.0);
        };
        left=height(q[0],q[1]); right=height(q[2],q[3]);
        return true;
    }

    bool flight_leg_motion_ready() const
    {
        // 仅用于“是否可开始收腿”的安全准入，不应把已经充分伸开的
        // 起跳构型误判为只能展腿保护。上一轮真实离地时 knee≈-1.31 rad，
        // 旧的 1.10 rad 阈值使它永远无法进入 TUCK。这里保留距轨迹硬限
        // 位约 0.02 rad 的裕量，并继续限制关节速度，避免高速反向收腿。
        return std::abs(hip_vel_left_) <= 4.0 && std::abs(hip_vel_right_) <= 4.0 &&
            std::abs(knee_vel_left_) <= 8.0 && std::abs(knee_vel_right_) <= 8.0 &&
            std::abs(hip_pos_left_) <= 1.30 && std::abs(hip_pos_right_) <= 1.30 &&
            std::abs(knee_pos_left_) <= 1.34 && std::abs(knee_pos_right_) <= 1.34;
    }

    double predicted_landing_forward_velocity(double now_sec) const
    {
        // FLIGHT 中轮速 x_dot_ 已被反作用轮姿态控制污染，不能代表机身平移速度。
        // 使用离地瞬间锁存的世界水平速度方向，把 odom [vx,vy] 投影到该轴。
        const bool fresh_world_v = odom_received_ &&
            world_xy_dot_filter_initialized_ &&
            landing_forward_axis_valid_ &&
            (now_sec - last_world_odom_time_ <= 0.10) &&
            std::isfinite(gazebo_world_x_dot_) &&
            std::isfinite(gazebo_world_y_dot_);
        if (fresh_world_v) {
            const double projected =
                gazebo_world_x_dot_ * landing_forward_axis_x_ +
                gazebo_world_y_dot_ * landing_forward_axis_y_;
            return landing_forward_direction_sign_ * projected;
        }
        return takeoff_forward_speed_;
    }

    double preview_landing_target_x(double now_sec) const
    {
        const double vx_raw = predicted_landing_forward_velocity(now_sec);
        const double vx = (std::abs(vx_raw) >= landing_capture_speed_deadband_) ? vx_raw : 0.0;
        const double h = bbot_jump::clamp_value(landing_capture_height_, 0.30, 0.50);
        const double omega = std::sqrt(9.81 / h);
        const double raw_capture = (omega > 1e-6) ? (vx / omega) : 0.0;
        const double forward_lead = landing_capture_gain_ * raw_capture;
        // target_x>0 => body 在 wheel 前方 => wheel 相对 body 后移。
        // 速度项按经典 capture 方向把轮子稍向前修正，因此从 back_bias 中减去。
        return bbot_jump::clamp_value(
            landing_wheel_back_bias_ - forward_lead,
            -0.03, landing_target_x_max_);
    }

    void latch_landing_capture_plan(double now_sec)
    {
        landing_capture_vx_ = predicted_landing_forward_velocity(now_sec);
        const double vx = (std::abs(landing_capture_vx_) >= landing_capture_speed_deadband_) ?
            landing_capture_vx_ : 0.0;
        const double h = bbot_jump::clamp_value(landing_capture_height_, 0.30, 0.50);
        landing_capture_omega_ = std::sqrt(9.81 / h);
        landing_capture_raw_offset_ =
            (landing_capture_omega_ > 1e-6) ? (vx / landing_capture_omega_) : 0.0;
        landing_capture_offset_ = landing_capture_gain_ * landing_capture_raw_offset_;
        landing_target_x_ = bbot_jump::clamp_value(
            landing_wheel_back_bias_ - landing_capture_offset_,
            -0.03, landing_target_x_max_);
        // 仅用于日志直观展示虚拟腿偏角，绝不再作为 body_pitch 传给 IK。
        landing_capture_comp_ = std::atan2(
            landing_target_x_, std::max(0.20, L_TOUCH_));
        landing_capture_planned_ = true;

        const auto landing_ik = inverse_kinematics_with_target_x(
            L_TOUCH_, balance_offset_ + flight_landing_pitch_bias_, landing_target_x_);
        const auto & p = kinematics_.get_params();
        landing_target_shank_abs_ = landing_ik.theta_shank;
        landing_target_knee_axis_clearance_ =
            p.l1 * std::cos(landing_target_shank_abs_);

        RCLCPP_INFO(this->get_logger(),
                    "[LANDING_PLACEMENT] vx_body=%.3f omega=%.3f raw_cp=%.3f "
                    "forward_lead=%.3f back_bias=%.3f -> target_x=%.3f m, "
                    "shank=%.3f rad, knee_axis_above_wheel=%.3f m",
                    landing_capture_vx_, landing_capture_omega_,
                    landing_capture_raw_offset_, landing_capture_offset_,
                    landing_wheel_back_bias_, landing_target_x_,
                    landing_target_shank_abs_, landing_target_knee_axis_clearance_);
    }

    bool tuck_round_trip_feasible(double now_sec, double tuck_duration,
                                  double extend_duration, double time_available)
    {
        if (!flight_leg_motion_ready() || time_available < tuck_duration + extend_duration)
            return false;
        std::array<double, 4> q{}, v{}, a{};
        sample_flight_joints(now_sec, q, v, a);
        const double tuck_comp = bbot_jump::clamp_value(
            pitch_ - balance_offset_, -0.30, 0.30);
        const double landing_target_x = preview_landing_target_x(now_sec);
        const auto tuck = inverse_kinematics_with_target_x(L_RETRACT_, tuck_comp, 0.0);
        const auto land = inverse_kinematics_with_target_x(
            L_TOUCH_, balance_offset_ + flight_landing_pitch_bias_, landing_target_x);
        const std::array<double, 4> mid{tuck.theta_hip, tuck.theta_knee, tuck.theta_hip, tuck.theta_knee};
        const std::array<double, 4> end{land.theta_hip, land.theta_knee, land.theta_hip, land.theta_knee};
        const std::array<double, 4> actual{hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_};
        for (size_t i = 0; i < q.size(); ++i) {
            if (std::abs(q[i] - actual[i]) > 0.12) return false;
            bbot_jump::QuinticTrajectory retract, deploy;
            retract.init(now_sec, tuck_duration, q[i], v[i], a[i], mid[i], 0.0, 0.0);
            deploy.init(now_sec + tuck_duration, extend_duration, mid[i], 0.0, 0.0, end[i], 0.0, 0.0);
            // v6.1：上一版的 6/8 rad/s、120/160 rad/s² 准入对低跳过严，
            // 即使离地关节速度已经安全，也会直接判定“不可TUCK”，从而整段保持长腿。
            // 这里仍保留有限速度/加速度上限，但允许约0.10s内完成一次明显收腿。
            const double vmax = (i % 2 == 0) ? 7.5 : 10.0;
            const double amax = (i % 2 == 0) ? 240.0 : 320.0;
            if (!bbot_jump::flight_trajectory_admissible(retract, vmax, amax) ||
                !bbot_jump::flight_trajectory_admissible(deploy, vmax, amax)) return false;
        }
        return true;
    }

    // 以同一时刻的旧轨迹作为新轨迹边界，避免子阶段切换时重置目标。
    void sample_flight_joints(double now_sec, std::array<double, 4> & q,
                              std::array<double, 4> & v, std::array<double, 4> & a) const
    {
        if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) {
            arrest_hip_left_traj_.evaluate(now_sec, q[0], v[0], a[0]);
            arrest_knee_left_traj_.evaluate(now_sec, q[1], v[1], a[1]);
            arrest_hip_right_traj_.evaluate(now_sec, q[2], v[2], a[2]);
            arrest_knee_right_traj_.evaluate(now_sec, q[3], v[3], a[3]);
        } else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY) {
            protective_hip_left_traj_.evaluate(now_sec, q[0], v[0], a[0]);
            protective_knee_left_traj_.evaluate(now_sec, q[1], v[1], a[1]);
            protective_hip_right_traj_.evaluate(now_sec, q[2], v[2], a[2]);
            protective_knee_right_traj_.evaluate(now_sec, q[3], v[3], a[3]);
        } else {
            for (size_t i = 0; i < q.size(); ++i)
                normal_flight_joint_traj_[i].evaluate(now_sec, q[i], v[i], a[i]);
        }
    }

    void plan_flight_joints(double now_sec, double duration, double height,
                            bool protective, double body_pitch_ref, double target_x)
    {
        std::array<double, 4> q{hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_};
        std::array<double, 4> v{hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_};
        std::array<double, 4> a{};
        if (flight_trajectory_initialized_) sample_flight_joints(now_sec, q, v, a);
        const auto ik = inverse_kinematics_with_target_x(
            height, bbot_jump::clamp_value(body_pitch_ref, -0.30, 0.30),
            target_x);
        const std::array<double, 4> end{ik.theta_hip, ik.theta_knee, ik.theta_hip, ik.theta_knee};
        std::array<bbot_jump::QuinticTrajectory *, 4> dest{
            &protective_hip_left_traj_, &protective_knee_left_traj_,
            &protective_hip_right_traj_, &protective_knee_right_traj_};
        if (protective) protective_deploy_rate_limited_ = false;
        for (size_t i = 0; i < q.size(); ++i) {
            auto & traj = protective ? *dest[i] : normal_flight_joint_traj_[i];
            double q_end = end[i];
            if (protective) {
                // 短腾空保护不能为追到理想着陆 IK 而在一两个采样周期内
                // 反向甩髋。零端速五次轨迹的峰值速度约为 1.875*Δq/T，
                // v6.1 提高到约50%的速度预算，并允许段末继续replan；
                // 这样既避免单帧反向甩腿，也不会永久停在中间长腿构型。
                const bool hip_joint = (i % 2) == 0;
                const double speed_limit = hip_joint ? 5.0 : 7.0;
                const double max_delta = 0.50 * speed_limit * std::max(0.05, duration);
                const double requested_delta = end[i] - q[i];
                double bounded_delta = bbot_jump::clamp_value(
                    requested_delta, -max_delta, max_delta);

                // 当髋仍沿反方向高速摆动时，短时间内强制反向会把等量
                // 角动量传给机身。髋关节先做短暂减速，不把这一帧变成
                // 反向摆腿命令；膝关节仍可配合 wheel-first 几何约束展开。
                if (hip_joint && std::abs(v[i]) > 0.50 &&
                    bounded_delta * v[i] < 0.0) {
                    bounded_delta = bbot_jump::clamp_value(
                        0.25 * v[i] * duration, -max_delta, max_delta);
                }
                q_end = q[i] + bounded_delta;
                protective_deploy_rate_limited_ =
                    protective_deploy_rate_limited_ ||
                    std::abs(q_end - end[i]) > 1e-6;
            }
            traj.init(now_sec, duration, q[i], v[i], a[i], q_end, 0.0, 0.0);
        }
    }

    // ── 阶段 3：腾空相控制 (FLIGHT) ──
    void run_state_flight(double now_sec)
    {
        double elapsed = now_sec - state_start_time_;
        double pitch_err = pitch_ - balance_offset_;
        const bool attitude_unstable = std::abs(pitch_err) > PITCH_FLIGHT_GUARD_ ||
                                       std::abs(pitch_rate_) > 3.0;

        // v5.6 全空中阶段共享同一姿态参考，髋部小辅助力矩与反作用轮不能各自
        // 追不同的 reference。离地时从真实 pitch 起步，平滑回到略前倾着陆姿态。
        const double landing_air_pitch_ref =
            balance_offset_ + flight_landing_pitch_bias_;
        const double ref_ratio = bbot_jump::clamp_value(
            elapsed / std::max(0.05, flight_pitch_transition_duration_), 0.0, 1.0);
        const double ref_smooth =
            ref_ratio * ref_ratio * (3.0 - 2.0 * ref_ratio);
        const double ref_smooth_dot =
            (ref_ratio > 0.0 && ref_ratio < 1.0) ?
            (6.0 * ref_ratio * (1.0 - ref_ratio) /
             std::max(0.05, flight_pitch_transition_duration_)) : 0.0;
        const double air_pitch_ref = bbot_jump::lerp(
            flight_pitch_ref_start_, landing_air_pitch_ref, ref_smooth);
        const double air_pitch_rate_ref = bbot_jump::clamp_value(
            (landing_air_pitch_ref - flight_pitch_ref_start_) * ref_smooth_dot,
            -flight_landing_pitch_rate_max_, flight_landing_pitch_rate_max_);
        const double air_pitch_err = pitch_ - air_pitch_ref;
        const double air_pitch_rate_err = pitch_rate_ - air_pitch_rate_ref;
        flight_air_pitch_ref_diag_ = air_pitch_ref;
        flight_air_pitch_rate_ref_diag_ = air_pitch_rate_ref;

        const bool attitude_stable =
            std::abs(air_pitch_err) <= 0.12 && std::abs(air_pitch_rate_err) <= 0.80;

        double L_target = bbot_jump::clamp_value(flight_start_z_, L_MIN_, L_TOUCH_);

        // v6.1：统一估计到 wheel-first 着陆高度的剩余弹道时间。
        // 低跳时不能继续依赖固定 T_FLIGHT_APEX_，否则腿刚缩完就已经来不及展回去。
        auto estimate_remaining_to_touchdown = [&]() -> double {
            const bool fresh_world = odom_received_ &&
                (now_sec - last_world_odom_time_ <= 0.10) &&
                std::isfinite(gazebo_world_z_) &&
                std::isfinite(gazebo_world_z_dot_);
            if (!fresh_world) return 0.30;
            const double landing_world_z = ground_height_offset_ + L_TOUCH_;
            const double dz = std::max(0.0, gazebo_world_z_ - landing_world_z);
            const double vz = gazebo_world_z_dot_;
            const double disc = std::max(0.0, vz * vz + 2.0 * 9.81 * dz);
            return std::max(0.0, (vz + std::sqrt(disc)) / 9.81);
        };

        auto begin_tuck = [&](const char * reason) {
            // v6.2：TUCK 必须从“当前真实关节状态”开始，而不是从 ATTITUDE_ARREST
            // 的预测轨迹采样点开始。上一版 round-trip 准入正是在这里把低速、可收腿
            // 的真实状态误判为不可行，导致直接跳到 PROTECTIVE_DEPLOY，肉眼完全看不到缩腿。
            const double tuck_comp = bbot_jump::clamp_value(
                pitch_ - balance_offset_, -0.24, 0.24);
            const auto tuck_ik = inverse_kinematics_with_target_x(
                L_RETRACT_, tuck_comp, 0.0);

            const std::array<double, 4> q0{
                hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_};
            const std::array<double, 4> v0{
                bbot_jump::clamp_value(hip_vel_left_,  -2.8, 2.8),
                bbot_jump::clamp_value(knee_vel_left_, -4.5, 4.5),
                bbot_jump::clamp_value(hip_vel_right_, -2.8, 2.8),
                bbot_jump::clamp_value(knee_vel_right_,-4.5, 4.5)};
            const std::array<double, 4> qf{
                tuck_ik.theta_hip, tuck_ik.theta_knee,
                tuck_ik.theta_hip, tuck_ik.theta_knee};

            for (size_t i = 0; i < 4; ++i) {
                normal_flight_joint_traj_[i].init(
                    now_sec, T_FLIGHT_TUCK_,
                    q0[i], v0[i], 0.0,
                    qf[i], 0.0, 0.0);
            }
            flight_trajectory_initialized_ = true;
            flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_TUCK;
            tuck_start_time_ = now_sec;
            tuck_start_timestamp_ = now_sec;
            tuck_start_z_ = current_z_;
            tuck_started_ = true;
            tuck_traj_.init(now_sec, T_FLIGHT_TUCK_, tuck_start_z_, 0.0, 0.0,
                            L_RETRACT_, 0.0, 0.0);
            L_target = tuck_start_z_;
            RCLCPP_INFO(this->get_logger(),
                        ">>> [FORCED_TUCK] %s：t=%.3fs remaining=%.3fs，"
                        "L %.3f -> %.3f m，T=%.3fs；q=(%.3f,%.3f)->(%.3f,%.3f) <<<",
                        reason, elapsed, estimate_remaining_to_touchdown(),
                        tuck_start_z_, L_RETRACT_, T_FLIGHT_TUCK_,
                        hip_pos_left_, knee_pos_left_, tuck_ik.theta_hip, tuck_ik.theta_knee);
        };

        auto begin_protective_deploy = [&](double start_target) {
            const double start_z = bbot_jump::clamp_value(
                start_target, L_MIN_, L_TOUCH_);
            protective_deploy_start_time_ = now_sec;
            // 锁存切换瞬间的真实关节角。仅按腿长做 IK 插值会因为当前构型
            // 与等效腿长 IK 不唯一而产生关节目标阶跃，正是本次展腿后再次
            // 注入后仰角动量的来源。
            protective_deploy_q_hip_left_ = hip_pos_left_;
            protective_deploy_q_knee_left_ = knee_pos_left_;
            protective_deploy_q_hip_right_ = hip_pos_right_;
            protective_deploy_q_knee_right_ = knee_pos_right_;
            // 落地构型只使用 capture 规划，不再锁存离地时的 pitch_err。
            latch_landing_capture_plan(now_sec);

            // v6.1：保护展腿仍按剩余弹道时间规划，但如果首段因为速度预算
            // rate_limited，没有到达完整 landing IK，后续允许继续追加一段。
            protective_deploy_replan_count_ = 0;
            const double remaining_to_touchdown = estimate_remaining_to_touchdown();
            protective_deploy_duration_active_ = bbot_jump::clamp_value(
                remaining_to_touchdown - landing_deploy_ready_margin_,
                landing_protective_deploy_min_, T_PROTECTIVE_DEPLOY_);
            const double deploy_duration = protective_deploy_duration_active_;
            plan_flight_joints(now_sec, deploy_duration, L_TOUCH_,
                               true, balance_offset_ + flight_landing_pitch_bias_, landing_target_x_);
            RCLCPP_INFO(this->get_logger(),
                        "[PROTECTIVE_DEPLOY_PLAN] remaining=%.3f s duration=%.3f s margin=%.3f s "
                        "target_x=%.3f shank_target=%.3f rate_limited=%d",
                        remaining_to_touchdown, deploy_duration, landing_deploy_ready_margin_,
                        landing_target_x_, landing_target_shank_abs_,
                        protective_deploy_rate_limited_ ? 1 : 0);
            protective_deploy_traj_.init(
                now_sec, deploy_duration, start_z, 0.0, 0.0,
                L_TOUCH_, 0.0, 0.0);
            protective_deploy_initialized_ = true;
            flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY;
            protective_landing_ = true;
            L_target = start_z;
        };

        // ── 内部子阶段状态机 (外部仍显示 FLIGHT) ──
        switch (flight_subphase_)
        {
            case bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST:
            {
                // 保持离地构型：保持离地瞬时的有效腿长目标，不随空中腿长被动拉伸而漂移
                L_target = bbot_jump::clamp_value(flight_start_z_, L_MIN_, L_TOUCH_);

                // 保护离地时不要求倾角已经回到 0；只要角速度已被刹住、
                // 倾角仍在可恢复范围，就应尽早开始低加速度展腿。否则固定
                // 等到 0.15 s 才展开，会把全部关节运动挤到下降末段。
                const bool rotation_arrested =
                    std::abs(air_pitch_err) <= 0.18 && std::abs(air_pitch_rate_err) <= 0.35;
                const bool arrest_condition = protective_landing_ ?
                    rotation_arrested : attitude_stable;
                if (arrest_condition) {
                    attitude_arrest_stable_count_++;
                } else {
                    attitude_arrest_stable_count_ = 0;
                }

                // v5.6：高速腿正是 ATTITUDE_ARREST 要处理的对象，不能再把
                // !flight_leg_motion_ready() 当成“立即展腿”的条件。否则离地膝速 -10rad/s
                // 时会直接绕过 arrest 轨迹。这里只对接近硬限位的构型立即保护；
                // 正常高速关节至少给 90ms 去沿 arrest 五次轨迹减速。
                const bool leg_position_unsafe =
                    std::abs(hip_pos_left_) > 1.40 || std::abs(hip_pos_right_) > 1.40 ||
                    std::abs(knee_pos_left_) > 1.40 || std::abs(knee_pos_right_) > 1.40;

                // v6.1：离地时如果“当前”关节速度已经低，不能再固定等65~110ms。
                // 上一轮真正离地时 hip_v≈1.1、knee_v≈-1.3，已经足够安全，却仍被
                // ATTITUDE_ARREST + protective 路径拖到整段不收腿。现在最早25ms即可进入TUCK。
                const double max_hip_speed = std::max(
                    std::abs(hip_vel_left_), std::abs(hip_vel_right_));
                const double max_knee_speed = std::max(
                    std::abs(knee_vel_left_), std::abs(knee_vel_right_));
                const bool tuck_joint_speed_safe =
                    flight_leg_motion_ready() &&
                    max_hip_speed <= 2.8 &&
                    max_knee_speed <= 4.5;
                const double remaining_time = estimate_remaining_to_touchdown();
                const double tuck_time_need =
                    T_FLIGHT_TUCK_ + T_FLIGHT_EXTEND_ + 0.025;
                const bool enough_time_for_tuck =
                    remaining_time >= tuck_time_need;
                const bool early_tuck_attitude_ok =
                    std::abs(air_pitch_err) <= 0.24 &&
                    std::abs(air_pitch_rate_err) <= 1.10;

                // v6.2：不再用 tuck_round_trip_feasible() 阻止第一次收腿。
                // 当前关节速度已经安全且还有足够时间时，直接从真实关节状态做一次可见TUCK。
                // 是否来得及完整EXTEND由TU CK阶段的剩余时间动态决定。
                const bool early_tuck_ready =
                    !protective_landing_ &&
                    !leg_position_unsafe &&
                    elapsed >= 0.015 &&
                    tuck_joint_speed_safe &&
                    early_tuck_attitude_ok &&
                    enough_time_for_tuck;

                if (early_tuck_ready) {
                    begin_tuck("当前真实关节已低速，直接执行TUCK");
                }

                if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) {
                    // protective_takeoff 只代表当前真实姿态/关节仍不安全；最多给50ms减速，
                    // 然后转入wheel-first保护轨迹，避免把所有运动拖到下降末段。
                    const bool protective_deploy_due =
                        protective_landing_ && elapsed >= 0.050;
                    const bool normal_arrest_timeout =
                        !protective_landing_ && elapsed >= 0.080;

                    if (leg_position_unsafe || protective_deploy_due ||
                        (attitude_unstable && elapsed >= 0.065) ||
                        normal_arrest_timeout) {
                        // 正常跳若已经具备TUCK速度条件但只是姿态稳定计数没到，
                        // 优先尝试TUCK；只有剩余时间不足/轨迹不可行才直接布置落地腿。
                        const bool late_tuck_possible =
                            !protective_landing_ &&
                            tuck_joint_speed_safe &&
                            enough_time_for_tuck &&
                            std::abs(air_pitch_err) <= 0.28 &&
                            std::abs(air_pitch_rate_err) <= 1.25;
                        if (late_tuck_possible) {
                            begin_tuck("ARREST超时但当前状态仍可安全收腿");
                        } else {
                            RCLCPP_WARN(this->get_logger(),
                                        "[腾空保护] arrest结束/超限 (elapsed=%.3fs, pitch=%.3f, gyro=%.3f, "
                                        "hip_v=%.2f knee_v=%.2f remaining=%.3f)，转入快速wheel-first展腿",
                                        elapsed, pitch_err, pitch_rate_,
                                        hip_vel_left_, knee_vel_left_, remaining_time);
                            begin_protective_deploy(current_height_);
                        }
                    }
                    // 若姿态已经稳定，不再固定等65ms；30ms以后即可开始TUCK。
                    else if (elapsed >= 0.030 && attitude_arrest_stable_count_ >= 2) {
                        if (protective_landing_) {
                            RCLCPP_INFO(this->get_logger(),
                                        ">>> [姿态角速度已刹住] t=%.3fs！开始保护展腿 <<<",
                                        elapsed);
                            begin_protective_deploy(current_height_);
                        } else if (enough_time_for_tuck && tuck_joint_speed_safe) {
                            begin_tuck("姿态与关节速度均已满足");
                        } else {
                            RCLCPP_INFO(this->get_logger(),
                                        "[跳过收腿] 剩余飞行时间或关节轨迹预算不足 "
                                        "(remaining=%.3f need=%.3f)，直接规划落地",
                                        remaining_time, tuck_time_need);
                            begin_protective_deploy(current_height_);
                        }
                    }
                }
                break;
            }

            case bbot_jump::FLIGHT_SUBPHASE_TUCK:
            {
                const bool tuck_finished =
                    (now_sec - tuck_start_time_) >= T_FLIGHT_TUCK_;
                const double remaining_time = estimate_remaining_to_touchdown();
                // v6.1：低跳不等固定0.22s apex。若剩余时间只够“展腿+55ms裕量”，
                // TUCK一完成就立即EXTEND，避免短腿保持过久后又来不及wheel-first。
                const bool landing_deadline_reached =
                    remaining_time <=
                    T_FLIGHT_EXTEND_ + landing_deploy_ready_margin_ + 0.020;

                if (attitude_unstable) {
                    RCLCPP_WARN(this->get_logger(),
                                "[腾空保护] 收腿期间姿态超限 (pitch=%.3f)，停止收腿转为展腿保护",
                                pitch_err);
                    begin_protective_deploy(current_height_);
                } else if (tuck_finished || landing_deadline_reached) {
                    // v6.2：低跳没有等待apex的余量。TUCK完成后立即EXTEND；
                    // 若剩余时间提前触及着陆deadline，即使TUCK尚差几毫秒也立即转展腿。
                    latch_landing_capture_plan(now_sec);
                    plan_flight_joints(now_sec, T_FLIGHT_EXTEND_, L_TOUCH_,
                                       false,
                                       balance_offset_ + flight_landing_pitch_bias_,
                                       landing_target_x_);
                    flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_EXTEND;
                    extend_start_time_ = now_sec;

                    extend_traj_.init(
                        now_sec,
                        T_FLIGHT_EXTEND_,
                        current_height_, 0.0, 0.0,
                        L_TOUCH_, 0.0, 0.0);

                    L_target = current_height_;
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [TUCK->EXTEND] t=%.3fs remaining=%.3fs tuck_done=%d deadline=%d <<<",
                                elapsed, remaining_time,
                                tuck_finished ? 1 : 0,
                                landing_deadline_reached ? 1 : 0);
                } else {
                    double des_z, des_v, des_acc;
                    tuck_traj_.evaluate(now_sec, des_z, des_v, des_acc);
                    (void)des_acc;
                    (void)des_v;
                    L_target = des_z;
                }
                break;
            }

            case bbot_jump::FLIGHT_SUBPHASE_EXTEND:
            {
                if (attitude_unstable) {
                    begin_protective_deploy(current_height_);
                } else {
                    double extend_z;
                    double extend_v;
                    double extend_acc;

                    extend_traj_.evaluate(
                        now_sec,
                        extend_z,
                        extend_v,
                        extend_acc);

                    L_target = extend_z;
                }
                break;
            }

            case bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY:
            {
                if (!protective_deploy_initialized_) {
                    begin_protective_deploy(current_height_);
                }
                double deploy_v = 0.0;
                double deploy_a = 0.0;
                protective_deploy_traj_.evaluate(
                    now_sec, L_target, deploy_v, deploy_a);

                // v6.1：v5.9/v6.0 的 rate_limited 只限制一次，然后永久保持
                // “中间长腿构型”。现在首段结束后若仍有飞行时间且端点被限幅，
                // 自动从当前轨迹末端继续追完整 landing IK，最多追加两段。
                const bool segment_finished =
                    (now_sec - protective_deploy_start_time_) >=
                    protective_deploy_duration_active_;
                if (segment_finished &&
                    protective_deploy_rate_limited_ &&
                    protective_deploy_replan_count_ < 2) {
                    const double remaining_time =
                        estimate_remaining_to_touchdown();
                    const double usable =
                        remaining_time - landing_deploy_ready_margin_;
                    if (usable >= 0.055) {
                        ++protective_deploy_replan_count_;
                        const double replan_duration =
                            bbot_jump::clamp_value(usable, 0.055, 0.12);
                        const double replan_start_z =
                            bbot_jump::clamp_value(current_z_, L_MIN_, L_TOUCH_);

                        protective_deploy_start_time_ = now_sec;
                        protective_deploy_duration_active_ = replan_duration;
                        plan_flight_joints(
                            now_sec, replan_duration, L_TOUCH_, true,
                            balance_offset_ + flight_landing_pitch_bias_,
                            landing_target_x_);
                        protective_deploy_traj_.init(
                            now_sec, replan_duration,
                            replan_start_z, 0.0, 0.0,
                            L_TOUCH_, 0.0, 0.0);
                        L_target = replan_start_z;

                        RCLCPP_INFO(this->get_logger(),
                                    "[PROTECTIVE_REPLAN] pass=%d remaining=%.3f "
                                    "duration=%.3f rate_limited=%d",
                                    protective_deploy_replan_count_,
                                    remaining_time, replan_duration,
                                    protective_deploy_rate_limited_ ? 1 : 0);
                    }
                }

                protective_landing_ = true;
                break;
            }
        }

        current_height_ = L_target;

        // 保持力矩控制模式
        const bool extend_finished =
        flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND &&
        (now_sec - extend_start_time_) >= T_FLIGHT_EXTEND_;

        const bool protective_deploy_finished =
        flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY &&
        protective_deploy_initialized_ &&
        (now_sec - protective_deploy_start_time_) >= protective_deploy_duration_active_;

        const bool legs_deployed =
        protective_deploy_finished ||
        extend_finished;
        if (!effort_mode_active_ && !leg_mode_switch_pending_) {
            request_effort_controller();
        }

        const bool landing_approach =
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND ||
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY ;

        // 所有空中子阶段均从独立关节轨迹解析获得位置和速度。
        // 左右速度不能取平均，否则不对称离地时首帧即产生额外阻尼冲击。
        std::array<double, 4> q_des{}, qdot_des{}, qddot_des{};
        sample_flight_joints(now_sec, q_des, qdot_des, qddot_des);
        flight_trajectory_initialized_ = true;

        // v5.9：落地接近时显式保证 wheel-first。
        // 不管五次轨迹的瞬态如何，都不允许期望小腿接近水平。
        bool landing_geom_clamped = false;
        if (landing_approach) {
            landing_geom_clamped |= enforce_wheel_first_target(
                q_des[0], q_des[1], qdot_des[0], qdot_des[1]);
            landing_geom_clamped |= enforce_wheel_first_target(
                q_des[2], q_des[3], qdot_des[2], qdot_des[3]);

            const double phi2_0 = std::atan2(0.28210870, 0.19553796);
            const auto & p = kinematics_.get_params();
            const double shank_l = phi2_0 + q_des[0] + q_des[1] - pitch_;
            const double shank_r = phi2_0 + q_des[2] + q_des[3] - pitch_;
            const double knee_axis_l = p.l1 * std::cos(shank_l);
            const double knee_axis_r = p.l1 * std::cos(shank_r);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 80,
                "[LANDING_GEOM] sub=%s shankL=%.3f shankR=%.3f limit=%.3f "
                "kneeAxisL=%.3f kneeAxisR=%.3f clamped=%d deploy=%.3f/%.3f",
                bbot_jump::flight_subphase_to_string(flight_subphase_),
                shank_l, shank_r, landing_shank_limit(),
                knee_axis_l, knee_axis_r, landing_geom_clamped ? 1 : 0,
                (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY) ?
                    (now_sec - protective_deploy_start_time_) : 0.0,
                protective_deploy_duration_active_);
        }

        last_thrust_force_ = 0.0;
        last_thrust_force_request_ = 0.0;
        last_thrust_force_limit_ = 0.0;
        last_tau_body_per_hip_ = 0.0;

        if (effort_mode_active_) {
            // 空中阶段：全程保持机身俯仰姿态稳定与角动量平衡
            double kp_hip_fl = 12.0;
            double kd_hip_fl = 1.5;
            double kp_knee_fl = 15.0;
            double kd_knee_fl = 1.5;
            if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) {
                // v5.7：ATTITUDE_ARREST 只做“限速”，不再用高 D 把高速腿急刹。
                // 高速时故意降低关节反馈，让主要减速工作发生在起跳前的地面阶段。
                const double knee_speed_mag = std::max(
                    std::abs(knee_vel_left_), std::abs(knee_vel_right_));
                const double fast_blend = bbot_jump::clamp_value(
                    (knee_speed_mag - 4.0) / 4.0, 0.0, 1.0);
                kp_hip_fl = bbot_jump::lerp(14.0, 8.0, fast_blend);
                kd_hip_fl = bbot_jump::lerp(2.8, 1.2, fast_blend);
                kp_knee_fl = bbot_jump::lerp(18.0, 8.0, fast_blend);
                kd_knee_fl = bbot_jump::lerp(3.2, 1.0, fast_blend);
            } else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY) {
                // 空中髋、膝只跟踪安全落地构型，机身姿态交给反作用轮。
                // 提高阻尼而不是只堆刚度，抑制上一轮中约 10 Hz 的关节摆动。
                const double gain_blend = bbot_jump::clamp_value(
                    (now_sec - protective_deploy_start_time_) / 0.10, 0.0, 1.0);
                kp_hip_fl = bbot_jump::lerp(18.0, 32.0, gain_blend);
                kd_hip_fl = bbot_jump::lerp(4.5, 7.0, gain_blend);
                kp_knee_fl = bbot_jump::lerp(22.0, 40.0, gain_blend);
                kd_knee_fl = bbot_jump::lerp(5.0, 8.0, gain_blend);
            } else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND) {
                // 展腿着陆阶段提高构型刚度，确保轮子接触前腿已充分伸展，
                // 但仍保留速度阻尼以免把伸腿冲击直接传给箱体。
                kp_hip_fl = 28.0;
                kd_hip_fl = 3.0;
                kp_knee_fl = 55.0;
                kd_knee_fl = 6.0;
            }
            // 髋力矩是机身与整条腿之间的内部力矩：用它大幅纠正机身必然
            // 反向甩动大腿，最终使膝盖先着地。姿态刹车前段仅保留小幅
            // 辅助力矩；一旦开始展腿，髋关节完全用于落地构型跟踪，机身
            // 俯仰由下方反作用轮控制。
            const double tau_limit = landing_approach ? 0.0 : 3.0;
            const double tau_hip_air = bbot_jump::clamp_value(
                -5.0 * air_pitch_rate_err - 28.0 * air_pitch_err,
                -tau_limit, tau_limit);

            publish_effort_leg_control_lr(
                q_des[0], q_des[1], q_des[2], q_des[3],
                qdot_des[0], qdot_des[1], qdot_des[2], qdot_des[3],
                tau_hip_air, 0.0,
                tau_hip_air, 0.0,
                kp_hip_fl, kd_hip_fl, kp_knee_fl, kd_knee_fl,
                tau_hip_air, tau_hip_air);
        } else {
            publish_position_leg_control_lr(
                q_des[0], q_des[1], q_des[2], q_des[3]);
        }

        // v6.4：着陆轮控和触地检测使用同一份按时间戳对齐的净空。
        const bool fresh_world = odom_received_ &&
            now_sec - last_world_odom_time_ <= 0.10;
        if (fresh_world && gazebo_world_z_dot_ < -0.20)
            last_world_descent_time_ = last_world_odom_time_;
        const bool world_descending = last_world_descent_time_ >= 0.0 &&
            now_sec - last_world_descent_time_ <= 0.060;
        double landing_geom_left=0.0, landing_geom_right=0.0;
        const bool landing_geometry_valid = aligned_takeoff_geometry(
            now_sec, landing_geom_left, landing_geom_right);
        if (landing_geometry_valid)
            wheel_clearance_ = gazebo_world_z_ - ground_height_offset_ -
                std::max(landing_geom_left, landing_geom_right);
        contact_window_ = landing_geometry_valid && world_descending && wheel_clearance_ <= 0.035;

        // 3. 空中飞轮效应姿态控制 (动量轮反作用扭矩)
        // air_pitch_ref / air_pitch_rate_ref 已在本状态函数顶部统一生成；
        // 髋部小辅助力矩和反作用轮共同跟踪它，避免 reference 打架。
        // 降低旧版过强的 D 制动；现在 D 项针对参考角速度误差，而不是强迫 gyro=0。
        constexpr double k_air_p = 0.50;
        constexpr double k_air_d = 0.45;

        const double air_p_term = k_air_p * bbot_jump::deadband(air_pitch_err, 0.02);
        const double air_d_term = k_air_d * bbot_jump::deadband(air_pitch_rate_err, 0.08);

        // ATTITUDE_ARREST 中的关节减速已知会产生负俯仰反作用。
        // 在姿态误差真正出现以前就给反作用轮一个小的前馈吸收量，
        // 避免上一轮“先被腿打成 -2 rad/s，再靠反馈追”的滞后。
        double arrest_wheel_ff = 0.0;
        if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) {
            const double knee_speed_mag = 0.5 * (
                std::abs(knee_vel_left_) + std::abs(knee_vel_right_));
            const double speed_ff = bbot_jump::clamp_value(
                (knee_speed_mag - 4.0) / 6.0, 0.0, 1.0);
            const double time_ff = 1.0 - bbot_jump::clamp_value(elapsed / 0.10, 0.0, 1.0);
            // 当前实测符号：负轮命令用于抵消负 pitch / 负 gyro。
            arrest_wheel_ff = -0.35 * speed_ff * time_ff;
        }

        air_wheel_cmd_raw_ =
            air_wheel_baseline_ + arrest_wheel_ff +
            air_wheel_sign_ * (air_p_term + air_d_term);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 80,
            "[FLIGHT_ATT] t=%.3f sub=%s pitch=%.3f/%.3f gyro=%.3f/%.3f "
            "perr=%.3f rerr=%.3f aff=%.3f wheel_raw=%.3f",
            elapsed, bbot_jump::flight_subphase_to_string(flight_subphase_),
            pitch_, air_pitch_ref, pitch_rate_, air_pitch_rate_ref,
            air_pitch_err, air_pitch_rate_err, arrest_wheel_ff, air_wheel_cmd_raw_);

        double cmd_target =
            bbot_jump::clamp_value(
                air_wheel_cmd_raw_,
                -1.40, 1.40);

        // 下降且轮底接近地面时，平滑退出反作用轮模式；到20mm内采用
        // 地面捕获方向。反作用轮的反转命令不能一直带到真实接触。
        landing_wheel_ground_blend_ = bbot_jump::landing_wheel_blend(
            landing_wheel_ground_blend_, landing_geometry_valid,
            world_descending, wheel_clearance_);
        interpolate_lqr_gain();
        const double landing_ground_cmd = landing_capture_target(bbot_jump::catch_wheel_target(
            ground_balance_angle(), ground_balance_rate(),
            predicted_landing_forward_velocity(now_sec),
            current_gain_.k_theta, current_gain_.k_theta_dot, cmd_scale_));
        cmd_target = bbot_jump::lerp(cmd_target, landing_ground_cmd, landing_wheel_ground_blend_);
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),70,
            "[LANDING_WHEEL_HANDOFF] clr=%.3f aligned=%d descending=%d blend=%.2f air=%.3f ground=%.3f target=%.3f",
            wheel_clearance_,landing_geometry_valid?1:0,world_descending?1:0,
            landing_wheel_ground_blend_,air_wheel_cmd_raw_,landing_ground_cmd,cmd_target);

        // 速度型轮控需要尽快建立轮加速度才能产生反作用力矩；0.12/周期
        // 对当前约 1.3 rad/s 的离地角速度制动偏慢。
        const double max_air_wheel_step =
            (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) ? 0.25 : 0.18;
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_air_wheel_step, max_air_wheel_step);
        publish_wheel_cmd(cmd_x, 0.0);

        const bool leg_compressed = legs_deployed && contact_window_ &&
            current_z_ < L_target - 0.010 && current_z_dot_ < -0.10;
        if (contact_window_ &&
            (std::abs(knee_effort_left_) > 15.0 || std::abs(knee_effort_right_) > 15.0)) {
            touchdown_knee_effort_count_++;
        } else {
            touchdown_knee_effort_count_ = 0;
        }
        // 膝力矩可以由控制器自身产生，必须同时观测到压缩或 IMU 冲击。
        // 冲击后世界速度可能立即回零，因此保留短暂的下降历史。
        const bool imu_impact = contact_window_ && acc_z_filt_ > 15.0;
        const bool torque_spike = touchdown_knee_effort_count_ >= 2 &&
            current_z_dot_ < -0.10 && wheel_clearance_ <= 0.010;

        log_data(cmd_x, 0.0, 0.0, 0.0);

        const bool persistent_contact = touchdown_confirmation_.update(
            takeoff_odom_stamp_,now_sec,landing_geometry_valid,wheel_clearance_,
            world_descending,acc_z_filt_);
        if (persistent_contact || leg_compressed || torque_spike || imu_impact) {
                RCLCPP_INFO(this->get_logger(), ">>> 触地检测触发 (t=%.3fs, z=%.3f)！进入缓冲阻抗控制", elapsed, current_z_);
                current_state_ = bbot_jump::STATE_TOUCHDOWN_BUFFER;
                state_start_time_ = now_sec;
                touchdown_buffer_initialized_ = false;
                touchdown_stable_count_ = 0;
                protective_landing_ = false;
                if (!touchdown_x_latched_) {
                    touchdown_x_ref_ = x_;
                    touchdown_x_latched_ = true;
                }
                landing_pitch_err_ = pitch_ - balance_offset_;
                target_x_ = touchdown_x_ref_;
                was_moving_ = false;
                air_wheel_cmd_raw_ = 0.0;
                // 保留 FLIGHT 最后一帧轮速指令，避免触地瞬间人为把姿态控制截断。
                touchdown_catch_active_ = true;
                touchdown_catch_stable_count_ = 0;
                touchdown_catch_stable_time_ = 0.0;
                touchdown_settle_start_time_ = -1.0;
                touchdown_brake_active_ = false;
                touchdown_brake_ready_count_ = 0;
                touchdown_brake_start_time_ = -1.0;
                touchdown_brake_cmd_ref_ = 0.0;
                // 锁存真实构型作为缓冲 IK 的起点。此时若直接切到 L_TOUCH 的
                // 逆解，最新日志中髋目标会从 0.66 rad 阶跃到 0.29 rad，
                // 由此产生的髋反向力矩会在真正接地后继续把机身压向后仰。
                touchdown_joint_handoff_start_time_ = now_sec;
                touchdown_hip_left_start_ = hip_pos_left_;
                touchdown_knee_left_start_ = knee_pos_left_;
                touchdown_hip_right_start_ = hip_pos_right_;
                touchdown_knee_right_start_ = knee_pos_right_;
                preload_touchdown_effort();
                request_effort_controller();
        } else if (elapsed >= T_FLIGHT_TIMEOUT_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[腾空超时] 尚未检测到触地，保持展腿等待真实接触 (z=%.3f, vz=%.2f)",
                current_z_, current_z_dot_);
        }
    }

    double landing_capture_target(double fallback) {
        if (!capture_world_valid_) return fallback;
        // Use the existing diff_drive YAML speed ceiling (5 m/s). The old
        // 0.9/1.5 m/s catch ceiling could be below the moving COM speed.
        // Capture, braking and recovery share this target and 8 m/s² slew limit.
        const double shank_rate=0.5*(hip_vel_left_+knee_vel_left_+
            hip_vel_right_+knee_vel_right_)-torso_imu_.rate();
        capture_world_target_=bbot_jump::centroidal_catch_target(
            capture_com_velocity_,centroidal_balance_.forward,centroidal_balance_.height,
            shank_rate,kinematics_.get_params().wheel_radius,5.0,
            effort_jump_preparation() ? target_speed_smoothed_ :
            (current_state_==bbot_jump::STATE_RECOVERY && recovery_ready_ ? recovery_drive_ref_ : 0.0));
        capture_world_active_=true;
        return capture_world_target_;
    }

    double ground_balance_angle() const {
        return centroidal_balance_.valid ? centroidal_balance_.angle : pitch_-balance_offset_;
    }
    double ground_balance_rate() const {
        return centroidal_balance_.valid ? centroidal_balance_.rate : pitch_rate_;
    }
    double ground_balance_height() const {
        return centroidal_balance_.valid ?
            bbot_jump::clamp_value(centroidal_balance_.height,0.15,0.55) :
            bbot_jump::clamp_value(current_z_,0.30,0.50);
    }

    // POST_BRAKE_HOLD 与 RECOVERY 共用的低速轮毂控制律。
    // 共用同一实现是状态切换连续性的硬约束，避免后续调参只改一侧。
    double compute_post_brake_hold_wheel_target(
        double & p_term, double & d_term,
        double & capture_state, double & capture_guard)
    {
        const double pitch_err = ground_balance_angle();
        const double balance_rate = ground_balance_rate();
        p_term = 0.65 * current_gain_.k_theta * pitch_err;
        d_term = 0.22 * current_gain_.k_theta_dot * balance_rate;
        const double attitude_cmd = cmd_scale_ * (p_term + d_term);

        // 当前符号链：+xdot 表示向前运动，正 cmd 会把 xdot 往负方向拉，
        // 因此 +K*xdot 是速度阻尼；任何姿态区间都不能撤掉该项。
        // 当前符号链中 +cmd 会把机身速度拉向负方向，因此正的 x_dot
        // 需要正命令制动，负的 x_dot 需要负命令制动。
        double velocity_error = bbot_jump::deadband(x_dot_, 0.03);
        double cmd_target = attitude_cmd + 0.85 * velocity_error;

        const double capture_height = ground_balance_height();
        const double capture_omega = std::sqrt(9.81 / capture_height);
        capture_state = pitch_err + balance_rate / capture_omega;

        // 落地后接受实际触地点，单一参考 touchdown_x_ref_
        const double raw_position_error = x_ - touchdown_x_ref_;
        double position_error = bbot_jump::clamp_value(
            raw_position_error, -0.75, 0.75);
        position_error = bbot_jump::deadband(position_error, 0.02);
        cmd_target += 0.35 * position_error;

        // V17/V19：微捕获必须前后对称，但不能把“当前速度+余量”作为
        // 常驻轮速目标。那会把姿态传感器的小振荡整流成持续行走；capture
        // 只用于平滑放宽对应方向的饱和上限，真正失稳仍由 CATCH 处理。
        capture_guard = 0.0;
        double max_backward_cmd = 0.75;
        double min_forward_cmd = -0.75;
        if (pitch_err < 0.0 && capture_state < -0.030) {
            capture_guard = bbot_jump::clamp_value(
                -capture_state - 0.030, 0.0, 0.20);
            max_backward_cmd = bbot_jump::clamp_value(
                0.75 + 1.75 * capture_guard, 0.75, 1.10);
        } else if (pitch_err > 0.0 && capture_state > 0.030) {
            capture_guard = bbot_jump::clamp_value(
                capture_state - 0.030, 0.0, 0.20);
            min_forward_cmd = -bbot_jump::clamp_value(
                0.75 + 1.75 * capture_guard, 0.75, 1.10);
        }
        // 超过回中范围后，只有明确进入倒下相平面才允许继续远离目标点。
        if (raw_position_error > 0.75 && capture_state < 0.12) {
            cmd_target = std::max(cmd_target, 0.0);
        } else if (raw_position_error < -0.75 && capture_state > -0.12) {
            cmd_target = std::min(cmd_target, 0.0);
        }
        return landing_capture_target(bbot_jump::clamp_value(
            cmd_target, min_forward_cmd, max_backward_cmd));
    }

    // ── 阶段 4：触地缓冲阻抗控制 (TOUCHDOWN_BUFFER) ──
    void run_state_touchdown_buffer(double now_sec, double dt)
    {
        if (!effort_mode_active_) {
            request_effort_controller();
            bbot_kinematics::IKSolution ik_hold = kinematics_.inverse_kinematics(L_TOUCH_, 0.0);
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);
            return;
        }
        if (!touchdown_buffer_initialized_) {
            state_start_time_ = now_sec;
            touchdown_buffer_initialized_ = true;
            touchdown_stable_count_ = 0;
            // 触地第一帧直接按下落速度预充阻尼支撑力；从静态重力起步会
            // 让高速下落的腿先压缩一段行程才开始承重。
            const double mass_per_leg = TOTAL_MASS_ * 0.5;
            const double preload_force = mass_per_leg * 9.81 +
                K_Z_BUFFER_ * (L_TOUCH_ - current_z_) - D_Z_BUFFER_ * current_z_dot_;
            const double min_force = 0.25 * mass_per_leg * 9.81;
            buffer_force_per_leg_ = bbot_jump::clamp_value(
                preload_force, min_force, F_Z_BUFFER_MAX_);
            touchdown_catch_active_ = true;
            touchdown_catch_stable_count_ = 0;
            touchdown_catch_stable_time_ = 0.0;
            touchdown_settle_start_time_ = -1.0;
            touchdown_brake_active_ = false;
            touchdown_brake_ready_count_ = 0;
            touchdown_brake_start_time_ = -1.0;
            touchdown_brake_cmd_ref_ = 0.0;
            if (touchdown_joint_handoff_start_time_ < 0.0) {
                touchdown_joint_handoff_start_time_ = now_sec;
                touchdown_hip_left_start_ = hip_pos_left_;
                touchdown_knee_left_start_ = knee_pos_left_;
                touchdown_hip_right_start_ = hip_pos_right_;
                touchdown_knee_right_start_ = knee_pos_right_;
            }
            // 不清零 last_wheel_cmd_x_：从空中姿态控制连续接管到触地捕获。
        }
        double elapsed = now_sec - state_start_time_;

        // 1. 任务空间阻抗计算
        // 初触地保留展腿高度来吸收冲击，然后在 0.18 s 内下沉到
        // 稳定缓冲高度。旧逻辑始终追踪 L_TOUCH_，会在首次压缩后将
        // 机体重新弹回 0.42 m，放大俯仰反向超调。
        const double settle_ratio = bbot_jump::clamp_value(elapsed / 0.18, 0.0, 1.0);
        const double smooth_settle = settle_ratio * settle_ratio * (3.0 - 2.0 * settle_ratio);
        const double buffer_height_target = bbot_jump::lerp(
            L_TOUCH_, L_BUFFER_SETTLE_, smooth_settle);
        double z_err = buffer_height_target - current_z_;
        double m_single_leg = TOTAL_MASS_ * 0.5;
        double F_z_target = K_Z_BUFFER_ * z_err - D_Z_BUFFER_ * current_z_dot_ +
                            (m_single_leg * 9.81);
        const double F_z_min = 0.25 * m_single_leg * 9.81; // 保持最小支撑力
        F_z_target = bbot_jump::clamp_value(F_z_target, F_z_min, F_Z_BUFFER_MAX_);
        // 接触后的支撑力可快速建立，后续仍保留变化率限制避免数值跳变。
        const double max_force_step = 3500.0 * dt;
        buffer_force_per_leg_ += bbot_jump::clamp_value(
            F_z_target - buffer_force_per_leg_, -max_force_step, max_force_step);
        double F_z_base = buffer_force_per_leg_;

        // 2. 实际几何雅可比力矩映射
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);
        double pitch_err = pitch_ - balance_offset_;

        // 关节构型保持与吸能阻尼：追踪预定缓冲高度轨迹，禁止随实测下沉动态塌陷
        const double ik_z_buf = bbot_jump::clamp_value(buffer_height_target, L_BUFFER_SETTLE_, L_TOUCH_);
        // 触地后的最初缓冲阶段继续保持腿在世界系近似竖直，防止一旦
        // 存在俯仰误差，IK 又把膝盖向地面方向折回去。
        const double buffer_pitch_comp = bbot_jump::clamp_value(
            pitch_err, -0.30, 0.30);
        bbot_kinematics::IKSolution ik_buf =
            kinematics_.inverse_kinematics(ik_z_buf, buffer_pitch_comp);
        // 触地瞬间保持空中末帧构型，随后再把 IK 参考平滑交接给缓冲控制。
        // 位置目标和速度目标都连续，避免髋关节为追赶新的 IK 反向猛甩。
        const double handoff_elapsed = std::max(
            0.0, now_sec - touchdown_joint_handoff_start_time_);
        const double handoff_ratio = bbot_jump::clamp_value(
            handoff_elapsed / landing_joint_handoff_duration_, 0.0, 1.0);
        const double handoff_smooth = handoff_ratio * handoff_ratio *
            (3.0 - 2.0 * handoff_ratio);
        const double handoff_smooth_dot =
            (handoff_ratio > 0.0 && handoff_ratio < 1.0) ?
            6.0 * handoff_ratio * (1.0 - handoff_ratio) /
                landing_joint_handoff_duration_ : 0.0;
        const double hip_left_cmd = bbot_jump::lerp(
            touchdown_hip_left_start_, ik_buf.theta_hip, handoff_smooth);
        const double knee_left_cmd = bbot_jump::lerp(
            touchdown_knee_left_start_, ik_buf.theta_knee, handoff_smooth);
        const double hip_right_cmd = bbot_jump::lerp(
            touchdown_hip_right_start_, ik_buf.theta_hip, handoff_smooth);
        const double knee_right_cmd = bbot_jump::lerp(
            touchdown_knee_right_start_, ik_buf.theta_knee, handoff_smooth);
        const double hip_left_vel_cmd = bbot_jump::clamp_value(
            (ik_buf.theta_hip - touchdown_hip_left_start_) * handoff_smooth_dot,
            -3.0, 3.0);
        const double knee_left_vel_cmd = bbot_jump::clamp_value(
            (ik_buf.theta_knee - touchdown_knee_left_start_) * handoff_smooth_dot,
            -5.0, 5.0);
        const double hip_right_vel_cmd = bbot_jump::clamp_value(
            (ik_buf.theta_hip - touchdown_hip_right_start_) * handoff_smooth_dot,
            -3.0, 3.0);
        const double knee_right_vel_cmd = bbot_jump::clamp_value(
            (ik_buf.theta_knee - touchdown_knee_right_start_) * handoff_smooth_dot,
            -5.0, 5.0);
        double tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                          K_BODY_D_BUFFER_ * pitch_rate_);
        tau_body_per_hip = bbot_jump::clamp_value(tau_body_per_hip,
                                                   -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        // 髋关节垂直支撑力矩：保持完整的几何雅可比力矩映射，平衡膝关节对大腿的反作用力矩
        double tau_hip_support = F_z_base * 0.5 * (jh_left + jh_right);
        const double tau_hip = tau_hip_support + tau_body_per_hip;
        const double tau_knee = F_z_base * 0.5 * (jk_left + jk_right);

        // 交接期先使用较软的构型增益，随后回到正常缓冲增益。承重前馈
        // 仍立即生效，因此减小关节阶跃不会等同于放弃支撑。
        const double hip_kp = bbot_jump::lerp(10.0, 25.0, handoff_smooth);
        const double hip_kd = bbot_jump::lerp(3.0, 3.5, handoff_smooth);
        const double knee_kp = bbot_jump::lerp(18.0, 45.0, handoff_smooth);
        const double knee_kd = bbot_jump::lerp(5.0, 6.0, handoff_smooth);
        publish_effort_leg_control_lr(
            hip_left_cmd, knee_left_cmd, hip_right_cmd, knee_right_cmd,
            hip_left_vel_cmd, knee_left_vel_cmd, hip_right_vel_cmd, knee_right_vel_cmd,
            tau_hip, tau_knee, tau_hip, tau_knee,
            hip_kp, hip_kd, knee_kp, knee_kd,
            tau_body_per_hip, tau_body_per_hip);

        // 3. 触地轮控：捕获稳定后制动，姿态再次发散时独立返回捕获；不累加轮速参考。
        interpolate_lqr_gain();
        double pos_error = bbot_jump::clamp_value(x_ - touchdown_x_ref_, -0.30, 0.30);

        // 轮轴应追赶整机重心。箱体前倾并不保证重心在轮前；最新日志
        // pitch=+0.36 时 COM 已在轮后 7.4 cm，旧轮控仍向前追导致反向倒下。
        // 髋部姿态 PD 仍用上方的箱体 pitch；轮控的零点是 COM 在轮轴正上方。
        const double balance_angle = ground_balance_angle();
        const double balance_rate = ground_balance_rate();
        const double touchdown_catch_pitch_err = balance_angle;
        const double capture_height = ground_balance_height();
        const double capture_omega = std::sqrt(9.81 / capture_height);
        const double pitch_capture_state = balance_angle + balance_rate / capture_omega;

        // 偶然回正不能说明捕获完成；车轮仍快速后退时必须继续捕获。
        // PREPARE/BRAKE 中再次向外倾倒，也必须及时返回，不能单向锁死。
        if (!touchdown_catch_active_ && bbot_jump::touchdown_capture_lost(
                balance_angle,balance_rate,pitch_capture_state)) {
            touchdown_catch_active_=true;
            touchdown_catch_stable_count_=0;
            touchdown_catch_stable_time_=0.0;
            touchdown_brake_active_=false;
            touchdown_brake_ready_count_=0;
            touchdown_brake_start_time_=-1.0;
            touchdown_brake_cmd_ref_=0.0;
            touchdown_settle_start_time_=-1.0;
            RCLCPP_WARN(this->get_logger(),
                "[RECAPTURE] 重心再次发散，返回CATCH com_lean=%.3f com_rate=%.3f capture=%.3f vx=%.3f",
                balance_angle,balance_rate,pitch_capture_state,x_dot_);
        }
        const bool catch_release_candidate = bbot_jump::touchdown_release_ready(
            balance_angle,balance_rate,pitch_capture_state,x_dot_,last_wheel_cmd_x_) &&
            std::abs(pitch_err)<0.10 && std::abs(pitch_rate_)<0.35;

        if (touchdown_catch_active_) {
            touchdown_catch_stable_count_ = catch_release_candidate ?
                std::min(touchdown_catch_stable_count_ + 1, 1000) : 0;
            touchdown_catch_stable_time_=catch_release_candidate ? touchdown_catch_stable_time_+dt : 0.0;
            if (touchdown_catch_stable_time_ >= 0.12) {
                touchdown_catch_active_ = false;
                touchdown_catch_stable_count_ = 0;
                touchdown_catch_stable_time_ = 0.0;
                touchdown_settle_start_time_ = now_sec;
                touchdown_brake_active_ = false;
                touchdown_brake_ready_count_ = 0;
                touchdown_brake_start_time_ = -1.0;
                touchdown_brake_cmd_ref_ = 0.0;
                // 单一落地点锚定，严禁在此覆盖 target_x_
                RCLCPP_INFO(this->get_logger(),
                    "[落地捕获] CATCH -> PREPARE_BRAKE (com_lean=%.3f, com_rate=%.3f, capture=%.3f, omega=%.2f, wheel_v=%.3f)",
                    balance_angle, balance_rate, pitch_capture_state, capture_omega, x_dot_);
            }
        } else if (!touchdown_brake_active_) {
            const double prepare_elapsed =
                (touchdown_settle_start_time_ >= 0.0) ?
                (now_sec - touchdown_settle_start_time_) : 0.0;

            const bool brake_ready_nominal =
                prepare_elapsed >= 0.08 &&
                pitch_capture_state > 0.08 &&
                balance_angle > 0.015 &&
                balance_rate > 0.05;

            const bool brake_ready_urgent =
                prepare_elapsed >= 0.06 &&
                pitch_capture_state > 0.12 &&
                balance_angle > 0.08 &&
                balance_rate > 0.30;

            const bool brake_ready_low_speed =
                prepare_elapsed > 0.25 &&
                std::abs(x_dot_) < 0.35 &&
                std::abs(balance_angle) < 0.06 &&
                std::abs(balance_rate) < 0.45;

            const bool brake_ready_counted =
                brake_ready_nominal || brake_ready_low_speed;
            touchdown_brake_ready_count_ = brake_ready_counted ?
                std::min(touchdown_brake_ready_count_ + 1, 1000) : 0;
            if (brake_ready_urgent || touchdown_brake_ready_count_ >= 2) {
                touchdown_brake_active_ = true;
                touchdown_brake_ready_count_ = 0;
                touchdown_brake_start_time_ = now_sec;
                touchdown_brake_cmd_ref_ = bbot_jump::clamp_value(
                    std::max(0.0, -x_dot_), 0.0, 1.2);
                RCLCPP_INFO(this->get_logger(),
                    "[落地捕获] PREPARE_BRAKE -> BRAKE (com_lean=%.3f, com_rate=%.3f, capture=%.3f, wheel_v=%.3f, brake_ref=%.3f)",
                    balance_angle, balance_rate, pitch_capture_state, x_dot_, touchdown_brake_cmd_ref_);
            }
        } else {
            // 正常制动继续收敛；真正姿态发散已由上方独立保护返回CATCH。
        }

        double cmd_target = 0.0;
        double cmd_accel_limit = 0.0;
        double wheel_pitch_err_diag = balance_angle;
        double wheel_p_term_diag = 0.0;
        double wheel_d_term_diag = 0.0;
        double phase_aux_diag = 0.0;
        double phase_guard_diag = 0.0;
        double phase_elapsed_diag = 0.0;
        const char * wheel_phase_name = "CATCH";

        if (touchdown_catch_active_) {
            wheel_phase_name = "CATCH";
            // 保留既有增益和命令变化率，仅将反馈改为重心倾角及其变化率。
            const double catch_p_term = 0.55 * current_gain_.k_theta * touchdown_catch_pitch_err;
            double catch_d_term = 0.35 * current_gain_.k_theta_dot * balance_rate;
            if (std::abs(touchdown_catch_pitch_err) > 0.08 && catch_p_term * catch_d_term < 0.0) {
                const double max_opposing_d = 0.45 * std::abs(catch_p_term);
                catch_d_term = bbot_jump::clamp_value(catch_d_term, -max_opposing_d, max_opposing_d);
            }
            wheel_pitch_err_diag = touchdown_catch_pitch_err;
            wheel_p_term_diag = catch_p_term;
            wheel_d_term_diag = catch_d_term;
            // 统一轮速控制律：包含速度死区
            // 与临近触地时完全相同的捕获律；刚接触时轮速尚未代表机身平移。
            const double contact_speed_blend = bbot_jump::clamp_value(elapsed/0.08,0.0,1.0);
            const double catch_forward_velocity = bbot_jump::lerp(
                predicted_landing_forward_velocity(now_sec),x_dot_,contact_speed_blend);
            cmd_target = bbot_jump::catch_wheel_target(
                touchdown_catch_pitch_err,balance_rate,catch_forward_velocity,
                current_gain_.k_theta,current_gain_.k_theta_dot,cmd_scale_,
                bbot_jump::touchdown_catch_limit(touchdown_catch_pitch_err,balance_rate,capture_height));
            cmd_target = landing_capture_target(cmd_target);
            cmd_accel_limit = 8.0;

        } else if (!touchdown_brake_active_) {
            wheel_phase_name = "PREPARE";
            phase_elapsed_diag =
                (touchdown_settle_start_time_ >= 0.0) ?
                (now_sec - touchdown_settle_start_time_) : 0.0;

            const double prepare_pitch_ref = 0.035;
            const double prepare_pitch_err = balance_angle - prepare_pitch_ref;

            double prepare_p_term = 0.45 * current_gain_.k_theta * prepare_pitch_err;
            double prepare_d_term = 0.20 * current_gain_.k_theta_dot * balance_rate;
            if (prepare_p_term * prepare_d_term < 0.0) {
                const double max_opposing_d = 0.55 * std::abs(prepare_p_term);
                prepare_d_term = bbot_jump::clamp_value(
                    prepare_d_term, -max_opposing_d, max_opposing_d);
            }

            wheel_pitch_err_diag = prepare_pitch_err;
            wheel_p_term_diag = prepare_p_term;
            wheel_d_term_diag = prepare_d_term;
            phase_aux_diag = prepare_pitch_ref;

            const double attitude_cmd = cmd_scale_ * (prepare_p_term + prepare_d_term);
            // 彻底删除 support_shift_cmd = backward_speed + 0.05 恶化项，采用统一阻尼控制律
            cmd_target = attitude_cmd + 1.20 * bbot_jump::deadband(x_dot_, 0.03);
            cmd_target = bbot_jump::clamp_value(cmd_target, -0.80, 0.80);
            cmd_accel_limit = 8.0;

        } else {
            wheel_phase_name = "BRAKE";
            phase_elapsed_diag =
                (touchdown_brake_start_time_ >= 0.0) ?
                (now_sec - touchdown_brake_start_time_) : 0.0;

            const double brake_ref_ramp =
                (phase_elapsed_diag < 0.25) ? 1.00 : 1.40;
            touchdown_brake_cmd_ref_ = std::max(
                0.0,
                touchdown_brake_cmd_ref_ -
                    brake_ref_ramp * std::max(dt, 0.001));

            const bool post_brake_hold =
                touchdown_brake_cmd_ref_ < 0.06 &&
                pitch_capture_state > -0.02 &&
                balance_rate > -0.20;

            if (post_brake_hold) {
                // ── Phase D: POST_BRAKE_HOLD ──
                wheel_phase_name = "HOLD";

                const double hold_pitch_err = balance_angle;
                double hold_p_term = 0.0;
                double hold_d_term = 0.0;
                double hold_capture_state = 0.0;
                double hold_capture_guard = 0.0;
                cmd_target = compute_post_brake_hold_wheel_target(
                    hold_p_term, hold_d_term,
                    hold_capture_state, hold_capture_guard);
                cmd_accel_limit = (std::abs(x_dot_) < 0.20 &&
                                   std::abs(hold_capture_state) < 0.12) ? 4.0 : 7.5;

                wheel_pitch_err_diag = hold_pitch_err;
                wheel_p_term_diag = hold_p_term;
                wheel_d_term_diag = hold_d_term;
                (void)hold_capture_state;
                phase_aux_diag = touchdown_brake_cmd_ref_;
                phase_guard_diag = hold_capture_guard;

            } else {
                // ── Phase C: BRAKE ──
                const double brake_lean = bbot_jump::clamp_value(
                    0.004 + 0.014 * touchdown_brake_cmd_ref_, 0.004, 0.026);
                const double brake_pitch_ref = brake_lean;
                const double brake_pitch_err = balance_angle - brake_pitch_ref;
                const double brake_p_term =
                    0.42 * current_gain_.k_theta * brake_pitch_err;
                const double brake_d_term =
                    0.30 * current_gain_.k_theta_dot * balance_rate;
                const double attitude_cmd =
                    cmd_scale_ * (brake_p_term + brake_d_term);

                const double xdot_ref = -touchdown_brake_cmd_ref_;
                const double velocity_error = bbot_jump::deadband(x_dot_ - xdot_ref, 0.03);

                wheel_pitch_err_diag = brake_pitch_err;
                wheel_p_term_diag = brake_p_term;
                wheel_d_term_diag = brake_d_term;
                phase_aux_diag = touchdown_brake_cmd_ref_;

                cmd_target = touchdown_brake_cmd_ref_ + attitude_cmd + 0.85 * velocity_error;
                cmd_target = bbot_jump::clamp_value(cmd_target, -0.60, 1.50);
                cmd_accel_limit = 12.0;
            }
        }

        // v6.11: PREPARE/BRAKE/HOLD must not disable the successful COM
        // feedback. Phase bookkeeping only changes readiness, not this law.
        cmd_target = landing_capture_target(cmd_target);
        if (capture_world_active_) cmd_accel_limit = 8.0;
        const double max_cmd_step = cmd_accel_limit * std::max(dt, 0.001);
        const double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

        touchdown_phase_diag_ = wheel_phase_name;
        touchdown_capture_diag_ = pitch_capture_state;
        touchdown_cmd_target_diag_ = cmd_target;
        touchdown_x_error_diag_ = pos_error;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[TOUCHDOWN_%s] body_pitch=%.3f com_err=%.3f com_rate=%.3f capture=%.3f P=%.2f D=%.2f "
            "xdot=%.3f xerr=%.3f aux=%.3f guard=%.3f phase_t=%.3f cmd_target=%.3f cmd=%.3f",
            wheel_phase_name,
            pitch_, wheel_pitch_err_diag, balance_rate, pitch_capture_state,
            wheel_p_term_diag, wheel_d_term_diag,
            x_dot_, pos_error, phase_aux_diag, phase_guard_diag,
            phase_elapsed_diag, cmd_target, cmd_x);

        log_data(cmd_x, tau_hip, tau_knee, F_z_base * 2.0);

        // 4. 缓冲沉降稳定退出判断
        // 上一版的 0.35 s buffer_timeout 只检查 current_z_dot_，会在机身仍明显后仰时
        // 强制进入 RECOVERY。日志中 elapsed=0.388 s 正是因此提前放弃 CATCH。
        // 现在必须先确认姿态与轮速处于可恢复域，才允许离开 TOUCHDOWN_BUFFER。
        // V12：RECOVERY 接管条件必须比“落地没有继续倒”严格得多。
        // V11 的普通 stable 路径没有检查 brake_ref，导致 BRAKE 参考仍约 0.2~0.3 m/s
        // 时就切换控制律。现在必须先真正完成制动，并连续静稳 0.10 s。
        const bool leg_settled = std::abs(current_z_dot_) < 0.05 &&
                                 current_z_ > (L_BUFFER_SETTLE_ - 0.040) &&
                                 current_z_ < (L_TOUCH_ + 0.030);
        const bool body_settled = std::abs(pitch_err) < 0.060 &&
                                  std::abs(pitch_rate_) < 0.20;
        const bool wheels_settled = std::abs(x_dot_) < 0.12;
        // 角度/角速度分别落在阈值内仍可能处于单调后倒轨迹；V15 切换时
        // pitch=0.010、gyro=-0.154 看似合格，但 capture≈-0.050 且
        // wheel cmd=+0.153，实际尚未静稳。
        const bool capture_settled = std::abs(pitch_capture_state) < 0.030;
        const bool wheel_command_settled = std::abs(cmd_x) < 0.10;
        const bool brake_finished = !touchdown_catch_active_ &&
                                    touchdown_brake_active_ &&
                                    touchdown_brake_cmd_ref_ < 0.06;
        const bool settled_now = elapsed >= 0.40 &&
                                 brake_finished && leg_settled &&
                                 body_settled && wheels_settled &&
                                 capture_settled && wheel_command_settled;
        touchdown_stable_count_ = settled_now ? (touchdown_stable_count_ + 1) : 0;

        // 软超时仅用于“已经基本安全但严格稳定计数迟迟未满足”的情况；
        // 不再允许仅凭腿部竖直速度结束捕获。
        // V11：禁止 1 s 时仅凭“差不多安全”就强制进入 RECOVERY。
        // V10 日志中 safe_timeout=1 发生时轮速仍约 -0.6 m/s，BRAKE 还没完成，
        // 随后控制律与腿部支撑分配同时切换，直接触发恢复阶段发散。
        // 软超时现在只允许在 BRAKE 已基本结束、姿态/轮速也真正进入稳定域时跳过计数等待。
        const bool safe_timeout_exit =
            elapsed >= 5.00 &&
            brake_finished &&
            leg_settled && body_settled && wheels_settled &&
            capture_settled && wheel_command_settled;

        if (touchdown_stable_count_ >= 20 || safe_timeout_exit) {
            RCLCPP_INFO(
                this->get_logger(),
                ">>> 落地缓冲完成 (elapsed=%.3fs, safe_timeout=%d)，进入阶段 5：恢复自平衡 (RECOVERY)... <<<",
                elapsed, safe_timeout_exit ? 1 : 0);

            current_state_ = bbot_jump::STATE_RECOVERY;
            recovery_ready_=false;
            recovery_drive_ref_=recovery_yaw_ref_=0.0;
            target_speed_const_=target_yaw_rate_=0.0;
            state_start_time_ = now_sec;
            recovery_x_ref_ = touchdown_x_ref_;
            target_x_ = touchdown_x_ref_;
            was_moving_ = false;
            recovery_stable_count_ = 0;
            post_landing_pitch_ref_ = balance_offset_;
            height_force_per_leg_ = buffer_force_per_leg_;
            height_force_initialized_ = true;
            target_height_ = L_STAND_;
            recovery_hold_height_ = L_BUFFER_SETTLE_;
            recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
            recovery_follow_height_ik_ = false;
            fail_recovery_stable_timer_ = 0.0;
            recovery_stable_timer_ = 0.0;
            position_preload_timer_ = 0.0;
            position_hold_timer_ = 0.0;
            position_return_timer_ = 0.0;
            handoff_fallback_reason_.clear();
            controller_mode_str_ = "EFFORT";
            quintic_traj_.init(now_sec, 0.80, recovery_hold_height_, 0.0, 0.0,
                               L_STAND_, 0.0, 0.0);
            recovery_descent_started_ = true;

            const auto ik_safe = kinematics_.inverse_kinematics(
                recovery_hold_height_, 0.0);
            recovery_hip_reference_ = ik_safe.theta_hip;
            RCLCPP_INFO(
                this->get_logger(),
                "[恢复连续接管] z_ref=%.3f qhip_ref=%.3f qknee_ref=%.3f "
                "support=%.1f wheel_cmd=%.3f",
                recovery_hold_height_, ik_safe.theta_hip, ik_safe.theta_knee,
                2.0 * height_force_per_leg_, last_wheel_cmd_x_);
        } else if (elapsed >= 0.80) {
            const char * touchdown_phase = touchdown_catch_active_ ? "CATCH" :
                (!touchdown_brake_active_ ? "PREPARE" :
                 (touchdown_brake_cmd_ref_ < 0.06 ? "HOLD" : "BRAKE"));
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[落地捕获进行中] phase=%s stable=%d/20 "
                                 "(z=%.3f, v=%.2f, pitch=%.2f, gyro=%.2f, "
                                 "capture=%.3f, wheel_v=%.2f, cmd=%.3f, brake_ref=%.3f)",
                                 touchdown_phase, touchdown_stable_count_,
                                 current_z_, current_z_dot_, pitch_, pitch_rate_,
                                 pitch_capture_state, x_dot_, cmd_x,
                                 touchdown_brake_cmd_ref_);
        }
    }

    // ── Position 交接异常安全回退 ──
    void trigger_handoff_fallback(const std::string & reason)
    {
        RCLCPP_ERROR(this->get_logger(), "[交接安全回退] %s", reason.c_str());
        handoff_fallback_reason_ = reason;
        jump_failure_reason_ = "Position交接回退: " + reason;
        request_effort_controller();
        height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
        height_force_initialized_ = true;
        recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_STABILIZE;
        recovery_follow_height_ik_ = false;
        recovery_hip_reference_ = kinematics_.inverse_kinematics(L_STAND_, 0.0).theta_hip;
        recovery_stable_timer_ = 0.0;
        position_handoff_suppressed_for_jump_ = true;
        controller_mode_str_ = "EFFORT";
        publish_effort_height_control(
            L_STAND_, 0.0,
            K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
            25.0, 5.0, 45.0, 5.0,
            true);
        write_jump_summary(false, jump_failure_reason_);
    }

    // ── 阶段 5：平稳沉降与平衡恢复 (RECOVERY) ──
    void run_state_recovery(double now_sec, double dt)
    {
        double elapsed = now_sec - state_start_time_;
        post_landing_pitch_ref_ = balance_offset_;
        double pitch_err = pitch_ - post_landing_pitch_ref_;
        interpolate_lqr_gain();
        const double pos_error = bbot_jump::clamp_value(x_ - touchdown_x_ref_, -0.30, 0.30);

        // Only the settled, user-operable recovery phase accepts travel.
        // A zero reference is exactly v6.11 hold, with unchanged hip/knee control.
        recovery_drive_ref_=bbot_jump::ground_drive_reference(recovery_drive_ref_,
            recovery_ready_ && capture_world_valid_ ? target_speed_const_ : 0.0,
            dt,walk_speed_,1.0/speed_ramp_time_);
        recovery_yaw_ref_=bbot_jump::ground_drive_reference(recovery_yaw_ref_,
            recovery_ready_ && capture_world_valid_ ? target_yaw_rate_ : 0.0,
            dt,turn_speed_,1.0/speed_ramp_time_);
        // 统一轮速控制律
        double recovery_p_term = 0.0;
        double recovery_d_term = 0.0;
        double recovery_capture_state = 0.0;
        double recovery_capture_guard = 0.0;
        const double cmd_target = compute_post_brake_hold_wheel_target(
            recovery_p_term, recovery_d_term,
            recovery_capture_state, recovery_capture_guard);
        const double max_cmd_accel = capture_world_active_ ? 8.0 : (std::abs(x_dot_) < 0.20 &&
                                      std::abs(recovery_capture_state) < 0.12) ? 4.0 : 7.5;
        const double max_cmd_step = max_cmd_accel * std::max(dt, 0.001);
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, recovery_yaw_ref_);

        touchdown_phase_diag_ = bbot_jump::recovery_subphase_to_string(recovery_subphase_);
        touchdown_capture_diag_ = recovery_capture_state;
        touchdown_cmd_target_diag_ = cmd_target;
        touchdown_x_error_diag_ = pos_error;

        double support_force_total = TOTAL_MASS_ * 9.81;

        switch (recovery_subphase_)
        {
            case bbot_jump::RECOVERY_FAIL_CROUCH:
            {
                controller_mode_str_ = "EFFORT_FAIL_CROUCH";
                if (!effort_mode_active_) {
                    request_effort_controller();
                }

                double des_z = failed_thrust_crouch_height_;
                double des_v = 0.0;
                double des_acc = 0.0;
                quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);
                (void)des_acc;
                current_height_ = des_z;

                // 失败缩腿必须让髋、膝一起跟随完整 IK。旧 RECOVERY 会固定 hip target，
                // 那正是“腿看起来一直顶得很长”的一个附加原因。
                recovery_follow_height_ik_ = true;
                const auto ik_fail = kinematics_.inverse_kinematics(des_z, 0.0);
                recovery_hip_reference_ = ik_fail.theta_hip;
                support_force_total = publish_effort_height_control(
                    des_z, des_v,
                    360.0, 85.0, 220.0,
                    22.0, 4.5, 42.0, 7.0,
                    true);

                const bool crouch_reached =
                    quintic_traj_.is_finished(now_sec) &&
                    current_z_ <= failed_thrust_crouch_height_ + 0.06;
                if (crouch_reached) {
                    recovery_subphase_ = bbot_jump::RECOVERY_FAIL_STABILIZE;
                    fail_recovery_stable_timer_ = 0.0;
                    recovery_hold_height_ = failed_thrust_crouch_height_;
                    current_height_ = failed_thrust_crouch_height_;
                    recovery_hip_reference_ = kinematics_.inverse_kinematics(
                        failed_thrust_crouch_height_, 0.0).theta_hip;
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [FAIL_RECOVERY] 已缩腿到 %.3fm，先在低位稳定姿态再重新站起 <<<",
                                failed_thrust_crouch_height_);
                }
                break;
            }

            case bbot_jump::RECOVERY_FAIL_STABILIZE:
            {
                controller_mode_str_ = "EFFORT_FAIL_STABILIZE";
                if (!effort_mode_active_) {
                    request_effort_controller();
                }
                current_height_ = failed_thrust_crouch_height_;
                recovery_follow_height_ik_ = true;
                recovery_hip_reference_ = kinematics_.inverse_kinematics(
                    failed_thrust_crouch_height_, 0.0).theta_hip;
                support_force_total = publish_effort_height_control(
                    failed_thrust_crouch_height_, 0.0,
                    380.0, 90.0, 220.0,
                    22.0, 4.5, 42.0, 7.0,
                    true);

                // 这里只要求“可站起”，不要求达到最终 BALANCE 的极严稳态。
                const bool fail_quiet =
                    std::abs(pitch_err) <= 0.18 &&
                    std::abs(pitch_rate_) <= 0.80 &&
                    std::abs(x_dot_) <= 0.25 &&
                    std::abs(current_z_dot_) <= 0.25;
                if (fail_quiet) {
                    fail_recovery_stable_timer_ += dt;
                } else {
                    fail_recovery_stable_timer_ = 0.0;
                }

                if (fail_recovery_stable_timer_ >= failed_thrust_settle_duration_) {
                    const double start_z = bbot_jump::clamp_value(
                        current_z_, failed_thrust_crouch_height_, L_STAND_);
                    const double start_v = bbot_jump::clamp_value(
                        current_z_dot_, -0.20, 0.20);
                    quintic_traj_.init(
                        now_sec, 0.70, start_z, start_v, 0.0,
                        L_STAND_, 0.0, 0.0);
                    recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
                    recovery_stable_timer_ = 0.0;
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [FAIL_RECOVERY] 低位姿态稳定 %.2fs，开始 0.70s 缓慢站回 %.3fm <<<",
                                failed_thrust_settle_duration_, L_STAND_);
                }
                break;
            }

            case bbot_jump::RECOVERY_EFFORT_RAISE:
            {
                controller_mode_str_ = "EFFORT";
                if (!effort_mode_active_) {
                    request_effort_controller();
                }
                double des_z = L_STAND_, des_v = 0.0, des_acc = 0.0;
                quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);
                current_height_ = des_z;
                if (recovery_follow_height_ik_) {
                    recovery_hip_reference_ = kinematics_.inverse_kinematics(
                        des_z, 0.0).theta_hip;
                }

                support_force_total = publish_effort_height_control(
                    des_z, des_v,
                    K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                    25.0, 3.5, 45.0, 6.0,
                    true);

                if (quintic_traj_.is_finished(now_sec) && std::abs(current_z_ - L_STAND_) < 0.04) {
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [RECOVERY] EFFORT_RAISE 完成，进入 EFFORT_STABILIZE 稳态检测 <<<");
                    recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_STABILIZE;
                    recovery_hip_reference_ = kinematics_.inverse_kinematics(
                        L_STAND_, 0.0).theta_hip;
                    recovery_follow_height_ik_ = false;
                    recovery_stable_timer_ = 0.0;
                }
                break;
            }

            case bbot_jump::RECOVERY_EFFORT_STABILIZE:
            {
                controller_mode_str_ = "EFFORT";
                if (!effort_mode_active_) {
                    request_effort_controller();
                }
                current_height_ = L_STAND_;
                support_force_total = publish_effort_height_control(
                    L_STAND_, 0.0,
                    K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                    25.0, 3.5, 45.0, 6.0,
                    true);

                const bool com_steady = bbot_jump::centroidal_hold_ready(
                    capture_world_valid_,ground_balance_angle(),ground_balance_rate(),
                    capture_com_velocity_);
                const bool steady = com_steady && std::abs(pitch_err) <= 0.04 &&
                                    std::abs(pitch_rate_) <= 0.15 &&
                                    std::abs(x_dot_) <= 0.08 &&
                                    std::abs(current_z_dot_) <= 0.03;
                if (steady) {
                    recovery_stable_timer_ += dt;
                } else {
                    recovery_stable_timer_ = 0.0;
                }

                if (recovery_stable_timer_ >= 0.50) {
                    // Keep the validated support and wheel law after settling.
                    // Position handoff remains debug code, never an automatic
                    // consequence of a short quiet interval (agents.md §4/14).
                    constexpr bool kHoldRecoveryAfterJump = true;
                    if (kHoldRecoveryAfterJump) {
                        if (!recovery_ready_) {
                            recovery_ready_=true;
                            RCLCPP_INFO(this->get_logger(),
                                "[READY v6.12] Effort就绪：W/S行驶，A/D转向，空格停车，停车后J再次跳跃");
                        }
                        recovery_stable_timer_ = 0.50;
                        write_jump_summary(true, "");
                        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),2000,
                            "[COM_HOLD v6.12] 持续Effort恢复: lean=%.4f vCOM=%.4f cmd=%.4f",
                            ground_balance_angle(),capture_com_velocity_,cmd_x);
                        break;
                    }
                    if (!enable_position_handoff_ || position_handoff_suppressed_for_jump_) {
                        RCLCPP_INFO(this->get_logger(),
                                    ">>> [RECOVERY] 静稳达标，未启用 Position 交接，直接切入 Effort BALANCE <<<");
                        write_jump_summary(true, "");
                        current_state_ = bbot_jump::STATE_BALANCE;
                        balance_entry_time_ = now_sec;
                        post_landing_balance_soft_start_ = true;
                        post_landing_effort_support_ = true;
                        return;
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                                    ">>> [RECOVERY] 连续静稳 0.50s，进入 POSITION_PRELOAD (0.10s 预充无偏置构型) <<<");
                        recovery_subphase_ = bbot_jump::RECOVERY_POSITION_PRELOAD;
                        position_preload_timer_ = 0.0;
                        latched_pos_hip_left_ = hip_pos_left_;
                        latched_pos_knee_left_ = knee_pos_left_;
                        latched_pos_hip_right_ = hip_pos_right_;
                        latched_pos_knee_right_ = knee_pos_right_;
                    }
                }
                break;
            }

            case bbot_jump::RECOVERY_POSITION_PRELOAD:
            {
                controller_mode_str_ = "PRELOAD";
                support_force_total = publish_effort_height_control(
                    L_STAND_, 0.0,
                    K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                    25.0, 3.5, 45.0, 6.0,
                    true);

                publish_position_leg_control_lr(
                    latched_pos_hip_left_, latched_pos_knee_left_,
                    latched_pos_hip_right_, latched_pos_knee_right_);

                position_preload_timer_ += dt;
                if (position_preload_timer_ >= 0.10) {
                    if (request_position_controller(true)) {
                        RCLCPP_INFO(this->get_logger(),
                                    ">>> [RECOVERY] 预充完成，已请求 STRICT 切换，进入 SWITCHING 等待回调 <<<");
                        recovery_subphase_ = bbot_jump::RECOVERY_SWITCHING;
                        position_switch_request_time_ = now_sec;
                    }
                }
                break;
            }

            case bbot_jump::RECOVERY_SWITCHING:
            {
                controller_mode_str_ = "SWITCHING";
                // 原子切换完成前 Effort 仍是当前控制器，持续刷新支撑命令；
                // 同时继续向 inactive Position 控制器预发无偏置目标。
                if (effort_mode_active_) {
                    support_force_total = publish_effort_height_control(
                        L_STAND_, 0.0,
                        K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                        25.0, 3.5, 45.0, 6.0,
                        true);
                }
                publish_position_leg_control_lr(
                    latched_pos_hip_left_, latched_pos_knee_left_,
                    latched_pos_hip_right_, latched_pos_knee_right_);

                if (position_switch_request_time_ > 0.0 &&
                    now_sec - position_switch_request_time_ > 1.0) {
                    trigger_handoff_fallback("Effort→Position切换回调超时");
                    return;
                }
                break;
            }

            case bbot_jump::RECOVERY_POSITION_HOLD:
            {
                controller_mode_str_ = "POSITION";
                publish_position_leg_control_lr(
                    latched_pos_hip_left_, latched_pos_knee_left_,
                    latched_pos_hip_right_, latched_pos_knee_right_);

                // 异常监测与安全回退
                const double err_hl = std::abs(latched_pos_hip_left_ - hip_pos_left_);
                const double err_kl = std::abs(latched_pos_knee_left_ - knee_pos_left_);
                const double err_hr = std::abs(latched_pos_hip_right_ - hip_pos_right_);
                const double err_kr = std::abs(latched_pos_knee_right_ - knee_pos_right_);
                const double max_joint_err = std::max({err_hl, err_kl, err_hr, err_kr});
                max_position_error_ = std::max(max_position_error_, max_joint_err);
                const double height_drop = L_STAND_ - current_z_;
                const double pitch_abs_err = std::abs(pitch_err);

                if (max_joint_err > handoff_joint_error_limit_) {
                    trigger_handoff_fallback("HOLD阶段关节跟踪误差过大: " + std::to_string(max_joint_err));
                    return;
                }
                if (height_drop > handoff_height_drop_limit_ || std::abs(current_z_dot_) > handoff_z_dot_limit_) {
                    trigger_handoff_fallback("HOLD阶段高度塌陷: drop=" + std::to_string(height_drop) + " vz=" + std::to_string(current_z_dot_));
                    return;
                }
                if (pitch_abs_err > handoff_pitch_error_limit_) {
                    trigger_handoff_fallback("HOLD阶段俯仰角超标: err=" + std::to_string(pitch_abs_err));
                    return;
                }

                position_hold_timer_ += dt;
                if (position_hold_timer_ >= 0.25) {
                    auto ik_stand = kinematics_.inverse_kinematics(L_STAND_, 0.0);
                    traj_return_hip_l_.init(now_sec, 0.80, latched_pos_hip_left_, 0.0, 0.0, ik_stand.theta_hip, 0.0, 0.0);
                    traj_return_knee_l_.init(now_sec, 0.80, latched_pos_knee_left_, 0.0, 0.0, ik_stand.theta_knee, 0.0, 0.0);
                    traj_return_hip_r_.init(now_sec, 0.80, latched_pos_hip_right_, 0.0, 0.0, ik_stand.theta_hip, 0.0, 0.0);
                    traj_return_knee_r_.init(now_sec, 0.80, latched_pos_knee_right_, 0.0, 0.0, ik_stand.theta_knee, 0.0, 0.0);
                    recovery_subphase_ = bbot_jump::RECOVERY_POSITION_RETURN;
                    position_return_timer_ = 0.0;
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [RECOVERY] POSITION_HOLD 稳定，启动 0.8s 五次归位轨迹进入 POSITION_RETURN <<<");
                }
                break;
            }

            case bbot_jump::RECOVERY_POSITION_RETURN:
            {
                controller_mode_str_ = "POSITION";
                double des_q_hl, des_v_hl, des_a_hl;
                double des_q_kl, des_v_kl, des_a_kl;
                double des_q_hr, des_v_hr, des_a_hr;
                double des_q_kr, des_v_kr, des_a_kr;
                traj_return_hip_l_.evaluate(now_sec, des_q_hl, des_v_hl, des_a_hl);
                traj_return_knee_l_.evaluate(now_sec, des_q_kl, des_v_kl, des_a_kl);
                traj_return_hip_r_.evaluate(now_sec, des_q_hr, des_v_hr, des_a_hr);
                traj_return_knee_r_.evaluate(now_sec, des_q_kr, des_v_kr, des_a_kr);

                publish_position_leg_control_lr(des_q_hl, des_q_kl, des_q_hr, des_q_kr);

                // 异常监测与安全回退
                const double err_hl = std::abs(des_q_hl - hip_pos_left_);
                const double err_kl = std::abs(des_q_kl - knee_pos_left_);
                const double err_hr = std::abs(des_q_hr - hip_pos_right_);
                const double err_kr = std::abs(des_q_kr - knee_pos_right_);
                const double max_joint_err = std::max({err_hl, err_kl, err_hr, err_kr});
                max_position_error_ = std::max(max_position_error_, max_joint_err);
                const double height_drop = L_STAND_ - current_z_;
                const double pitch_abs_err = std::abs(pitch_err);

                if (max_joint_err > handoff_joint_error_limit_) {
                    trigger_handoff_fallback("RETURN阶段关节跟踪误差过大: " + std::to_string(max_joint_err));
                    return;
                }
                if (height_drop > handoff_height_drop_limit_ || std::abs(current_z_dot_) > handoff_z_dot_limit_) {
                    trigger_handoff_fallback("RETURN阶段高度塌陷: drop=" + std::to_string(height_drop) + " vz=" + std::to_string(current_z_dot_));
                    return;
                }
                if (pitch_abs_err > handoff_pitch_error_limit_) {
                    trigger_handoff_fallback("RETURN阶段俯仰角超标: err=" + std::to_string(pitch_abs_err));
                    return;
                }

                position_return_timer_ += dt;
                if (position_return_timer_ >= 0.80 && traj_return_hip_l_.is_finished(now_sec)) {
                    touchdown_drift_x_ = x_ - touchdown_x_ref_;
                    write_jump_summary(true, "");
                    current_state_ = bbot_jump::STATE_BALANCE;
                    balance_entry_time_ = now_sec;
                    post_landing_balance_soft_start_ = true;
                    post_landing_effort_support_ = false;
                    target_height_ = L_STAND_;
                    target_x_ = x_;
                    was_moving_ = false;
                    vel_integral_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "=====================================================");
                    RCLCPP_INFO(this->get_logger(),
                                ">>> Position 交接全流程圆满完成！正式切入 Position BALANCE <<<");
                    RCLCPP_INFO(this->get_logger(), "=====================================================");
                    return;
                }
                break;
            }

            default:
                break;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[RECOVERY_%s] t=%.3f z=%.3f vz=%.3f pitch=%.3f err=%.3f gyro=%.3f "
            "xdot=%.3f xerr=%.3f cmd=%.3f mode=%s",
            bbot_jump::recovery_subphase_to_string(recovery_subphase_), elapsed,
            current_z_, current_z_dot_, pitch_, pitch_err, pitch_rate_,
            x_dot_, pos_error, cmd_x, controller_mode_str_.c_str());

        auto ik_log = kinematics_.inverse_kinematics(current_height_, 0.0);
        bbot_kinematics::JointTorques g_torques = kinematics_.compute_gravity_torques(
            0.0, ik_log.theta_hip, ik_log.theta_knee);
        log_data(cmd_x, g_torques.hip_torque * 0.5, g_torques.knee_torque * 0.5,
                 support_force_total);
    }

    // ── 起立自恢复模式 (STANDUP) ──
    void run_state_standup()
    {
        bbot_kinematics::IKSolution ik_stand =
            kinematics_.inverse_kinematics(L_MIN_, 0.0);
        if (effort_mode_active_) {
            publish_effort_leg_control(ik_stand.theta_hip, ik_stand.theta_knee, 0.0, 0.0,
                                       0.0, 0.0, 100.0, 5.0, 100.0, 5.0);
        } else {
            publish_position_leg_control(ik_stand.theta_hip, ik_stand.theta_knee);
        }

        double pitch_err = pitch_ - balance_offset_;
        if (std::abs(pitch_err) < 0.18 && std::abs(pitch_rate_) < 1.5) {
            current_state_ = bbot_jump::STATE_BALANCE;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[平衡控制器] 机身摆起成功，切入 LQR 自平衡！");
        } else {
            double standup_vel = (pitch_err < -0.15) ? 2.5 : ((pitch_err > 0.15) ? -2.5 : 0.0);
            publish_wheel_cmd(standup_vel, 0.0);
        }
    }

    // ── 辅助与发布函数 ──
    void abort_jump_to_recovery(double now_sec, const char * reason)
    {
        const bool grounded_thrust_abort =
            current_state_ == bbot_jump::STATE_THRUST && !last_wheels_airborne_;
        jump_failure_reason_ = reason ? reason : "thrust_abort";
        RCLCPP_ERROR(this->get_logger(),
                     "[跳跃保护] %s (z=%.3f m, grounded_thrust=%d)，中止推地并进入恢复",
                     reason, current_z_, grounded_thrust_abort ? 1 : 0);
        current_state_ = bbot_jump::STATE_RECOVERY;
        recovery_ready_=false;
        recovery_drive_ref_=recovery_yaw_ref_=0.0;
        state_start_time_ = now_sec;
        target_speed_const_ = 0.0;
        target_speed_smoothed_ = 0.0;
        target_yaw_rate_ = 0.0;
        recovery_x_ref_ = x_;
        touchdown_x_ref_ = x_;
        touchdown_x_latched_ = true;
        target_x_ = x_;
        was_moving_ = false;
        recovery_stable_count_ = 0;
        recovery_stable_timer_ = 0.0;
        fail_recovery_stable_timer_ = 0.0;
        height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
        height_force_initialized_ = true;
        recovery_descent_started_ = false;

        if (grounded_thrust_abort) {
            // v6.0：轮子仍接地时，长腿是最差的失败恢复构型。先缩到约0.38m，
            // 降低质心并增加膝盖弯曲，再等待姿态变缓；之后才重新站回0.50m。
            const double safe_start_height = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, 0.70);
            // 失败恢复不允许目标轨迹继续向上伸长；若当前仍在上升，
            // 从 0 期望速度开始平滑缩腿，若已经下降则保留有限负速度连续性。
            const double safe_start_v = bbot_jump::clamp_value(
                current_z_dot_, -0.45, 0.0);
            recovery_hold_height_ = failed_thrust_crouch_height_;
            current_height_ = safe_start_height;
            recovery_follow_height_ik_ = true;
            recovery_hip_reference_ = kinematics_.inverse_kinematics(
                safe_start_height, 0.0).theta_hip;
            recovery_subphase_ = bbot_jump::RECOVERY_FAIL_CROUCH;
            quintic_traj_.init(
                now_sec, failed_thrust_crouch_duration_,
                safe_start_height, safe_start_v, 0.0,
                failed_thrust_crouch_height_, 0.0, 0.0);
            target_height_ = failed_thrust_crouch_height_;
            RCLCPP_WARN(this->get_logger(),
                        "[FAIL_RECOVERY] 轮子仍接地：先 %.2fs 缩腿 %.3f -> %.3f m，禁止直接顶回 L_STAND",
                        failed_thrust_crouch_duration_, safe_start_height,
                        failed_thrust_crouch_height_);
        } else {
            // 非接地 THRUST 失败保留原恢复入口，但修正位置参考并初始化 hip target。
            const double safe_start_height = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, L_STAND_);
            recovery_hold_height_ = safe_start_height;
            current_height_ = safe_start_height;
            recovery_follow_height_ik_ = true;
            recovery_hip_reference_ = kinematics_.inverse_kinematics(
                safe_start_height, 0.0).theta_hip;
            recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
            quintic_traj_.init(now_sec, 0.45, safe_start_height, 0.0, 0.0,
                               L_STAND_, 0.0, 0.0);
            target_height_ = L_STAND_;
        }

        write_jump_summary(false, jump_failure_reason_);
        // THRUST 已处于 effort 模式；恢复环继续用 effort，避免异步控制器往返切换。
    }

    void transition_to_protective_landing(double now_sec, const char * reason)
    {
        jump_failure_reason_ = reason ? reason : "protective_landing";
        RCLCPP_WARN(this->get_logger(),
                    "[跳跃保护] %s (z=%.3f m)，保持展腿并等待触地，不提前进入恢复",
                    reason, current_z_);
        current_state_ = bbot_jump::STATE_FLIGHT;
        state_start_time_ = now_sec;
        target_speed_const_ = 0.0;
        target_speed_smoothed_ = 0.0;
        target_yaw_rate_ = 0.0;
        flight_start_z_ = L_TOUCH_;
        flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY;
        protective_deploy_initialized_ = false;
        flight_trajectory_initialized_ = false;
        protective_deploy_start_time_ = now_sec;
        protective_landing_ = true;
        touchdown_knee_effort_count_ = 0;
        // 推地阶段已经在 effort 模式。此处若切到 Position，FLIGHT 的预展腿逻辑
        // 下一帧又会切回 Effort，造成落地前的切换抖动和力矩缓存跳变。
    }

    // effort 控制器在 inactive 时仍会缓存最近命令。触地切换前先写入安全支撑，
    // 防止重新激活时短暂执行推地末端遗留的饱和力矩。
    void preload_touchdown_effort()
    {
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);
        const double mass_per_leg = TOTAL_MASS_ * 0.5;
        const double f_hold = bbot_jump::clamp_value(
            mass_per_leg * 9.81 + K_Z_BUFFER_ * (L_TOUCH_ - current_z_) -
            D_Z_BUFFER_ * current_z_dot_,
            0.25 * mass_per_leg * 9.81, F_Z_BUFFER_MAX_);
        double tau_hip = f_hold * 0.5 * (jh_left + jh_right);
        double tau_knee = f_hold * 0.5 * (jk_left + jk_right);
        const double pitch_err = pitch_ - balance_offset_;
        double tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                          K_BODY_D_BUFFER_ * pitch_rate_);
        tau_body_per_hip = bbot_jump::clamp_value(tau_body_per_hip,
                                                   -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        tau_hip += tau_body_per_hip;

        // 预加载只建立承重前馈；位置参考保持当前实测构型，避免 effort
        // 控制器接管的第一帧就追向新的 IK 目标而造成关节阶跃。
        publish_effort_leg_control_lr(
            hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_,
            0.0, 0.0, 0.0, 0.0,
            tau_hip, tau_knee, tau_hip, tau_knee,
            10.0, 3.0, 18.0, 5.0,
            tau_body_per_hip, tau_body_per_hip);
    }

    void interpolate_lqr_gain()
    {
        double ratio = bbot_jump::clamp_value((current_height_ - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        current_gain_.k_x = bbot_jump::lerp(gain_low_.k_x, gain_high_.k_x, ratio);
        current_gain_.k_x_dot = bbot_jump::lerp(gain_low_.k_x_dot, gain_high_.k_x_dot, ratio);
        current_gain_.k_theta = bbot_jump::lerp(gain_low_.k_theta, gain_high_.k_theta, ratio);
        current_gain_.k_theta_dot = bbot_jump::lerp(gain_low_.k_theta_dot, gain_high_.k_theta_dot, ratio);
    }

    // 计算单腿垂直方向几何雅可比 (Jz_hip, Jz_knee)
    void compute_leg_vertical_jacobian(double body_pitch, double hip_angle, double knee_angle,
                                       double & jz_hip, double & jz_knee) const
    {
        const auto & params = kinematics_.get_params();
        const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
        const double phi2_0 = std::atan2(0.28210870, 0.19553796);
        const double phi_thigh = phi1_0 + hip_angle - body_pitch;
        const double phi_shank = phi2_0 + hip_angle + knee_angle - body_pitch;

        jz_hip = -params.l2 * std::sin(phi_thigh) - params.l1 * std::sin(phi_shank);
        jz_knee = -params.l1 * std::sin(phi_shank);
    }

    // 任务空间高度阻抗控制 (垂直力映射 + 关节PD保持)
    double publish_effort_height_control(
        double z_des, double z_dot_des,
        double k_z, double d_z, double force_per_leg_max,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee,
        bool stabilize_body)
    {
        const double mass_per_leg = TOTAL_MASS_ * 0.5;
        double force_target = mass_per_leg * 9.81 +
                              k_z * (z_des - current_z_) +
                              d_z * (z_dot_des - current_z_dot_);
        force_target = bbot_jump::clamp_value(force_target, 0.0, force_per_leg_max);

        if (!height_force_initialized_) {
            height_force_per_leg_ = force_target;
            height_force_initialized_ = true;
        }
        // 支撑力变化率限制 (1500 N/s)
        const double max_force_step = 1500.0 * 0.005;
        height_force_per_leg_ += bbot_jump::clamp_value(
            force_target - height_force_per_leg_, -max_force_step, max_force_step);
        const double force_per_leg = height_force_per_leg_;

        // 左右腿几何雅可比独立解算
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);

        double tau_hip_l = force_per_leg * jh_left;
        double tau_knee_l = force_per_leg * jk_left;
        double tau_hip_r = force_per_leg * jh_right;
        double tau_knee_r = force_per_leg * jk_right;
        double tau_body_per_hip = 0.0;

        if (stabilize_body) {
            const bool use_landing_pitch_ref =
                (current_state_ == bbot_jump::STATE_RECOVERY) || post_landing_gyro_reduced_;
            const double pitch_ref = use_landing_pitch_ref ?
                post_landing_pitch_ref_ : balance_offset_;
            const double pitch_err = pitch_ - pitch_ref;
            // V11：TOUCHDOWN 阶段一直使用髋、膝共同承担 Jz^T F，V10 一进入
            // RECOVERY 却把髋部垂直支撑力矩瞬间清零，造成明显的负载重分配阶跃。
            // RECOVERY 保留髋部垂直支撑，只在其上叠加机身俯仰阻尼；其它调用场景
            // 仍保留原先的“膝主承重”策略，避免扩大改动范围。
            // SQUAT 需要髋、膝共同提供 J^T Fz 支撑。此前把髋部支撑清零，
            // 使下蹲后的腿只能靠膝关节和小 PD 承重，导致直接后倒。
            if (current_state_ != bbot_jump::STATE_RECOVERY &&
                current_state_ != bbot_jump::STATE_SQUAT) {
                tau_hip_l = 0.0;
                tau_hip_r = 0.0;
            }
            tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                       K_BODY_D_BUFFER_ * pitch_rate_);
            tau_body_per_hip = bbot_jump::clamp_value(
                tau_body_per_hip, -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
            tau_hip_l += tau_body_per_hip;
            tau_hip_r += tau_body_per_hip;
        }

        const auto ik = kinematics_.inverse_kinematics(z_des, 0.0);
        const double hip_position_target =
            (current_state_ == bbot_jump::STATE_RECOVERY && !recovery_follow_height_ik_) ?
            recovery_hip_reference_ : ik.theta_hip;
        publish_effort_leg_control_lr(hip_position_target, ik.theta_knee, 0.0, 0.0,
                                      tau_hip_l, tau_knee_l,
                                      tau_hip_r, tau_knee_r,
                                      kp_hip, kd_hip, kp_knee, kd_knee,
                                      tau_body_per_hip, tau_body_per_hip);
        return force_per_leg * 2.0;
    }

    // ── 实验性稳态力矩平衡控制 ──
    double publish_effort_balance_control(double z_des)
    {
        const double mass_per_leg = TOTAL_MASS_ * 0.5;
        const double force_target = mass_per_leg * 9.81;

        if (!height_force_initialized_) {
            height_force_per_leg_ = force_target;
            height_force_initialized_ = true;
        }

        // 支撑力变化率限制 (1500 N/s)
        const double max_force_step = 1500.0 * 0.005;
        height_force_per_leg_ += bbot_jump::clamp_value(
            force_target - height_force_per_leg_,
            -max_force_step,
            max_force_step);
        const double force_per_leg = height_force_per_leg_;

        // 左右腿几何雅可比独立解算
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(
            pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(
            pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);

        double tau_hip_l = force_per_leg * jh_left;
        double tau_knee_l = force_per_leg * jk_left;
        double tau_hip_r = force_per_leg * jh_right;
        double tau_knee_r = force_per_leg * jk_right;

        const auto ik = kinematics_.inverse_kinematics(z_des, 0.0);

        // 零空间构型保持 (Jz * n = 0)
        constexpr double kNullspaceKp = 10.0;
        constexpr double kNullspaceKd = 1.2;
        constexpr double kNullspaceTauMax = 10.0;
        const auto add_nullspace_shape_hold =
            [&](double jh, double jk, double qh, double qk,
                double qdh, double qdk, double vh, double vk,
                double & tauh, double & tauk) {
                const double norm = std::hypot(jh, jk);
                if (norm < 1e-5) return;
                const double nh = jk / norm;
                const double nk = -jh / norm;
                const double shape_error = nh * (qdh - qh) + nk * (qdk - qk);
                const double shape_velocity = nh * vh + nk * vk;
                double tau_shape = kNullspaceKp * shape_error -
                                   kNullspaceKd * shape_velocity;
                tau_shape = bbot_jump::clamp_value(
                    tau_shape, -kNullspaceTauMax, kNullspaceTauMax);
                tauh += nh * tau_shape;
                tauk += nk * tau_shape;
            };

        add_nullspace_shape_hold(jh_left, jk_left,
                                 hip_pos_left_, knee_pos_left_,
                                 ik.theta_hip, ik.theta_knee,
                                 hip_vel_left_, knee_vel_left_,
                                 tau_hip_l, tau_knee_l);
        add_nullspace_shape_hold(jh_right, jk_right,
                                 hip_pos_right_, knee_pos_right_,
                                 ik.theta_hip, ik.theta_knee,
                                 hip_vel_right_, knee_vel_right_,
                                 tau_hip_r, tau_knee_r);

        // 关节空间 P/D 置零；构型保持已在上方以零空间力矩完成。
        constexpr double kHipKpSteady = 0.0;
        constexpr double kKneeKpSteady = 0.0;
        constexpr double kSoftwareKdSteady = 0.0;

        publish_effort_leg_control_lr(
            ik.theta_hip, ik.theta_knee,
            0.0, 0.0,
            tau_hip_l, tau_knee_l,
            tau_hip_r, tau_knee_r,
            kHipKpSteady, kSoftwareKdSteady,
            kKneeKpSteady, kSoftwareKdSteady);

        if (num_ % 25 == 0) {
            RCLCPP_INFO(
                this->get_logger(),
                "[Effort BALANCE] F_leg=%.2fN + nullspace_shape_hold | "
                "HL=%.2f KL=%.2f HR=%.2f KR=%.2f | "
                "z_cmd=%.3f z=%.3f pitch=%.3f gyro=%.3f",
                force_per_leg,
                hip_cmd_left_, knee_cmd_left_,
                hip_cmd_right_, knee_cmd_right_,
                z_des, current_z_, pitch_, pitch_rate_);
        }

        return force_per_leg * 2.0;
    }

    void update_leg_height_by_dt(double dt)
    {
        double step = leg_transition_speed_ * dt;
        if (current_height_ > target_height_) {
            current_height_ -= step;
            if (current_height_ < target_height_) current_height_ = target_height_;
        } else if (current_height_ < target_height_) {
            current_height_ += step;
            if (current_height_ > target_height_) current_height_ = target_height_;
        }
        current_height_ = bbot_jump::clamp_value(current_height_, L_MIN_, L_MAX_);
    }

    void publish_wheel_cmd(double linear_x, double angular_z)
    {
        last_wheel_cmd_x_ = linear_x;
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = linear_x;
        cmd.twist.angular.z = angular_z;
        cmd_pub_->publish(cmd);
    }

    // ── 位置阶段：只向位置控制器发布构型命令 ──
    void publish_position_leg_control(double q_hip_des, double q_knee_des)
    {
        publish_position_leg_control_lr(q_hip_des, q_knee_des, q_hip_des, q_knee_des);
    }

    void publish_position_leg_control_lr(double q_hip_left, double q_knee_left,
                                         double q_hip_right, double q_knee_right)
    {
        hip_pos_cmd_left_ = q_hip_left;
        knee_pos_cmd_left_ = q_knee_left;
        hip_pos_cmd_right_ = q_hip_right;
        knee_pos_cmd_right_ = q_knee_right;

        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {q_hip_left, q_knee_left, q_hip_right, q_knee_right};
        leg_pos_pub_->publish(leg_cmd);
    }

    bbot_jump::JointEffortLimits current_effort_limits() const
    {
        return bbot_jump::jump_effort_limits(current_state_==bbot_jump::STATE_THRUST,
            this->get_parameter("use_sim_time").as_bool(),sim_relax_thrust_limits_);
    }

    // ── 力矩阶段：支持左右腿独立目标构型与前馈力矩 + 关节保持 PD ──
    void publish_effort_leg_control_lr(
        double q_hip_des_l, double q_knee_des_l,
        double q_hip_des_r, double q_knee_des_r,
        double q_dot_hip_des_l, double q_dot_knee_des_l,
        double q_dot_hip_des_r, double q_dot_knee_des_r,
        double tau_ff_hip_l, double tau_ff_knee_l,
        double tau_ff_hip_r, double tau_ff_knee_r,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee,
        double tau_att_hip_l = 0.0, double tau_att_hip_r = 0.0)
    {
        hip_pos_cmd_left_ = q_hip_des_l;
        knee_pos_cmd_left_ = q_knee_des_l;
        hip_pos_cmd_right_ = q_hip_des_r;
        knee_pos_cmd_right_ = q_knee_des_r;

        joint_velocity_cmd_ = {q_dot_hip_des_l, q_dot_knee_des_l,
                               q_dot_hip_des_r, q_dot_knee_des_r};

        // 预算和最终下发必须使用同一上限，避免推地预算放宽后又被这里截断。
        // 仿真试验仅在 THRUST 使用URDF已有150Nm；其余阶段仍为75/60Nm。
        active_effort_limits_ = current_effort_limits();
        const double kHipTauLimit = active_effort_limits_.hip;
        const double kKneeTauLimit = active_effort_limits_.knee;

        auto compute_hip = [&](double tau_ff, double tau_att, double q_des, double q_dot_des, double q_act, double q_vel) {
            double att_clamped = bbot_jump::clamp_value(tau_att, -kHipTauLimit, kHipTauLimit);
            double rem_pos = kHipTauLimit - att_clamped;
            double rem_neg = -kHipTauLimit - att_clamped;
            double other = (tau_ff - tau_att) + kp_hip * (q_des - q_act) + kd_hip * (q_dot_des - q_vel);
            // tau_att 只拥有饱和余量优先级，不强制最终力矩与其同号。
            // 空中若强制同号，髋关节姿态项会压过关节阻尼，把整条腿甩向
            // 极端构型。展腿阶段 tau_att 已为零，由关节 PD 独立控制腿姿态。
            return att_clamped + bbot_jump::clamp_value(other, rem_neg, rem_pos);
        };

        double tau_hip_left = compute_hip(tau_ff_hip_l, tau_att_hip_l, q_hip_des_l, q_dot_hip_des_l, hip_pos_left_, hip_vel_left_);
        double tau_hip_right = compute_hip(tau_ff_hip_r, tau_att_hip_r, q_hip_des_r, q_dot_hip_des_r, hip_pos_right_, hip_vel_right_);

        double tau_knee_left = bbot_jump::clamp_value(
            tau_ff_knee_l + kp_knee * (q_knee_des_l - knee_pos_left_) + kd_knee * (q_dot_knee_des_l - knee_vel_left_),
            -kKneeTauLimit, kKneeTauLimit);
        double tau_knee_right = bbot_jump::clamp_value(
            tau_ff_knee_r + kp_knee * (q_knee_des_r - knee_pos_right_) + kd_knee * (q_dot_knee_des_r - knee_vel_right_),
            -kKneeTauLimit, kKneeTauLimit);

        if (current_state_ == bbot_jump::STATE_FLIGHT) {
            const bbot_jump::JointVector q(hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_);
            const bbot_jump::JointVector v(hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_);
            const bbot_jump::JointVector qd(q_hip_des_l, q_knee_des_l, q_hip_des_r, q_knee_des_r);
            const bbot_jump::JointVector vd(q_dot_hip_des_l, q_dot_knee_des_l, q_dot_hip_des_r, q_dot_knee_des_r);
            const bbot_jump::JointVector kp(kp_hip, kp_knee, kp_hip, kp_knee);
            const bbot_jump::JointVector kd(kd_hip, kd_knee, kd_hip, kd_knee);
            const bbot_jump::JointVector ff(tau_ff_hip_l, tau_ff_knee_l, tau_ff_hip_r, tau_ff_knee_r);
            const double sample_age = std::max(0.0, this->now().seconds() - joint_sample_time_);
            air_pd_horizon_ = bbot_jump::clamp_value(
                2.0 * joint_sample_period_ + sample_age, 0.025, 0.060);
            const auto mass = bbot_jump::flight_joint_inertia(q, body_mass_);
            const auto raw = (ff + kp.cwiseProduct(qd - q) + kd.cwiseProduct(vd - v)).eval();
            const auto tau = bbot_jump::discrete_flight_pd(mass, q, v, qd, vd, kp, kd, ff, air_pd_horizon_);
            for (int i = 0; i < 4; ++i) {
                air_pd_raw_[i] = raw[i];
                air_pd_discrete_[i] = tau[i];
            }
            tau_hip_left = tau[0]; tau_knee_left = tau[1];
            tau_hip_right = tau[2]; tau_knee_right = tau[3];
        }

        // v6.9：箱体姿态与腿部任务分配不同自由度。承重前馈只限速，
        // 关节阻尼保持离散求解；髋共同力矩不再进入自由腿惯量求解后被抵消。
        const bool ground_feedback =
            effort_jump_preparation() ||
            current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER ||
            current_state_ == bbot_jump::STATE_RECOVERY ||
            (current_state_ == bbot_jump::STATE_BALANCE && post_landing_effort_support_);
        if (ground_feedback) {
            const bbot_jump::JointVector q(hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_);
            const bbot_jump::JointVector v(hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_);
            const bbot_jump::JointVector qd(q_hip_des_l,q_knee_des_l,q_hip_des_r,q_knee_des_r);
            const bbot_jump::JointVector vd(q_dot_hip_des_l,q_dot_knee_des_l,q_dot_hip_des_r,q_dot_knee_des_r);
            const bbot_jump::JointVector kp(kp_hip,kp_knee,kp_hip,kp_knee);
            const bbot_jump::JointVector kd(kd_hip,kd_knee,kd_hip,kd_knee);
            const auto gravity = bbot_jump::leg_gravity_torques(
                {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_},pitch_,body_mass_);
            // 支撑前馈必须包含腿/轮自身重力。只用 J^T F 会漏掉约数Nm的
            // 髋轴力矩，让已经离散衰减的箱体纠姿长期承担这一静态偏差。
            const bbot_jump::JointVector support = gravity + bbot_jump::JointVector(
                tau_ff_hip_l-tau_att_hip_l,tau_ff_knee_l,
                tau_ff_hip_r-tau_att_hip_r,tau_ff_knee_r);
            for (int i=0;i<4;++i) ground_gravity_torque_[i]=gravity[i];
            const double sample_age = std::max(0.0,this->now().seconds()-joint_sample_time_);
            ground_pd_horizon_ = bbot_jump::clamp_value(
                2.0*joint_sample_period_+sample_age,0.025,0.060);
            const auto mass = bbot_jump::flight_joint_inertia(q,body_mass_);
            const auto feedback = bbot_jump::discrete_ground_leg_feedback(
                mass,q,v,qd,vd,kp,kd,ground_pd_horizon_);
            if (!ground_support_initialized_) {
                // 仅从上一前馈接入承重；不能把上一帧关节反馈当成前馈继续保持。
                ground_support_previous_ = last_support_feedforward_;
                ground_support_initialized_ = true;
            }
            const double effort_dt = bbot_jump::clamp_value(
                this->now().seconds()-last_effort_time_,0.0,0.020);
            const double step = (current_state_==bbot_jump::STATE_RECOVERY?800.0:3000.0)*effort_dt;
            ground_support_previous_ += (support-ground_support_previous_).cwiseMax(-step).cwiseMin(step);
            last_support_feedforward_ = ground_support_previous_;
            const auto leg_torque = (ground_support_previous_+feedback).eval();
            const double now_sec=this->now().seconds();
            torso_imu_fresh_=torso_imu_.fresh(now_sec);
            torso_control_rate_=torso_imu_fresh_ ? torso_imu_.rate() : pitch_rate_;
            const double imu_age=torso_imu_fresh_ ? std::max(0.0,now_sec-torso_imu_.stamp()) : sample_age;
            // 加上专用IMU滤波的10ms时间常数，不把200Hz定时器当传感器频率。
            torso_control_horizon_=bbot_jump::clamp_value(
                2.0*std::max(joint_sample_period_,torso_imu_.period())+
                std::max(sample_age,imu_age)+0.010,0.030,0.080);
            // 过期或无效加速度只退回静态重力补偿，不能继续用旧接触冲击。
            const double fy=torso_imu_fresh_ ? torso_imu_.fy() : -9.81*std::sin(pitch_);
            const double fz=torso_imu_fresh_ ? torso_imu_.fz() : 9.81*std::cos(pitch_);
            torso_torque_=bbot_jump::torso_pitch_torque(
                pitch_,effort_jump_preparation()?active_jump_pitch_ref_:balance_offset_,
                torso_control_rate_-(effort_jump_preparation()?active_jump_pitch_rate_ref_:0.0),
                fy,fz,body_mass_,
                K_BODY_P_BUFFER_,K_BODY_D_BUFFER_,torso_control_horizon_,TAU_HIP_BODY_MAX_);
            last_tau_body_per_hip_=torso_torque_.command;
            hip_common_before_allocation_=0.5*(leg_torque[0]+leg_torque[2]);
            const auto tau=bbot_jump::allocate_torso_hips(
                leg_torque,torso_torque_.command,kHipTauLimit);
            hip_differential_torque_=0.5*(tau[0]-tau[2]);
            for (int i=0;i<4;++i) ground_pd_feedback_[i]=feedback[i];
            tau_hip_left=tau[0]; tau_knee_left=tau[1];
            tau_hip_right=tau[2]; tau_knee_right=tau[3];
        } else {
            last_support_feedforward_ = bbot_jump::JointVector(
                tau_ff_hip_l-tau_att_hip_l,tau_ff_knee_l,tau_ff_hip_r-tau_att_hip_r,tau_ff_knee_r);
            ground_support_initialized_=false;
            ground_pd_horizon_=0.0;
            ground_pd_feedback_.fill(0.0);
            ground_gravity_torque_.fill(0.0);
            torso_torque_={};
            torso_imu_fresh_=false;
            hip_common_before_allocation_=0.0;
            hip_differential_torque_=0.0;
        }

        tau_hip_left = bbot_jump::clamp_value(tau_hip_left, -kHipTauLimit, kHipTauLimit);
        tau_knee_left = bbot_jump::clamp_value(tau_knee_left, -kKneeTauLimit, kKneeTauLimit);
        tau_hip_right = bbot_jump::clamp_value(tau_hip_right, -kHipTauLimit, kHipTauLimit);
        tau_knee_right = bbot_jump::clamp_value(tau_knee_right, -kKneeTauLimit, kKneeTauLimit);

        actual_tau_hip_left_ = tau_hip_left;
        actual_tau_knee_left_ = tau_knee_left;
        actual_tau_hip_right_ = tau_hip_right;
        actual_tau_knee_right_ = tau_knee_right;

        last_effort_tau_hip_left_ = tau_hip_left;
        last_effort_tau_knee_left_ = tau_knee_left;
        last_effort_tau_hip_right_ = tau_hip_right;
        last_effort_tau_knee_right_ = tau_knee_right;
        last_effort_time_ = this->now().seconds();
        effort_slew_initialized_ = true;

        hip_cmd_left_ = tau_hip_left;
        knee_cmd_left_ = tau_knee_left;
        hip_cmd_right_ = tau_hip_right;
        knee_cmd_right_ = tau_knee_right;

        // 下发力矩命令；位置控制器在该阶段处于 inactive，绝不同时下发位置命令。
        std_msgs::msg::Float64MultiArray effort_cmd;
        effort_cmd.data = {tau_hip_left, tau_knee_left, tau_hip_right, tau_knee_right};
        leg_effort_pub_->publish(effort_cmd);
    }

    void publish_effort_leg_control_lr(
        double q_hip_des, double q_knee_des,
        double q_dot_hip_des, double q_dot_knee_des,
        double tau_ff_hip_l, double tau_ff_knee_l,
        double tau_ff_hip_r, double tau_ff_knee_r,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee,
        double tau_att_hip_l = 0.0, double tau_att_hip_r = 0.0)
    {
        publish_effort_leg_control_lr(
            q_hip_des, q_knee_des, q_hip_des, q_knee_des,
            q_dot_hip_des, q_dot_knee_des, q_dot_hip_des, q_dot_knee_des,
            tau_ff_hip_l, tau_ff_knee_l, tau_ff_hip_r, tau_ff_knee_r,
            kp_hip, kd_hip, kp_knee, kd_knee,
            tau_att_hip_l, tau_att_hip_r);
    }

    void publish_effort_leg_control(
        double q_hip_des, double q_knee_des,
        double q_dot_hip_des, double q_dot_knee_des,
        double tau_ff_hip, double tau_ff_knee,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee,
        double tau_att_hip = 0.0)
    {
        publish_effort_leg_control_lr(
            q_hip_des, q_knee_des, q_dot_hip_des, q_dot_knee_des,
            tau_ff_hip, tau_ff_knee, tau_ff_hip, tau_ff_knee,
            kp_hip, kd_hip, kp_knee, kd_knee,
            tau_att_hip, tau_att_hip);
    }

    // ── 运行时控制器动态切换 ──
    void request_effort_controller()
    {
        if (effort_mode_active_ || leg_mode_switch_pending_) return;
        if (!switch_ctrl_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[控制器切换] switch_controller 服务尚未就绪");
            return;
        }
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = {"leg_effort_controller"};
        request->deactivate_controllers = {"leg_position_controller"};
        // 任一控制器无法切换就保留原 position 控制，绝不能出现“已停位置、未起 effort”。
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        request->activate_asap = true;
        leg_mode_switch_pending_ = true;
        switch_ctrl_client_->async_send_request(request,
            [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                auto result = future.get();
                leg_mode_switch_pending_ = false;
                if (result->ok) {
                    effort_mode_active_ = true;
                    RCLCPP_INFO(this->get_logger(), "[控制器切换] >>> Position → Effort 切换成功！全力爆发模式已激活 <<<");
                } else {
                    RCLCPP_WARN(this->get_logger(), "[控制器切换] Position → Effort 切换失败！");
                }
            });
        RCLCPP_INFO(this->get_logger(), "[控制器切换] 请求 Position → Effort ...");
    }

    // 请求切换为位置控制器 (调试/兼容)
    bool request_position_controller(bool preload_current_pose = false)
    {
        if (!effort_mode_active_ || leg_mode_switch_pending_) return false;
        if (!switch_ctrl_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[控制器切换] switch_controller 服务尚未就绪");
            return false;
        }
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        if (preload_current_pose) {
            // 真实构型无偏置预充，确保位置模式启动时期望与当前状态完全一致
            const double hip_cmd_left = hip_pos_left_;
            const double knee_cmd_left = knee_pos_left_;
            const double hip_cmd_right = hip_pos_right_;
            const double knee_cmd_right = knee_pos_right_;

            publish_position_leg_control_lr(
                hip_cmd_left,
                knee_cmd_left,
                hip_cmd_right,
                knee_cmd_right);

            post_landing_position_hip_cmd_left_ = hip_cmd_left;
            post_landing_position_knee_cmd_left_ = knee_cmd_left;
            post_landing_position_hip_cmd_right_ = hip_cmd_right;
            post_landing_position_knee_cmd_right_ = knee_cmd_right;
        } else {
            bbot_kinematics::IKSolution ik_stand = kinematics_.inverse_kinematics(current_height_, 0.0);
            publish_position_leg_control(ik_stand.theta_hip, ik_stand.theta_knee);
        }

        request->activate_controllers = {"leg_position_controller"};
        request->deactivate_controllers = {"leg_effort_controller"};
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        request->activate_asap = true;
        leg_mode_switch_pending_ = true;
        switch_ctrl_client_->async_send_request(request,
            [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                auto result = future.get();
                leg_mode_switch_pending_ = false;
                if (result->ok) {
                    effort_mode_active_ = false;
                    position_switch_time_ = this->now().seconds();

                    // 激活成功后重发位置指令
                    publish_position_leg_control_lr(
                        hip_pos_cmd_left_, knee_pos_cmd_left_,
                        hip_pos_cmd_right_, knee_pos_cmd_right_);

                    post_landing_effort_support_ = false;

                    hip_cmd_left_ = knee_cmd_left_ =
                        hip_cmd_right_ = knee_cmd_right_ = 0.0;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "[Position已激活] "
                        "HL cmd=%.4f act=%.4f | KL cmd=%.4f act=%.4f | "
                        "HR cmd=%.4f act=%.4f | KR cmd=%.4f act=%.4f",
                        hip_pos_cmd_left_, hip_pos_left_,
                        knee_pos_cmd_left_, knee_pos_left_,
                        hip_pos_cmd_right_, hip_pos_right_,
                        knee_pos_cmd_right_, knee_pos_right_);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[控制器切换] >>> Effort → Position 切换成功！已立即重发 Position 目标 <<<");

                    if (current_state_ == bbot_jump::STATE_RECOVERY &&
                        recovery_subphase_ == bbot_jump::RECOVERY_SWITCHING) {
                        recovery_subphase_ = bbot_jump::RECOVERY_POSITION_HOLD;
                        position_hold_timer_ = 0.0;
                        position_switch_request_time_ = -1.0;
                    } else {
                        // 超时回退后才到达的成功回调：不能让 Position 留在激活态。
                        RCLCPP_WARN(this->get_logger(),
                                    "[控制器切换] 收到过期的 Position 成功回调，立即恢复 Effort");
                        request_effort_controller();
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "[控制器切换] Effort → Position 切换失败！");
                    if (current_state_ == bbot_jump::STATE_RECOVERY &&
                        recovery_subphase_ == bbot_jump::RECOVERY_SWITCHING) {
                        trigger_handoff_fallback("controller_manager拒绝Effort→Position切换");
                    }
                }
            });
        RCLCPP_INFO(this->get_logger(), "[控制器切换] 请求 Effort → Position ...");
        return true;
    }

    void open_log_files()
    {
        jump_log_file_.open(data_path_ + "jump_velocity_control_log.csv");
        jump_summary_file_.open(data_path_ + "jump_velocity_summary.csv");
        if (jump_summary_file_.is_open()) {
            jump_summary_file_ << "jump_id,jump_height_target,target_takeoff_velocity,"
                               << "takeoff_velocity,apex_height_delta,apex_world_z_delta,max_abs_pitch,"
                               << "takeoff_pitch_rate,landing_pitch_err,touchdown_drift_x,"
                               << "max_abs_hip_torque,max_abs_knee_torque,mechanical_work,"
                               << "recovery_completed,failed,reason\n";
        }
        if (jump_log_file_.is_open()) {
            jump_log_file_ << "timestamp,state,state_name,z,z_dot,gazebo_world_z,gazebo_world_z_dot,pitch,pitch_rate,acc_z,"
                           << "cmd_x,x,x_dot,touchdown_x_ref,hip_pos_left,knee_pos_left,hip_pos_right,knee_pos_right,"
                           << "hip_effort_left,knee_effort_left,hip_cmd_left,knee_cmd_left,"
                           << "hip_cmd_right,knee_cmd_right,"
                           << "actual_tau_hip_left,actual_tau_knee_left,actual_tau_hip_right,actual_tau_knee_right,"
                           << "hip_pos_cmd_left,knee_pos_cmd_left,hip_pos_cmd_right,knee_pos_cmd_right,"
                           << "tau_ff_hip,tau_ff_knee,F_z,F_z_request,F_z_limit,velocity_reached,wheels_airborne,airborne_confidence,target_takeoff_velocity,tau_body_hip,thrust_reaction_ff,"
                           << "flight_subphase,recovery_subphase,controller_mode,thrust_motion_elapsed,thrust_attitude_blocked,air_wheel_cmd_raw,left_wheel_vel,right_wheel_vel,attitude_arrest_stable_count,"
                           << "touchdown_phase,brake_ref,capture_state,target_x,x_error,cmd_target,hip_vel_left,knee_vel_left,hip_vel_right,knee_vel_right,hip_vel_cmd_left,knee_vel_cmd_left,hip_vel_cmd_right,knee_vel_cmd_right,wheel_clearance,contact_window,thrust_extension_scale,air_pd_horizon,air_pd_raw_hl,air_pd_raw_kl,air_pd_raw_hr,air_pd_raw_kr,air_pd_discrete_hl,air_pd_discrete_kl,air_pd_discrete_hr,air_pd_discrete_kr,jump_forward_speed,jump_takeoff_forward_speed,jump_pitch_ref,active_jump_pitch_ref,jump_takeoff_pitch_rate,active_jump_pitch_rate_ref,pre_jump_stable_timer,takeoff_forward_speed,air_wheel_baseline,gazebo_world_x_dot,gazebo_world_y_dot,landing_capture_vx,landing_capture_raw_offset,landing_capture_offset,landing_target_x,landing_capture_comp,landing_capture_omega,landing_forward_axis_x,landing_forward_axis_y,landing_capture_planned,flight_air_pitch_ref,flight_air_pitch_rate_ref,protective_deploy_duration,protective_deploy_rate_limited,takeoff_aligned,takeoff_sample_stamp,takeoff_clearance_left,takeoff_clearance_right,landing_wheel_ground_blend,ground_pd_horizon,ground_pd_hl,ground_pd_kl,ground_pd_hr,ground_pd_kr,world_velocity_valid,odom_twist_z,thrust_release_active,thrust_release_blend,catch_stable_time,com_world_z,com_world_vz,com_velocity_valid,com_sample_stamp,com_forward_from_axle,com_height_above_axle,com_lean,com_lean_rate,com_balance_valid,thrust_feedback_vz,ground_gravity_hl,ground_gravity_kl,ground_gravity_hr,ground_gravity_kr,hip_effort_limit,knee_effort_limit,thrust_force_before_budget,thrust_knee_pd_left,torso_force_ff,torso_feedback,torso_hip_command,torso_rate,torso_horizon,torso_imu_fresh,torso_imu_fy,torso_imu_fz,hip_common_before_allocation,hip_differential_torque,capture_com_velocity,capture_world_valid,capture_world_active,capture_world_target,recovery_ready,recovery_drive_ref,recovery_yaw_ref,effort_jump_cycle,thrust_knee_position_yield\n";
        }
    }

    void close_log_files()
    {
        if (jump_log_file_.is_open()) {
            jump_log_file_.close();
        }
        if (jump_summary_file_.is_open()) {
            jump_summary_file_.close();
        }
    }

    void write_jump_summary(bool recovery_completed, const std::string & reason)
    {
        if (jump_summary_written_ || !jump_summary_file_.is_open()) return;
        const bool world_height_valid = world_height_valid_for_jump_ && odom_received_;
        const double apex_world_z_delta = world_height_valid ?
            (max_world_z_during_jump_ - thrust_start_world_z_) : -1.0;
        std::string effective_reason = reason;
        if (!world_height_valid) {
            if (!effective_reason.empty()) effective_reason += "; ";
            effective_reason += "Gazebo世界高度里程计不可用";
        }
        const bool effective_recovery_completed = recovery_completed && world_height_valid;
        jump_summary_file_ << jump_id_ << ","
                           << jump_height_target_ << ","
                           << target_takeoff_velocity_ << ","
                           << actual_takeoff_velocity_ << ","
                           << (max_z_during_jump_ - thrust_start_z_) << ","
                           << apex_world_z_delta << ","
                           << max_abs_pitch_during_jump_ << ","
                           << takeoff_pitch_rate_ << ","
                           << landing_pitch_err_ << ","
                           << (x_ - touchdown_x_ref_) << ","
                           << max_abs_hip_torque_during_jump_ << ","
                           << max_abs_knee_torque_during_jump_ << ","
                           << thrust_mechanical_work_ << ","
                           << (effective_recovery_completed ? 1 : 0) << ","
                           << (!effective_reason.empty() ? 1 : 0) << ","
                           << effective_reason << "\n";
        jump_summary_file_.flush();
        jump_summary_written_ = true;
    }

    void log_data(double cmd_x, double tau_hip, double tau_knee, double f_z)
    {
        double t = (this->now() - start_time_).seconds();
        if (current_state_ != bbot_jump::STATE_BALANCE) {
            max_z_during_jump_ = std::max(max_z_during_jump_, current_z_);
            max_abs_pitch_during_jump_ = std::max(max_abs_pitch_during_jump_, std::abs(pitch_));
            const double commanded_hip = std::max({std::abs(hip_cmd_left_), std::abs(hip_cmd_right_),
                                                   std::abs(tau_hip)});
            const double commanded_knee = std::max({std::abs(knee_cmd_left_), std::abs(knee_cmd_right_),
                                                    std::abs(tau_knee)});
            max_abs_hip_torque_during_jump_ = std::max(max_abs_hip_torque_during_jump_, commanded_hip);
            max_abs_knee_torque_during_jump_ = std::max(max_abs_knee_torque_during_jump_, commanded_knee);
        }
        if (jump_log_file_.is_open()) {
            jump_log_file_ << t << ","
                           << static_cast<int>(current_state_) << ","
                           << bbot_jump::state_to_string(current_state_) << ","
                           << current_z_ << ","
                           << current_z_dot_ << ","
                           << gazebo_world_z_ << ","
                           << gazebo_world_z_dot_ << ","
                           << pitch_ << ","
                           << pitch_rate_ << ","
                           << acc_z_filt_ << ","
                           << cmd_x << ","
                           << x_ << ","
                           << x_dot_ << ","
                           << touchdown_x_ref_ << ","
                           << hip_pos_left_ << ","
                           << knee_pos_left_ << ","
                           << hip_pos_right_ << ","
                           << knee_pos_right_ << ","
                           << hip_effort_left_ << ","
                           << knee_effort_left_ << ","
                           << hip_cmd_left_ << ","
                           << knee_cmd_left_ << ","
                           << hip_cmd_right_ << ","
                           << knee_cmd_right_ << ","
                           << actual_tau_hip_left_ << ","
                           << actual_tau_knee_left_ << ","
                           << actual_tau_hip_right_ << ","
                           << actual_tau_knee_right_ << ","
                           << hip_pos_cmd_left_ << ","
                           << knee_pos_cmd_left_ << ","
                           << hip_pos_cmd_right_ << ","
                           << knee_pos_cmd_right_ << ","
                           << tau_hip << ","
                           << tau_knee << ","
                           << f_z << ","
                           << last_thrust_force_request_ << ","
                           << last_thrust_force_limit_ << ","
                           << (velocity_takeoff_reached_ ? 1 : 0) << ","
                           << (last_wheels_airborne_ ? 1 : 0) << ","
                           << airborne_confidence_count_ << ","
                           << target_takeoff_velocity_ << ","
                           << last_tau_body_per_hip_ << ","
                           << last_thrust_reaction_ff_ << ","
                           << (current_state_ == bbot_jump::STATE_FLIGHT ? static_cast<int>(flight_subphase_) : -1) << ","
                           << (current_state_ == bbot_jump::STATE_RECOVERY ? static_cast<int>(recovery_subphase_) : -1) << ","
                           << controller_mode_str_ << ","
                           << thrust_motion_elapsed_ << ","
                           << (thrust_attitude_blocked_ ? 1 : 0) << ","
                           << air_wheel_cmd_raw_ << ","
                           << left_wheel_vel_ << ","
                           << right_wheel_vel_ << ","
                           << attitude_arrest_stable_count_ << ","
                           << touchdown_phase_diag_ << ","
                           << touchdown_brake_cmd_ref_ << ","
                           << touchdown_capture_diag_ << ","
                           << target_x_ << ","
                           << touchdown_x_error_diag_ << ","
                           << touchdown_cmd_target_diag_ << ","
                           << hip_vel_left_ << "," << knee_vel_left_ << ","
                           << hip_vel_right_ << "," << knee_vel_right_ << ","
                           << joint_velocity_cmd_[0] << "," << joint_velocity_cmd_[1] << ","
                           << joint_velocity_cmd_[2] << "," << joint_velocity_cmd_[3] << ","
                           << wheel_clearance_ << "," << contact_window_ << "," << thrust_extension_scale_ << "," << air_pd_horizon_ << ","
                           << air_pd_raw_[0] << "," << air_pd_raw_[1] << "," << air_pd_raw_[2] << "," << air_pd_raw_[3] << ","
                           << air_pd_discrete_[0] << "," << air_pd_discrete_[1] << "," << air_pd_discrete_[2] << "," << air_pd_discrete_[3] << ","
                           << jump_forward_speed_ << "," << jump_takeoff_forward_speed_ << ","
                           << jump_pitch_ref_ << "," << active_jump_pitch_ref_ << ","
                           << jump_takeoff_pitch_rate_ << "," << active_jump_pitch_rate_ref_ << ","
                           << pre_jump_stable_timer_ << "," << takeoff_forward_speed_ << ","
                           << air_wheel_baseline_ << ","
                           << gazebo_world_x_dot_ << "," << gazebo_world_y_dot_ << ","
                           << landing_capture_vx_ << ","
                           << landing_capture_raw_offset_ << "," << landing_capture_offset_ << ","
                           << landing_target_x_ << "," << landing_capture_comp_ << "," << landing_capture_omega_ << ","
                           << landing_forward_axis_x_ << "," << landing_forward_axis_y_ << ","
                           << (landing_capture_planned_ ? 1 : 0) << ","
                           << flight_air_pitch_ref_diag_ << "," << flight_air_pitch_rate_ref_diag_ << ","
                           << protective_deploy_duration_active_ << ","
                           << (protective_deploy_rate_limited_ ? 1 : 0) << ","
                           << takeoff_geometry_aligned_ << "," << takeoff_odom_stamp_ << ","
                           << takeoff_clearance_left_ << "," << takeoff_clearance_right_ << ","
                           << landing_wheel_ground_blend_ << "," << ground_pd_horizon_ << ","
                           << ground_pd_feedback_[0] << "," << ground_pd_feedback_[1] << ","
                           << ground_pd_feedback_[2] << "," << ground_pd_feedback_[3] << ","
                           << world_pose_velocity_.valid() << "," << odom_twist_z_diag_ << ","
                           << thrust_release_.active() << "," << thrust_release_.blend(this->now().seconds()) << ","
                           << touchdown_catch_stable_time_ << ","
                           << centroidal_height_.height() << "," << centroidal_height_.velocity() << ","
                           << centroidal_velocity_valid_ << "," << centroidal_height_.stamp() << ","
                           << centroidal_balance_.forward << "," << centroidal_balance_.height << ","
                           << centroidal_balance_.angle << "," << centroidal_balance_.rate << ","
                           << centroidal_balance_.valid << "," << thrust_vertical_velocity_ << ","
                           << ground_gravity_torque_[0] << "," << ground_gravity_torque_[1] << ","
                           << ground_gravity_torque_[2] << "," << ground_gravity_torque_[3] << ","
                           << active_effort_limits_.hip << "," << active_effort_limits_.knee << ","
                           << thrust_force_unlimited_request_ << "," << thrust_knee_pd_left_ << ","
                           << torso_torque_.force_feedforward << "," << torso_torque_.feedback << ","
                           << torso_torque_.command << "," << torso_control_rate_ << "," << torso_control_horizon_ << ","
                           << torso_imu_fresh_ << "," << torso_imu_.fy() << "," << torso_imu_.fz() << ","
                           << hip_common_before_allocation_ << "," << hip_differential_torque_ << "," << capture_com_velocity_ << ","
                           << capture_world_valid_ << "," << capture_world_active_ << ","
                           << capture_world_target_ << "," << recovery_ready_ << ","
                           << recovery_drive_ref_ << "," << recovery_yaw_ref_ << ","
                           << effort_jump_cycle_ << "," << thrust_knee_position_yield_ << "\n";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BBotVelocityJumpController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
