// Experimental landing repair based on the complete-jump baseline.
// Uses world-referenced IMU orientation for CAD geometry in Gazebo.
#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <filesystem>
#include <iomanip>
#include <limits>

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
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/allocator_flight_plan.hpp"
#include "bbot_balance_controller/takeoff_detection.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"
#include "bbot_balance_controller/latest_sensor_qos.hpp"
#include "bbot_balance_controller/control_timing.hpp"
#include "bbot_balance_controller/ground_joint_pd.hpp"
#include "bbot_balance_controller/torso_pitch_control.hpp"
#include "bbot_balance_controller/world_pose_velocity.hpp"
#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/thrust_forward_speed_predictor.hpp"
#include "bbot_balance_controller/thrust_wheel_kinematics_observer.hpp"
#include "bbot_balance_controller/complete_contact_takeoff.hpp"
#include "bbot_balance_controller/allocator_contact_history.hpp"
#include "bbot_balance_controller/native_command_state.hpp"
#include "bbot_balance_controller/thrust_velocity_reference.hpp"
#include "bbot_balance_controller/thrust_support_coordination.hpp"
#include "bbot_balance_controller/effort_allocation.hpp"
#include "bbot_balance_controller/rolling_jump_control.hpp"
#include "bbot_balance_controller/reference_ground_torque_pulse.hpp"
#include "bbot_balance_controller/landing_repair_momentum.hpp"
#include "bbot_balance_controller/landing_repair_thrust_reference.hpp"
#include "bbot_balance_controller/landing_repair_handoff.hpp"
#include "bbot_balance_controller/velocity_launch_capture.hpp"
#include "bbot_balance_controller/thrust_support_allocator.hpp"
#include "bbot_balance_controller/thrust_output_ownership.hpp"
#include "bbot_balance_controller/ground_motion_probe.hpp"
#include "bbot_balance_controller/ground_engine_state.hpp"
#include <stdexcept>
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

namespace bbot_jump
{
    inline double low_pass_filter(double new_value, double old_value, double alpha)
    {
        return alpha * new_value + (1.0 - alpha) * old_value;
    }

    inline double lerp(double a, double b, double ratio)
    {
        return a + (b - a) * ratio;
    }

    // 末端行程保护：只预测“离地前仍可能继续保持当前关节速度”的短时间窗口。
    // v5.6：上一轮在 knee≈-0.92 rad、-9.7 rad/s 时仍被 45 ms 预瞄提前削到 0.75，
    // 实际离地速度仅 1.83 m/s。缩短到 30 ms：仍保留高速末端保护，但不在尚有
    // 约 0.5 rad 行程时过早损失竖直冲量。硬关节限幅本身仍保持不变。
    inline double joint_extension_scale(double q, double v)
    {
        if (std::abs(v) < 1.0)
            return 1.0;
        constexpr double kSoftLimit = 1.56; // 离物理硬限位 1.5708 rad 保留软裕量
        const double remaining = v > 0.0 ? kSoftLimit - q : q + kSoftLimit;
        constexpr double preview_time = 0.002; // 微小前瞻，允许利用完整行程爆发
        constexpr double ramp_margin = 0.02;
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
        STATE_BALANCE = 0,          // 0: 变高度 LQR 自平衡状态
        STATE_SQUAT = 1,            // 1: 下蹲蓄力阶段 (L -> L_SQUAT_)
        STATE_THRUST = 2,           // 2: 爆发推地阶段 (L_SQUAT_ -> H_TAKEOFF_)
        STATE_FLIGHT = 3,           // 3: 腾空相阶段 (冲顶 -> 收腿 0.30m -> 预展腿 0.40m)
        STATE_TOUCHDOWN_BUFFER = 4, // 4: 触地缓冲阻抗控制
        STATE_RECOVERY = 5,         // 5: 平稳沉降与消除反弹
        STATE_STANDUP = 6,          // 6: 倒地起立自恢复
        STATE_EMERGENCY = 7,        // 7: 紧急停机
        STATE_PRE_JUMP = 8          // 8: 滚动起跳准备：建立前向速度与前倾工作点
    };

    /// @brief 腾空相内部子阶段枚举
    enum FlightSubphase
    {
        FLIGHT_SUBPHASE_ATTITUDE_ARREST = 0,  // 0: 姿态刹车阶段
        FLIGHT_SUBPHASE_TUCK = 1,             // 1: 五次平滑收腿阶段
        FLIGHT_SUBPHASE_EXTEND = 2,           // 2: 顶点展腿阶段
        FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY = 3 // 3: 保护展腿阶段
    };

    inline const char *flight_subphase_to_string(FlightSubphase s)
    {
        switch (s)
        {
        case FLIGHT_SUBPHASE_ATTITUDE_ARREST:
            return "ATTITUDE_ARREST";
        case FLIGHT_SUBPHASE_TUCK:
            return "TUCK";
        case FLIGHT_SUBPHASE_EXTEND:
            return "EXTEND";
        case FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY:
            return "PROTECTIVE_DEPLOY";
        default:
            return "UNKNOWN";
        }
    }

    /// @brief 恢复阶段内部交接子阶段枚举
    enum RecoverySubphase
    {
        RECOVERY_EFFORT_RAISE = 0,     // 0: 五次轨迹站高
        RECOVERY_EFFORT_STABILIZE = 1, // 1: 连续 0.50s 检查稳态
        RECOVERY_POSITION_PRELOAD = 2, // 2: 锁存实际构型并预发 0.10s
        RECOVERY_SWITCHING = 3,        // 3: STRICT 原子切换进行中
        RECOVERY_POSITION_HOLD = 4,    // 4: 保持锁存构型 0.25s
        RECOVERY_POSITION_RETURN = 5,  // 5: 0.8s 五次轨迹回到 L_STAND 对应 IK
        RECOVERY_COMPLETE = 6,         // 6: 切换完成进入 BALANCE
        // v6.0：THRUST 失败且轮子仍接地时，不允许直接保持长腿/站高。
        // 先缩腿到安全高度，待姿态和角速度缓和后再重新站起。
        RECOVERY_FAIL_CROUCH = 7,   // 7: 失败接地缩腿
        RECOVERY_FAIL_STABILIZE = 8 // 8: 低位姿态稳定
    };

    inline const char *recovery_subphase_to_string(RecoverySubphase s)
    {
        switch (s)
        {
        case RECOVERY_EFFORT_RAISE:
            return "EFFORT_RAISE";
        case RECOVERY_EFFORT_STABILIZE:
            return "EFFORT_STABILIZE";
        case RECOVERY_POSITION_PRELOAD:
            return "POSITION_PRELOAD";
        case RECOVERY_SWITCHING:
            return "SWITCHING";
        case RECOVERY_POSITION_HOLD:
            return "POSITION_HOLD";
        case RECOVERY_POSITION_RETURN:
            return "POSITION_RETURN";
        case RECOVERY_COMPLETE:
            return "COMPLETE";
        case RECOVERY_FAIL_CROUCH:
            return "FAIL_CROUCH";
        case RECOVERY_FAIL_STABILIZE:
            return "FAIL_STABILIZE";
        default:
            return "UNKNOWN";
        }
    }

    inline const char *state_to_string(JumpState s)
    {
        switch (s)
        {
        case STATE_BALANCE:
            return "BALANCE";
        case STATE_PRE_JUMP:
            return "PRE_JUMP";
        case STATE_SQUAT:
            return "SQUAT";
        case STATE_THRUST:
            return "THRUST";
        case STATE_FLIGHT:
            return "FLIGHT";
        case STATE_TOUCHDOWN_BUFFER:
            return "TOUCHDOWN_BUFFER";
        case STATE_RECOVERY:
            return "RECOVERY";
        case STATE_STANDUP:
            return "STANDUP";
        case STATE_EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN";
        }
    }

} // namespace bbot_jump

class BBotLandingRepairController : public rclcpp::Node
{
public:
    BBotLandingRepairController()
        : Node("bbot_velocity_jump_controller")
    {
        gain_low_ = {-6.1624, -45.8436, -179.6985, -42.8109};
        gain_high_ = {-6.3650, -49.5719, -233.4004, -62.6391};
        current_gain_ = gain_high_;

        balance_offset_ = 0.030;
        const double initial_imu_roll = this->declare_parameter<double>(
            "sim_imu_reference_roll", 0.0);
        bool simulation_clock = false;
        this->get_parameter_or("use_sim_time", simulation_clock, false);
        if (simulation_clock && std::isfinite(initial_imu_roll))
        {
            sim_imu_reference_roll_ = initial_imu_roll;
            imu_world_reference_.setRPY(initial_imu_roll, 0.0, 0.0);
            // Keep the original physical standing / body-PD targets while
            // converting their old relative-IMU frame into the world frame.
            balance_offset_ -= initial_imu_roll;
        }
        cmd_scale_ = 0.043;
        wheel_radius_ = 0.07;
        max_cmd_x_ = 10.0;

        walk_speed_ = 0.50;
        turn_speed_ = 0.60;
        speed_ramp_time_ = 1.0;

        // ── jump-thrust-v5：滚动起跳准备参数 ──
        // 这些是首轮 Gazebo 试验值，不是辨识后的最优参数；全部可通过 ROS 参数覆盖。
        // PRE_JUMP 只建立稳定的接近速度；SQUAT 末段和 THRUST 再把速度推到离地目标。
        jump_forward_speed_ = this->declare_parameter<double>("jump_forward_speed", 0.35);
        jump_forward_speed_ = bbot_jump::clamp_value(jump_forward_speed_, 0.0, 0.70);
        jump_takeoff_forward_speed_ = this->declare_parameter<double>("jump_takeoff_forward_speed", 0.45);
        jump_takeoff_forward_speed_ = bbot_jump::clamp_value(
            jump_takeoff_forward_speed_, 0.20, 0.85);
        jump_log_path_ = this->declare_parameter<std::string>("jump_log_path", "");
        jump_summary_path_ = this->declare_parameter<std::string>("jump_summary_path", "");
        auto_return_balance_ = this->declare_parameter<bool>("auto_return_balance", true);
        thrust_forward_velocity_kp_ = this->declare_parameter<double>(
            "thrust_forward_velocity_kp", 1.20);
        thrust_forward_velocity_kp_ = bbot_jump::clamp_value(
            thrust_forward_velocity_kp_, 0.0, 2.0);
        thrust_forward_attitude_taper_enable_ = this->declare_parameter<bool>(
            "thrust_forward_attitude_taper", true);
        thrust_release_fast_rate_correction_enable_ = this->declare_parameter<bool>(
            "thrust_release_fast_rate_correction", true);
        thrust_fast_rate_correction_from_gate_ = this->declare_parameter<bool>(
            "thrust_fast_rate_correction_from_gate", true);
        const double requested_release_ratio = this->declare_parameter<double>(
            "thrust_release_velocity_ratio", 0.78);
        thrust_release_velocity_ratio_ = std::isfinite(requested_release_ratio)
            ? bbot_jump::clamp_value(requested_release_ratio, 0.70, 0.78)
            : 0.78;
        thrust_forward_speed_prediction_enable_ = this->declare_parameter<bool>(
            "thrust_forward_speed_prediction", true);
        thrust_wheel_kinematics_compensation_enable_ = this->declare_parameter<bool>(
            "thrust_wheel_kinematics_compensation", true);
        arrest_dynamics_feedforward_enable_ = this->declare_parameter<bool>(
            "arrest_dynamics_feedforward", true);
        complete_contact_takeoff_confirmation_enable_ = this->declare_parameter<bool>(
            "complete_contact_takeoff_confirmation", true);
        thrust_wheel_max_decel_ = this->declare_parameter<double>(
            "thrust_wheel_max_decel", 16.0);
        thrust_wheel_max_decel_ = bbot_jump::clamp_value(
            thrust_wheel_max_decel_, 4.0, 50.0);
        // Diagnostic-only contact-period hip torque pulse, enabled only in this
        // independent landing-repair controller; the baseline controller is untouched.
        reference_ground_torque_pulse_enable_ = this->declare_parameter<bool>(
            "reference_ground_torque_pulse", false);
        thrust_momentum_reference_enable_ = this->declare_parameter<bool>(
            "thrust_momentum_reference", false);
        thrust_reference_handoff_enable_ = this->declare_parameter<bool>(
            "thrust_reference_handoff", false);
        if (thrust_reference_handoff_enable_ && !thrust_momentum_reference_enable_)
            throw std::runtime_error("thrust_reference_handoff requires thrust_momentum_reference");
        thrust_support_coordination_enable_ = this->declare_parameter<bool>(
            "thrust_support_coordination", false);
        thrust_support_allocator_enable_ = this->declare_parameter<bool>(
            "thrust_support_allocator", false);
        ground_input_experiment_ = this->declare_parameter<bool>("ground_input_experiment", false);
        ground_motion_experiment_ = this->declare_parameter<bool>("ground_motion_experiment", false);
        ground_contact_motion_experiment_ = this->declare_parameter<bool>("ground_contact_motion_experiment", false);
        if (ground_contact_motion_experiment_ && !ground_motion_experiment_)
            throw std::runtime_error("ground_contact_motion_experiment requires ground_motion_experiment");
        if (ground_motion_experiment_ && !ground_input_experiment_)
            throw std::runtime_error("ground_motion_experiment requires ground_input_experiment");
        velocity_capture_enable_ = this->declare_parameter<bool>("velocity_capture_experiment", false);
        if (velocity_capture_enable_ && (thrust_support_allocator_enable_ ||
            thrust_momentum_reference_enable_ || thrust_reference_handoff_enable_ ||
            thrust_support_coordination_enable_ || reference_ground_torque_pulse_enable_))
            throw std::runtime_error("velocity_capture_experiment must run alone");
        velocity_capture_hip_decel_ = this->declare_parameter<double>("velocity_capture_hip_brake_decel", 0.0);
        velocity_capture_knee_decel_ = this->declare_parameter<double>("velocity_capture_knee_brake_decel", 0.0);
        velocity_capture_delay_ = this->declare_parameter<double>("velocity_capture_command_delay", 0.015);
        if (!std::isfinite(velocity_capture_hip_decel_) || velocity_capture_hip_decel_<0 ||
            !std::isfinite(velocity_capture_knee_decel_) || velocity_capture_knee_decel_<0 ||
            !std::isfinite(velocity_capture_delay_) || velocity_capture_delay_<0)
            throw std::runtime_error("invalid velocity capture brake qualification");
        native_state_diagnostics_ = this->declare_parameter<bool>("native_command_state_diagnostics", false);
        if (ground_input_experiment_ && !native_state_diagnostics_)
            throw std::runtime_error("ground_input_experiment requires native command diagnostics");
        wheel_servo_gain_ = this->declare_parameter<double>("wheel_servo_gain", 1.0);
        thrust_support_target_h_ = this->declare_parameter<double>("allocator_target_H", -0.08);
        if (!std::isfinite(thrust_support_target_h_))
            throw std::runtime_error("allocator_target_H must be finite");
        if (wheel_servo_gain_ != 0.5 && wheel_servo_gain_ != 1.0 && wheel_servo_gain_ != 2.0)
            throw std::runtime_error("wheel_servo_gain must be 0.5, 1.0, or 2.0");
        if (ground_input_experiment_ &&
            (thrust_support_allocator_enable_ || velocity_capture_enable_ ||
             thrust_momentum_reference_enable_ || thrust_reference_handoff_enable_ ||
             thrust_support_coordination_enable_ || reference_ground_torque_pulse_enable_ ||
             wheel_servo_gain_ != 1.0))
            throw std::runtime_error(
                "ground_input_experiment requires gain=1 and all allocator/capture/thrust experiments disabled");
        if (thrust_support_allocator_enable_ &&
            (thrust_momentum_reference_enable_ || thrust_reference_handoff_enable_ ||
             thrust_support_coordination_enable_ || reference_ground_torque_pulse_enable_))
            throw std::runtime_error(
                "thrust_support_allocator is mutually exclusive with other THRUST experiments");
        if (thrust_support_coordination_enable_ &&
            (thrust_momentum_reference_enable_ || thrust_reference_handoff_enable_ ||
             reference_ground_torque_pulse_enable_))
            throw std::runtime_error(
                "thrust_support_coordination is mutually exclusive with momentum_reference, handoff, and ground_torque_pulse");

        // 机身前倾角微偏置；避免大前倾推力产生过度水平分量导致前向速度过冲。
        jump_pitch_offset_ = this->declare_parameter<double>("jump_pitch_offset", 0.020);
        jump_pitch_offset_ = bbot_jump::clamp_value(jump_pitch_offset_, 0.0, 0.16);
        jump_takeoff_pitch_rate_ = this->declare_parameter<double>(
            "jump_takeoff_pitch_rate", 0.05);
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
            "landing_wheel_back_bias", 0.120);
        landing_wheel_back_bias_ = bbot_jump::clamp_value(
            landing_wheel_back_bias_, 0.0, 0.12);
        // 水平速度越大，轮子按捕获点方向前置。平地低跳的离地长腿已接近
        // wheel-first 构型；把轮位限制在 -0.03 m 会迫使髋在约 0.20 s 内
        // 额外反摆 0.4 rad。允许到 CAD/小腿安全锥内的 -0.12 m，既缩短
        // 关节行程，也让支撑点落在前向 COM 速度的捕获方向。
        landing_capture_gain_ = this->declare_parameter<double>(
            "landing_capture_gain", 1.60);
        landing_capture_gain_ = bbot_jump::clamp_value(
            landing_capture_gain_, 0.0, 2.00);
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
        current_height_ = 0.40;                          // 从仿真自然落地构型 (0.40m) 平滑渐变至 L_STAND_ (0.50m)
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0; // 0.05 m/s

        // ── 跳跃核心参数 ──
        // 不做过深、过快的下蹲：位置控制器切到 Effort 的短暂过渡期间，
        // 0.30 m / 0.15 s 会让机身先失去支撑再来不及推地。
        L_SQUAT_ = 0.34; // 下蹲蓄力高度 [m]
        T_SQUAT_ = 0.50; // 下蹲过渡时间 [s]，与旧稳定控制器一致

        T_THRUST_ = 0.10;   // 推地规划时间 [s]
        H_TAKEOFF_ = 0.475; // 离地目标高度 [m]
        V_TAKEOFF_ = 2.30;  // 离地初速度 [m/s]
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
            "thrust_leg_reaction_ff_gain", 0.15);
        K_LEG_REACTION_FF_THRUST_ = bbot_jump::clamp_value(
            K_LEG_REACTION_FF_THRUST_, 0.0, 3.0);
        TAU_LEG_REACTION_FF_MAX_ = this->declare_parameter<double>(
            "thrust_leg_reaction_ff_max", 2.0);
        TAU_LEG_REACTION_FF_MAX_ = bbot_jump::clamp_value(
            TAU_LEG_REACTION_FF_MAX_, 0.0, 12.0);
        K_BODY_P_BUFFER_ = 55.0; // 缓冲阶段姿态刚度 [Nm/rad]
        K_BODY_D_BUFFER_ = 15.0; // 缓冲阶段姿态阻尼 [Nm*s/rad]

        // 轮式机器人应以较长腿让轮子先接地，再在接触后压缩吸能。
        // 旧 0.47 -> 0.50 m 空中构型会把接近完全伸直的离地腿收成深蹲，
        // 实测需要约 0.9~1.2 rad 的关节重构并把箱体推到 -1 rad 以上。
        // 0.66 -> 0.69 m 保留明确的 0.03 m 收展动作，同时显著降低空中
        // 内部角动量。0.72 m 着陆时实测膝关节直接撞到 -1.5708 rad
        // 硬限位，冲击后无膝部压缩行程；0.69 m 的着陆膝目标约为
        // -1.28 rad，保留约 0.29 rad 机械缓冲余量。
        L_RETRACT_ = velocity_capture_enable_ ? 0.60 : 0.66;
        L_TOUCH_ = velocity_capture_enable_ ? 0.60 : 0.69;   // 腾空 wheel-first 着陆高度 [m]
        flight_hip_speed_limit_ = this->declare_parameter<double>(
            "flight_hip_speed_limit", 11.0);
        flight_knee_speed_limit_ = this->declare_parameter<double>(
            "flight_knee_speed_limit", 13.0);
        flight_hip_acc_limit_ = this->declare_parameter<double>(
            "flight_hip_acc_limit", 450.0);
        flight_knee_acc_limit_ = this->declare_parameter<double>(
            "flight_knee_acc_limit", 500.0);
        flight_hip_pos_limit_ = this->declare_parameter<double>(
            "flight_hip_pos_limit", 1.52);
        flight_knee_pos_limit_ = this->declare_parameter<double>(
            "flight_knee_pos_limit", 1.5708);
        L_BUFFER_SETTLE_ = this->declare_parameter<double>("landing_buffer_height", 0.34);
        L_BUFFER_SETTLE_ = bbot_jump::clamp_value(
            L_BUFFER_SETTLE_, L_SQUAT_ + 0.01, L_TOUCH_ - 0.02);
        // 触地时先保持空中末帧的实际关节构型，再平滑交给缓冲 IK。
        // 实际交接时长还会按长腿落地的缓冲行程自动放大。
        landing_joint_handoff_duration_ = this->declare_parameter<double>(
            "landing_joint_handoff_duration", 0.16);
        landing_joint_handoff_duration_ = bbot_jump::clamp_value(
            landing_joint_handoff_duration_, 0.08, 0.70);
        // 放慢收腿，避免腿部反作用角动量把箱体继续推向后仰。
        T_FLIGHT_TUCK_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("flight_tuck_nominal_duration", 0.06),
            0.06, 0.12); // 实际时长仍由 round-trip 可行性规划器决定
        T_FLIGHT_APEX_ = 0.30;       // 高跳时仍可在顶点附近开始展腿；低跳由剩余时间提前触发
        T_FLIGHT_EXTEND_ = 0.090;    // v6.2：给收腿腾出时间，同时保留wheel-first展腿 [s]
        T_PROTECTIVE_DEPLOY_ = 0.24; // 超标离地后，从离地初速度连续过渡到着陆构型
        T_FLIGHT_TIMEOUT_ = 0.60;    // 腾空超时保护阈值 [s]
        PITCH_FLIGHT_GUARD_ = 0.45;  // 腾空姿态保护阈值 [rad]

        // 落地速度约 1.5 m/s 时，原 160 N/腿上限与缓慢建力不足以在
        // 有效腿程内吸收动能。允许用户已放宽的关节力矩用于触地承重。
        K_Z_BUFFER_ = 450.0;     // 单腿垂直刚度 [N/m]
        D_Z_BUFFER_ = 75.0;      // 单腿垂直阻尼 [N*s/m]
        F_Z_BUFFER_MAX_ = 240.0; // 单腿最大缓冲支撑力 [N]
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
        // URDF 腿关节 velocity=30 rad/s；旧的 6 rad/s 轨迹上限无法表达
        // 1.98 m/s 的 COM 离地速度所需膝速。
        // 试验/调参值：30 实测膝速到 -11.6 rad/s，离地 z_dot 2.84 但
        // pitch_rate -0.96 → protective_takeoff → 落地后翻倒。
        // 8 实测离地 knee_v -9.9~-10.3、hip_v 4.66，同时打断
        // flight_leg_motion_ready()(knee≤8) 与 tuck_joint_speed_safe()(hip≤2.8,
        // knee≤4.5)，导致 v6.14 的正常 TUCK/EXTEND 路径一次都进不去。
        thrust_knee_velocity_limit_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("thrust_knee_velocity_limit", 15.0), 1.0, 30.0);
        thrust_torque_margin_ = this->declare_parameter<double>("thrust_torque_margin", 0.95);
        // 仅 Gazebo 启动文件默认开启；离开 THRUST 或关闭仿真时钟后恢复75/60。
        sim_relax_thrust_limits_ = this->declare_parameter<bool>("sim_relax_thrust_limits", false);
        // 推地轨迹结束不等于轮子已离地。留出额外时间给速度闭环维持
        // 有效接地推力，直到失重确认；该超时仍是未离地时的保护上限。
        thrust_timeout_ = this->declare_parameter<double>("thrust_timeout", 0.60);
        target_takeoff_velocity_ = takeoff_velocity_override_ > 0.0 ? takeoff_velocity_override_ : std::sqrt(2.0 * 9.81 * std::max(0.01, jump_height_target_));
        thrust_duration_ = bbot_jump::clamp_value(thrust_duration_, 0.08, 0.32);
        thrust_timeout_ = bbot_jump::clamp_value(thrust_timeout_, thrust_duration_ + 0.12, 0.75);
        thrust_peak_ratio_ = bbot_jump::clamp_value(thrust_peak_ratio_, 1.0, 4.0);
        grounded_launch_boost_ratio_ = bbot_jump::clamp_value(
            grounded_launch_boost_ratio_, 0.0, 1.50);
        thrust_shape_early_ = bbot_jump::clamp_value(thrust_shape_early_, 0.05, 2.0);
        thrust_shape_late_ = bbot_jump::clamp_value(thrust_shape_late_, 0.05, 2.0);
        thrust_velocity_kp_ = bbot_jump::clamp_value(thrust_velocity_kp_, 0.0, 30.0);
        thrust_torque_margin_ = bbot_jump::clamp_value(thrust_torque_margin_, 0.80, 1.0);
        // 轮子前进为负轮速，机身后仰(pitch<0, gyro<0)需轮子向后加速(正轮加速度)产生前倾反作用力矩。
        // 因此 air_wheel_sign 默认为 -1.0。
        air_wheel_sign_ = this->declare_parameter<double>("air_wheel_sign", -1.0);
        // URDF 连续轮关节的物理速度上限为 30 rad/s。以 0.07 m 轮半径
        // 换算，2.0 m/s 对应 28.6 rad/s，保留少量执行器裕量。旧的 1.4 m/s
        // 在收腿中约 80 ms 就饱和到 20 rad/s，剩余空中姿态误差无法继续吸收。
        air_wheel_linear_speed_limit_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("air_wheel_linear_speed_limit", 2.0),
            0.70, 2.0);
        air_wheel_extend_kd_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("air_wheel_extend_kd", 2.50),
            0.20, 2.50);
        air_wheel_tuck_kd_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("air_wheel_tuck_kd", 0.45),
            0.20, 2.50);
        flight_arrest_freewheel_duration_ = bbot_jump::clamp_value(
            this->declare_parameter<double>("flight_arrest_freewheel_duration", 0.0), 0.0, 0.020);
        thrust_terminal_hip_floor_ =
            this->declare_parameter<bool>("thrust_terminal_hip_floor", false);

        RCLCPP_INFO(this->get_logger(),
                    "[jump-thrust-v6.2] 强制实际状态TUCK / 去除round-trip误拦截 / 低跳立即再展腿");
        RCLCPP_INFO(this->get_logger(),
                    "[rolling-jump-config] approach_v=%.3f takeoff_v=%.3f m/s pitch_ref=%.3f rad "
                    "takeoff_pitch_rate=%.3f rad/s thrust_forward_velocity_kp=%.3f "
                    "forward_attitude_taper=%d release_fast_rate_correction=%d gate_fast_rate_scope=%d release_ratio=%.3f "
                    "forward_speed_prediction=%d prepare_timeout=%.2fs",
                    jump_forward_speed_, jump_takeoff_forward_speed_, jump_pitch_ref_,
                    jump_takeoff_pitch_rate_, thrust_forward_velocity_kp_,
                    thrust_forward_attitude_taper_enable_ ? 1 : 0,
                    thrust_release_fast_rate_correction_enable_ ? 1 : 0,
                    thrust_fast_rate_correction_from_gate_ ? 1 : 0,
                    thrust_release_velocity_ratio_,
                    thrust_forward_speed_prediction_enable_ ? 1 : 0, pre_jump_timeout_);
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

        RCLCPP_INFO(this->get_logger(), "[takeoff-sync-v6.3] 同时刻轮底几何 / 独立采样确认 / 12mm进入6mm退出");

        RCLCPP_INFO(this->get_logger(), "[ground-handoff-v6.4] 有限前倾参考 / 全程推地姿态轮控 / 着陆轮控提前衔接");

        RCLCPP_INFO(this->get_logger(), "[ground-sampling-v6.5] 仿真时钟门控 / 持续接触确认 / 地面离散反馈 / 世界速度修正");

        RCLCPP_INFO(this->get_logger(), "[launch-capture-v6.6] 推地收尾锁存 / 明确净空确认 / 落地持续捕获与再捕获");

        RCLCPP_INFO(this->get_logger(), "[centroidal-v6.7] 整机质心速度推地 / 重心相对轮轴捕获 / 保留空中离散关节反馈");

        RCLCPP_INFO(this->get_logger(),
                    "[effort-allocation-v6.8] 有符号推力预算 / 地面腿部重力补偿 / 仿真推地放宽=%d (150Nm)",
                    (sim_relax_thrust_limits_ && this->get_parameter("use_sim_time").as_bool()) ? 1 : 0);

        RCLCPP_INFO(this->get_logger(),
                    "[com-hold-v6.11 +thrust-vref-v6.13 +flight-plan-v6.14 +fall-guard-v6.27 "
                    "+support-exit-v6.28 +capture-anchor-b-v6.31 +terrain-free-exit-v6.32] "
                    "捕获停车恢复共用COM轮控 / "
                    "静稳后保持Effort RECOVERY / THRUST 用COM速度反解膝参考 / "
                    "FLIGHT 收展腿按整程轨迹预算准入 / 缓冲段不可恢复俯仰即停轮退出 / "
                    "FLIGHT 用躯干IMU比力判支撑即退出 / "
                    "地面静稳期限幅积分学掉速度不动点、保留 ωr 姿态反馈 / "
                    "缓冲交权改用与支撑面无关的捕获静稳判据");

        // 日志路径初始化
        const char *home_dir = getenv("HOME");
        data_path_ = std::string(home_dir ? home_dir : "/home/admin") + "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/";
        open_log_files();

        // ── 订阅话题 ──
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", bbot_jump::latest_sensor_qos(),
            std::bind(&BBotLandingRepairController::imu_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/bbot/odometry", bbot_jump::latest_sensor_qos(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                if (!std::isfinite(msg->pose.pose.position.x) ||
                    !std::isfinite(msg->pose.pose.position.y) ||
                    !std::isfinite(msg->pose.pose.position.z))
                    return;
                odom_base_position_ = {msg->pose.pose.position.x,
                                       msg->pose.pose.position.y, msg->pose.pose.position.z};
                gazebo_world_z_ = msg->pose.pose.position.z;
                last_world_odom_time_ = this->now().seconds();
                takeoff_odom_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
                const auto &orientation = msg->pose.pose.orientation;
                tf2::Quaternion body_q(orientation.x, orientation.y, orientation.z, orientation.w);
                takeoff_odom_pose_valid_ = std::isfinite(body_q.length2()) && body_q.length2() > 1e-8;
                if (takeoff_odom_pose_valid_)
                {
                    body_q.normalize();
                    const tf2::Matrix3x3 rotation(body_q);
                    for (int row = 0; row < 3; ++row)
                        for (int col = 0; col < 3; ++col)
                            odom_body_rotation_(row, col) = rotation[row][col];
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(body_q).getRPY(roll, pitch, yaw);
                    takeoff_odom_pitch_ = -roll;
                    const auto vertical_axis = tf2::Matrix3x3(body_q).getRow(2);
                    odom_world_z_in_body_ = {vertical_axis.x(), vertical_axis.y(), vertical_axis.z()};
                }
                // Gazebo的3D twist在机身系且内部已滤波；不能直接把z当世界vz。
                // 按世界位置差分只保留一个odom区间延迟，不再串联0.25慢低通。
                odom_twist_z_diag_ = msg->twist.twist.linear.z;
                world_pose_velocity_.update(takeoff_odom_stamp_,
                                            {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z});
                const bool velocity_valid = world_pose_velocity_.valid();
                world_xy_dot_filter_initialized_ = velocity_valid;
                world_z_dot_filter_initialized_ = velocity_valid;
                const auto &world_velocity = world_pose_velocity_.velocity();
                gazebo_world_x_dot_ = velocity_valid ? world_velocity[0] : 0.0;
                gazebo_world_y_dot_ = velocity_valid ? world_velocity[1] : 0.0;
                gazebo_world_z_dot_ = velocity_valid ? world_velocity[2] : 0.0;
                odom_received_ = true;
                if (current_state_ != bbot_jump::STATE_BALANCE)
                {
                    if (!world_height_valid_for_jump_ &&
                        current_state_ == bbot_jump::STATE_SQUAT)
                    {
                        thrust_start_world_z_ = gazebo_world_z_;
                        max_world_z_during_jump_ = gazebo_world_z_;
                        world_height_valid_for_jump_ = true;
                    }
                    max_world_z_during_jump_ = std::max(
                        max_world_z_during_jump_, gazebo_world_z_);
                }
            });

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", bbot_jump::latest_sensor_qos(),
            std::bind(&BBotLandingRepairController::joint_state_callback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                target_speed_const_ = msg->linear.x;
                target_yaw_rate_ = msg->angular.z;
                if (std::abs(msg->linear.x) < 0.001 && std::abs(msg->angular.z) < 0.001)
                {
                    if (was_moving_)
                    {
                        target_x_ = x_;
                        was_moving_ = false;
                    }
                }
            });

        target_height_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target_height", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (current_state_ == bbot_jump::STATE_BALANCE)
                {
                    target_height_ = bbot_jump::clamp_value(msg->data, L_MIN_, L_MAX_);
                }
            });

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                handle_mode_command(msg->data);
            });

        jump_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/jump_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "jump" || msg->data == "J" || msg->data == "j")
                {
                    jump_cmd_rx_stamp_ = this->now().seconds();
                    trigger_jump();
                }
            });

        if (complete_contact_takeoff_confirmation_enable_ || velocity_capture_enable_ || thrust_momentum_reference_enable_ ||
            thrust_support_coordination_enable_ || thrust_support_allocator_enable_ || ground_input_experiment_)
        {
            contact_frame_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/world/flat_jump_world/ground_contact_frames",
                rclcpp::QoS(2048).best_effort(),
                [this](const std_msgs::msg::String::SharedPtr msg)
                {
                    bbot_jump::CompleteGroundContactFrame frame;
                    if (!bbot_jump::CompleteGroundContactWire::parse(msg->data, frame))
                    {
                        frame.decoded = false;
                        contact_takeoff_observer_.receive(frame);
                        record_reference_ground_pulse_contact_frame(frame);
                        return;
                    }
                    contact_takeoff_observer_.receive(frame);
                    record_reference_ground_pulse_contact_frame(frame);
                });
        }

        if (native_state_diagnostics_) {
            if ((!thrust_support_allocator_enable_ && !ground_input_experiment_) || jump_log_path_.empty())
                throw std::runtime_error("native diagnostics require allocator or ground-input experiment and explicit log path");
            native_state_log_.open(jump_log_path_+".native_state.csv");
            command_publication_log_.open(jump_log_path_+".command_publication.csv");
            if (!native_state_log_ || !command_publication_log_)
                throw std::runtime_error("cannot open private native/publication audit logs");
            native_state_log_<<"receipt_ns,magic,sim_ns,iteration,dt_ns,qmask,vmask,before_vmask,input_mask";
            for(const char*part:{"q","v","before_v"})for(int i=0;i<9;++i)native_state_log_<<','<<part<<i;
            for(int i=0;i<6;++i)native_state_log_<<",simulation_input"<<i;
            native_state_log_<<'\n';
            command_publication_log_<<"command_id,publish_ns,control_ns,state,kind,c0,c1,c2,c3,wheel_linear,wheel_angular,zero_effort,publish_end_ns,wall_publish_ns,wall_publish_end_ns\n";
            native_state_sub_=this->create_subscription<std_msgs::msg::String>(
                "/world/flat_jump_world/native_command_state",rclcpp::QoS(2048).best_effort(),
                [this](const std_msgs::msg::String::SharedPtr msg){
                    bbot_jump::NativeCommandState state;
                    if(bbot_jump::NativeCommandStateWire::parse(msg->data,state))native_state_history_.push(state);
                    else native_state_history_.clear();
                    native_state_log_<<this->now().nanoseconds()<<','<<msg->data<<'\n';
                });
        }
        if (ground_contact_motion_experiment_) {
            ground_engine_source_log_.open(jump_log_path_ + ".engine_source.csv");
            if (!ground_engine_source_log_)
                throw std::runtime_error("cannot open direct engine control-state log");
            ground_engine_source_log_ << "receipt_ns,wall_receipt_ns,parse_valid,history_accepted,magic,sim_ns,iteration,dt_ns,qmask,vmask,before_vmask,input_mask";
            for (const char *part : {"q", "v", "before_v"})
                for (int i = 0; i < 9; ++i) ground_engine_source_log_ << ',' << part << i;
            for (int i = 0; i < 6; ++i) ground_engine_source_log_ << ",simulation_input" << i;
            ground_engine_source_log_ << '\n';
            ground_engine_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/ground_input_engine_state", rclcpp::QoS(128),
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    bbot_jump::NativeCommandState state;
                    const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    const bool parsed = bbot_jump::GroundEngineStateWire::parse(msg->data, state);
                    const bool accepted = parsed && ground_engine_history_.push(state, wall);
                    if (!accepted) {
                        ground_engine_history_.clear();
                        ground_engine_selected_valid_ = false;
                        if (ground_motion_law_active()) fail_ground_motion("direct_engine_source_invalid_or_clock_discontinuous");
                    }
                    ground_engine_source_log_ << this->now().nanoseconds() << ',' << wall << ','
                        << parsed << ',' << accepted << ',' << msg->data << '\n';
                });
        }
        if (ground_input_experiment_) {
            if (jump_log_path_.empty())
                throw std::runtime_error("ground_input_experiment requires explicit jump_log_path");
            ground_input_log_.open(jump_log_path_ + ".ground_input.csv");
            if (!ground_input_log_)
                throw std::runtime_error("cannot open ground-input experiment log");
            ground_input_log_ << "ros_ns,stage,reason,stable_time,effort_active,switch_pending,post_effort_support,contact_valid,contact_continuous,contact_wheel_mask,contact_sequence,contact_stamp_ns,imu_stamp,joint_stamp,odom_stamp,pitch,pitch_rate,ground_angle,ground_rate,body_vx,com_vx,com_vz,height,z,hip_l,knee_l,hip_r,knee_r,hip_v_l,knee_v_l,hip_v_r,knee_v_r,wheel_left_rate,wheel_right_rate\n";
            ground_input_stage_ = "waiting";
            ground_input_reason_ = "awaiting_enter_effort";
            if (ground_motion_experiment_) {
                motion_trace_log_.open(jump_log_path_ + ".motion_trace.csv");
                phase_events_log_.open(jump_log_path_ + ".phase_events.csv");
                if (!motion_trace_log_ || !phase_events_log_)
                    throw std::runtime_error("cannot open ground-motion probe logs");
                motion_trace_log_ << "trial_id,replay_id,phase,stage,reason,sim_time_ns,normal_stop,fault,failed,complete,effort_active,switch_pending,contact_valid,contact_continuous,contact_wheel_mask,joint_source_valid,joint_source_ns,imu_source_valid,imu_source_ns,odom_source_valid,odom_source_ns,contact_source_valid,contact_source_ns,contact_seq,command_id,control_ns,publish_ns,publish_end_ns,wall_publish_ns,wall_publish_end_ns,pitch_delta,pitch_rate,anchor_error_max,joint_rate_max,soft_margin_min,qref_hl,qref_kl,qref_hr,qref_kr,vref_hl,vref_kl,vref_hr,vref_kr,aref_hl,aref_kl,aref_hr,aref_kr,start_anchor_hl,start_anchor_kl,start_anchor_hr,start_anchor_kr,q_hl,q_kl,q_hr,q_kr,v_hl,v_kl,v_hr,v_kr,wheel_left_rate,wheel_right_rate,wheel_cmd_linear,wheel_cmd_angular,gravity_jzf_hl,gravity_jzf_kl,gravity_jzf_hr,gravity_jzf_kr,joint_feedback_hl,joint_feedback_kl,joint_feedback_hr,joint_feedback_kr,torso_static_ff,torso_dynamic_pd,effort_hl,effort_kl,effort_hr,effort_kr\n";
                phase_events_log_ << "trial_id,replay_id,phase,event,event_id,sim_event_ns,wall_event_ns,command_id,command_kind,reason,target_hl,target_kl,target_hr,target_kr,target_wheel_linear,target_wheel_angular\n";
                ground_motion_stage_ = "idle";
                ground_motion_reason_ = "awaiting_start_motion";
            }
            ground_input_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/ground_input_cmd", 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    if (msg->data == "start_motion") {
                        if (!ground_motion_experiment_ || ground_motion_start_requested_ ||
                            ground_motion_started_ || ground_motion_failed_ ||
                            ground_input_stage_ != "effort_hold") {
                            ground_input_reason_ = "ground_motion_start_not_allowed";
                            return;
                        }
                        ground_motion_start_requested_ = true;
                        ground_motion_start_request_time_ = this->now().seconds();
                        ground_motion_reason_ = "start_requested_waiting_stable_support";
                        return;
                    }
                    if (msg->data != "enter_effort") {
                        ground_input_reason_ = "unknown_command";
                        return;
                    }
                    if (current_state_ != bbot_jump::STATE_BALANCE ||
                        ground_input_stage_ == "failed" ||
                        ground_input_stage_ == "switching" ||
                        ground_input_stage_ == "effort_hold") {
                        if (ground_input_stage_ != "failed")
                            ground_input_reason_ = "command_not_allowed_in_current_stage";
                        return;
                    }
                    ground_input_requested_ = true;
                    ground_input_reason_ = "request_received_waiting_stable_support";
                });
        }

        if (velocity_capture_enable_) {
            if (jump_log_path_.empty()) throw std::runtime_error("velocity capture requires explicit trial log");
            velocity_capture_log_.open(jump_log_path_+".capture_control.csv");
            velocity_capture_log_<<"control_s,jump_id,phase,reason,owns_command,legacy_terms_active,pair_stamp,inverse_valid,sensors_fresh,support,stop_valid,stop_brake,latency,hip_decel,knee_decel";
            for (const char* part:{"q","v","requested_v","applied_v","remaining","required","issued_tau"})
                for(int i=0;i<4;++i) velocity_capture_log_<<','<<part<<i;
            velocity_capture_log_<<",force_per_leg\n";
            if (!native_state_diagnostics_) {
                command_publication_log_.open(jump_log_path_+".command_publication.csv");
                command_publication_log_<<"command_id,publish_ns,control_ns,state,kind,c0,c1,c2,c3,wheel_linear,wheel_angular,zero_effort,publish_end_ns,wall_publish_ns,wall_publish_end_ns\n";
            }
            if (!velocity_capture_log_ || !command_publication_log_) throw std::runtime_error("cannot open velocity capture audit logs");
        }
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
        timer_ = this->create_wall_timer(5ms, std::bind(&BBotLandingRepairController::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=====================================================");
        RCLCPP_INFO(this->get_logger(), "  BBot 跳跃与 LQR 复合控制器已启动 (200Hz)");
        RCLCPP_WARN(this->get_logger(),
                    "  EXPERIMENTAL LANDING REPAIR CONTROLLER: bounded -2 Nm/hip contact pulse=%s; baseline controller unchanged",
                    reference_ground_torque_pulse_enable_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(),
                    "[IMU_WORLD_FRAME] initial roll reference=%.6f rad; physical balance reference=%.6f rad",
                    sim_imu_reference_roll_, balance_offset_);
        RCLCPP_INFO(this->get_logger(), "  支持按键: J (跳跃), W/A/S/D (遥控), Q/E (变高度), R (起立), X (停机)");
        RCLCPP_INFO(this->get_logger(), "=====================================================");
    }

    ~BBotLandingRepairController()
    {
        if (jump_attempted_ && !jump_summary_written_)
        {
            if (current_state_ == bbot_jump::STATE_BALANCE && balance_return_time_ > 0.0)
                write_jump_summary(true, "");
            else
                write_jump_summary(false, jump_failure_reason_.empty() ? "进程退出或中断未完成全流程" : jump_failure_reason_);
        }
        close_log_files();
    }

private:
    int num_ = 0;
    // ── 状态机相关 ──
    bbot_jump::JumpState current_state_ = bbot_jump::STATE_BALANCE;
    double state_start_time_ = 0.0;
    double jump_cmd_rx_stamp_ = -1.0;
    double jump_cmd_accept_stamp_ = -1.0;
    bbot_jump::QuinticTrajectory quintic_traj_;

    // ── 传感器与里程计数据 ──
    bool imu_received_ = false;
    bool wheel_origin_set_ = false;
    bool was_moving_ = false;

    double pitch_ = 0.0;
    double imu_relative_pitch_ = 0.0;
    bool velocity_capture_enable_ = false;
    bool velocity_capture_reference_active_ = false;
    double velocity_capture_hip_decel_=0.0,velocity_capture_knee_decel_=0.0,velocity_capture_delay_=0.015;
    bbot_jump::VelocityCaptureSampleHistory velocity_capture_history_;
    bbot_jump::VelocityCaptureSession velocity_capture_session_;
    std::ofstream velocity_capture_log_;
    std::ofstream ground_input_log_;
    std::ofstream motion_trace_log_;
    std::ofstream phase_events_log_;
    bool thrust_momentum_reference_enable_ = false;
    bool thrust_support_coordination_enable_ = false;
    bool thrust_support_allocator_enable_ = false;
    bool ground_input_experiment_ = false;
    bool ground_motion_experiment_ = false;
    bool ground_motion_start_requested_ = false;
    bool ground_motion_started_ = false;
    bool ground_motion_failed_ = false;
    bool ground_motion_complete_ = false;
    bool ground_motion_fault_logged_ = false;
    bool ground_motion_law_active_ = false;
    bool ground_motion_guard_valid_ = false;
    double ground_motion_start_request_time_ = -1.0;
    double ground_motion_ready_elapsed_ = 0.0;
    double ground_motion_blend_start_time_ = -1.0;
    double ground_motion_blend_settle_start_time_ = -1.0;
    double ground_motion_trial_start_time_ = -1.0;
    double ground_motion_stop_start_time_ = -1.0;
    double ground_motion_stop_quiet_time_ = 0.0;
    int ground_motion_trial_id_ = 0;
    int ground_motion_trial_direction_ = 1;
    std::array<double, 4> ground_motion_anchor_{};
    std::array<double, 4> ground_motion_trial_anchor_{};
    std::array<double, 4> ground_motion_qref_{};
    std::array<double, 4> ground_motion_vref_{};
    std::array<double, 4> ground_motion_aref_{};
    std::array<double, 4> ground_motion_baseline_effort_{};
    std::array<double, 4> ground_motion_static_feedforward_{};
    std::array<double, 4> ground_motion_joint_feedback_{};
    double ground_motion_torso_static_ff_ = 0.0;
    std::array<int, 4> ground_motion_excitation_steps_{};
    double ground_motion_pitch_anchor_ = 0.0;
    double ground_motion_blend_alpha_ = 0.0;
    double ground_motion_anchor_error_max_ = 0.0;
    double ground_motion_joint_rate_max_ = 0.0;
    double ground_motion_soft_margin_min_ = 0.0;
    double ground_motion_torso_request_ = 0.0;
    std::string ground_motion_stage_ = "disabled";
    std::string ground_motion_reason_ = "disabled";
    std::string ground_motion_phase_ = "idle";
    uint64_t ground_motion_event_id_ = 0;
    struct GroundMotionEvent {
        int trial_id{0};
        int replay_id{0};
        std::string phase;
        std::string event;
        std::string reason;
        std::array<double, 4> q{};
    };
    std::vector<GroundMotionEvent> ground_motion_pending_events_;
    bool ground_input_requested_ = false;
    bool ground_input_preload_sent_ = false;
    double ground_input_stable_time_ = 0.0;
    std::string ground_input_stage_ = "disabled";
    std::string ground_input_reason_ = "disabled";
    double wheel_servo_gain_ = 1.0;
    double thrust_support_target_h_ = -0.08;
    bbot_jump::ThrustSupportAllocation thrust_support_allocation_diag_{};
    bbot_jump::ThrustSupportTasks thrust_support_tasks_diag_{};
    std::array<double, 4> thrust_support_actual_leg_command_diag_{};
    double thrust_support_actual_wheel_command_diag_ = 0.0;
    int thrust_support_allocator_guard_diag_ = 0;
    bool thrust_support_allocator_active_diag_ = false;
    bool thrust_support_allocator_legacy_active_diag_ = false;
    bool thrust_support_allocator_contact_hold_diag_ = false;
    double thrust_support_com_rel_forward_diag_ = 0.0;
    double thrust_support_com_rel_velocity_diag_ = 0.0;
    bbot_jump::ThrustSupportReference thrust_support_reference_diag_{};
    double thrust_support_imu_stamp_diag_ = 0.0;
    double thrust_support_joint_stamp_diag_ = 0.0;
    double thrust_support_com_stamp_diag_ = 0.0;
    double thrust_support_hip_servo_left_diag_ = 0.0;
    double thrust_support_hip_servo_right_diag_ = 0.0;
    bool thrust_reference_handoff_enable_ = false;
    bbot_jump::LandingReferenceHandoff thrust_reference_handoff_;
    bool thrust_reference_handoff_flight_done_ = false;
    int handoff_reference_mode_diag_ = 0;
    bool handoff_reference_valid_diag_ = false;
    bool handoff_reference_limited_diag_ = false;
    bool handoff_force_suppressed_diag_ = false;
    std::array<double, 4> handoff_reference_target_diag_{};
    std::array<double, 4> handoff_reference_applied_diag_{};
    std::array<double, 4> handoff_reference_rate_diag_{};
    bool thrust_momentum_reference_active_diag_ = false;
    double thrust_momentum_reference_blend_diag_ = 0.0;
    double thrust_momentum_reference_vz_diag_ = 0.0;
    double thrust_momentum_reference_h_diag_ = 0.0;
    double thrust_momentum_reference_hip_delta_diag_ = 0.0;
    double thrust_momentum_reference_knee_delta_diag_ = 0.0;
    double thrust_momentum_reference_hip_feedback_l_diag_ = 0.0;
    double thrust_momentum_reference_hip_feedback_r_diag_ = 0.0;
    bbot_jump::LandingJointHistory landing_joint_history_;
    bbot_jump::LandingMomentum landing_momentum_diag_;
    double landing_momentum_stamp_diag_ = -1.0;
    double sim_imu_reference_roll_ = 0.0;
    tf2::Quaternion imu_world_reference_{0.0, 0.0, 0.0, 1.0};
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
    bool balance_started_ = false;
    double balance_start_time_ = 0.0;
    bool pos_cmd_init_ = false;
    double last_q_hip_cmd_left_ = 0.0, last_q_knee_cmd_left_ = 0.0;
    double last_q_hip_cmd_right_ = 0.0, last_q_knee_cmd_right_ = 0.0;

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
    bool thrust_reference_seed_pending_ = false;
    double thrust_force_per_leg_ = 0.0;
    double thrust_force_command_per_leg_ = 0.0;
    double last_thrust_force_request_ = 0.0;
    double last_thrust_force_limit_ = 0.0;
    double last_tau_body_per_hip_ = 0.0;
    bbot_jump::JointVector ground_support_previous_ = bbot_jump::JointVector::Zero();
    bbot_jump::JointVector last_support_feedforward_ = bbot_jump::JointVector::Zero();
    bool ground_support_initialized_ = false;
    double ground_pd_horizon_ = 0.0;
    std::array<double, 4> ground_pd_feedback_{};
    bool last_wheels_airborne_ = false;
    double flight_ff_force_start_ = 0.0;
    bool flight_trajectory_initialized_ = false;
    // -1: 本次腾空尚未检查，0: 预算拒绝，1: 预算通过。
    int flight_tuck_plan_check_ = -1;
    int flight_extend_plan_check_ = -1;
    bbot_jump::FlightRoundTripPlan flight_round_trip_plan_;
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
    bbot_jump::StableDurationNs thrust_attitude_stable_duration_;
    double thrust_gate_pitch_err_diag_ = 0.0;
    double thrust_gate_rate_err_diag_ = 0.0;
    double thrust_gate_stable_elapsed_diag_ = 0.0;
    double thrust_gate_open_stamp_ = -1.0;
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
    bool native_state_diagnostics_=false;
    bool ground_contact_motion_experiment_ = false;
    bbot_jump::GroundEngineStateHistory ground_engine_history_;
    bbot_jump::NativeCommandState ground_engine_selected_;
    bbot_jump::AllocatorContactSnapshot ground_engine_contact_selected_;
    bool ground_engine_selected_valid_ = false;
    double ground_engine_pitch_anchor_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ground_engine_sub_;
    std::ofstream ground_engine_source_log_;
    int64_t capture_control_ns_=0;
    uint64_t command_publication_id_=0;
    int64_t last_command_publish_ns_ = -1;
    int64_t last_command_publish_end_ns_ = -1;
    int64_t last_command_wall_publish_ns_ = -1;
    int64_t last_command_wall_publish_end_ns_ = -1;
    bbot_jump::NativeCommandStateHistory native_state_history_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr native_state_sub_;
    std::ofstream native_state_log_,command_publication_log_;
    bbot_jump::AllocatorFlightPlan allocator_flight_plan_;
    bool allocator_flight_plan_attempted_ = false;
    double allocator_flight_pitch_reference_ = 0.0;
    double allocator_flight_previous_time_ = -1.0;
    std::ofstream allocator_flight_log_;
    std::array<double, 4> joint_velocity_cmd_{};
    double ground_height_offset_ = 0.0;
    bbot_jump::JointPoseHistory takeoff_joint_history_;
    bbot_jump::TakeoffConfirmation takeoff_confirmation_;
    bbot_jump::TakeoffSpeedLatch takeoff_speed_latch_;
    bbot_jump::CompleteContactTakeoffObserver contact_takeoff_observer_;
    bbot_jump::AllocatorContactHistory allocator_contact_history_;
    bbot_jump::GroundContactContinuityGuard ground_contact_stream_guard_;
    bbot_jump::AllocatorContactSnapshot thrust_support_contact_diag_;
    bool reference_ground_torque_pulse_enable_ = false;
    bool reference_ground_pulse_contact_valid_ = false;
    bool reference_ground_pulse_contact_continuous_ = false;
    bool reference_ground_pulse_have_previous_ = false;
    uint64_t reference_ground_pulse_sequence_ = 0;
    uint64_t reference_ground_pulse_iteration_ = 0;
    uint64_t reference_ground_pulse_previous_sequence_ = 0;
    uint64_t reference_ground_pulse_previous_iteration_ = 0;
    int64_t reference_ground_pulse_contact_stamp_ns_ = 0;
    int64_t reference_ground_pulse_previous_stamp_ns_ = 0;
    uint8_t reference_ground_pulse_wheel_mask_ = 0;
    int reference_ground_pulse_guard_diag_ =
        static_cast<int>(bbot_jump::ReferenceGroundPulseGuard::WrongPhase);
    double reference_ground_pulse_gate_elapsed_diag_ = 0.0;
    double reference_ground_pulse_shape_diag_ = 0.0;
    double reference_ground_pulse_requested_left_diag_ = 0.0;
    double reference_ground_pulse_requested_right_diag_ = 0.0;
    double reference_ground_pulse_applied_left_diag_ = 0.0;
    double reference_ground_pulse_applied_right_diag_ = 0.0;
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
    double capture_world_target_ = 0.0;
    bool capture_creep_anchor_latched_ = false;
    bool capture_anchor_active_ = false;
    double capture_v_ref_ = 0.0;
    bbot_jump::CentroidalBalanceState centroidal_balance_;
    bbot_jump::CentroidalLeanRateObserver centroidal_lean_rate_;
    double centroidal_legacy_lean_rate_ = 0.0;
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
    double air_wheel_sign_ = -1.0;
    double air_wheel_linear_speed_limit_ = 2.0;
    double air_wheel_extend_kd_ = 2.50;
    double air_wheel_tuck_kd_ = 0.45;
    double air_wheel_cmd_raw_ = 0.0;
    double flight_arrest_freewheel_duration_ = 0.0;
    bool flight_arrest_freewheel_active_ = false;
    bool thrust_terminal_hip_floor_ = false;
    double thrust_hip_requested_left_diag_ = 0.0;
    double thrust_hip_requested_right_diag_ = 0.0;
    bool thrust_hip_floor_applied_diag_ = false;

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
    double buffer_force_per_leg_ = 0.0; // 触地缓冲滤波支撑力 [N]
    double height_force_per_leg_ = 0.0; // 恢复/稳态单腿滤波支撑力 [N]
    bool height_force_initialized_ = false;
    int recovery_stable_count_ = 0;                // 恢复连续稳定计数
    bool post_landing_balance_soft_start_ = false; // 落地后平衡软接管标志
    bool post_landing_gyro_reduced_ = false;       // 落地后陀螺仪增益缩放标志
    bool post_landing_effort_support_ = false;     // 跳后保持力矩控制模式
    bool pre_jump_effort_capture_ = false;         // 连续起跳沿用落地后的承重捕获路径
    bool effort_support_handoff_preserved_ = false;
    bool pre_jump_capture_armed_ = false;
    double pre_jump_capture_ready_timer_ = 0.0;
    double pre_jump_roll_in_elapsed_ = 0.0;
    std::string jump_reject_reason_diag_;
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
    double landing_fall_start_time_ = -1.0;
    double landing_support_start_time_ = -1.0;
    bool touchdown_catch_active_ = true;
    bool touchdown_reverse_brake_active_ = false;
    bool touchdown_reverse_brake_consumed_ = false;
    int touchdown_catch_stable_count_ = 0;
    double touchdown_settle_start_time_ = -1.0; // CATCH 释放后 PREPARE/BRAKE 阶段计时起点
    bool touchdown_brake_active_ = false;       // false: PREPARE_BRAKE, true: BRAKE
    int touchdown_brake_ready_count_ = 0;
    double touchdown_brake_start_time_ = -1.0;
    double touchdown_brake_cmd_ref_ = 0.0; // BRAKE 阶段逐步下降的后退轮速目标幅值
    bool touchdown_torso_convergence_latched_ = false;
    double touchdown_torso_convergence_start_time_ = -1.0;
    double touchdown_torso_convergence_blend_ = 0.0;
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

    // 落地后恢复与低速保持轮控分解在线诊断
    double wheel_pitch_err_diag_ = 0.0;
    double wheel_balance_rate_diag_ = 0.0;
    double wheel_k_theta_diag_ = 0.0;
    double wheel_k_theta_dot_diag_ = 0.0;
    double wheel_control_height_diag_ = 0.0;
    double wheel_p_term_diag_ = 0.0;
    double wheel_d_term_diag_ = 0.0;
    double wheel_attitude_cmd_diag_ = 0.0;
    double wheel_vel_error_diag_ = 0.0;
    double wheel_vel_term_diag_ = 0.0;
    double wheel_raw_pos_err_diag_ = 0.0;
    double wheel_pos_error_diag_ = 0.0;
    double wheel_pos_term_diag_ = 0.0;
    double wheel_cmd_raw_diag_ = 0.0;
    double wheel_cmd_pre_clamp_diag_ = 0.0;
    double wheel_min_fwd_cmd_diag_ = 0.0;
    double wheel_max_bwd_cmd_diag_ = 0.0;
    double wheel_cmd_target_diag_ = 0.0;
    bool initial_balance_wheel_slew_active_diag_ = false;
    double initial_balance_wheel_raw_target_diag_ = 0.0;
    double initial_balance_wheel_applied_cmd_diag_ = 0.0;

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
    double jump_forward_speed_ = 0.30;         // PRE_JUMP / SQUAT 接近速度 [m/s]
    double jump_takeoff_forward_speed_ = 0.45; // THRUST 离地前向速度目标 [m/s]
    double thrust_forward_velocity_kp_ = 1.20; // THRUST 前向速度误差补偿
    bool thrust_forward_attitude_taper_enable_ = true;
    bool thrust_release_fast_rate_correction_enable_ = true;
    bool thrust_fast_rate_correction_from_gate_ = true;
    double thrust_release_velocity_ratio_ = 0.78;
    bool thrust_forward_speed_prediction_enable_ = true;
    bool thrust_wheel_kinematics_compensation_enable_ = true;
    bool complete_contact_takeoff_confirmation_enable_ = true;
    bool arrest_dynamics_feedforward_enable_ = true;
    bool arrest_dynamics_feedforward_scope_diag_ = false;
    int arrest_dynamics_feedforward_guard_diag_ = 0;
    double arrest_dynamics_feedforward_blend_diag_ = 0.0;
    std::array<double, 4> arrest_dynamics_qdd_diag_{};
    std::array<double, 4> arrest_dynamics_inertial_diag_{};
    std::array<double, 4> arrest_dynamics_bias_diag_{};
    std::array<double, 4> arrest_dynamics_raw_diag_{};
    std::array<double, 4> arrest_dynamics_bounded_diag_{};
    std::array<double, 4> arrest_dynamics_applied_diag_{};
    bool contact_takeoff_confirmed_diag_ = false;
    bbot_jump::ThrustForwardSpeedPredictor thrust_forward_speed_predictor_;
    bbot_jump::ThrustWheelKinematicsObserver thrust_wheel_kinematics_observer_;
    bool thrust_release_fast_rate_correction_active_diag_ = false;
    bool thrust_fast_rate_correction_scope_diag_ = false;
    double thrust_fast_rate_correction_blend_diag_ = 0.0;
    double thrust_release_fast_rate_correction_requested_diag_ = 0.0;
    double thrust_release_fast_rate_correction_applied_diag_ = 0.0;
    Eigen::Vector2d jump_forward_axis_world_ = Eigen::Vector2d::Zero();
    bool jump_forward_axis_valid_ = false;
    double thrust_wheel_max_decel_ = 16.0;     // THRUST 轮速制动/减速斜率上限 [m/s^2]
    double jump_pitch_offset_ = 0.020;
    double jump_pitch_ref_ = 0.058;
    double active_jump_pitch_ref_ = 0.038;
    double jump_takeoff_pitch_rate_ = 0.05; // 正值=继续向前旋转 [rad/s]
    double jump_takeoff_pitch_rate_tolerance_ = 0.18;
    double thrust_pitch_rate_lead_time_ = 0.07;
    double active_jump_pitch_rate_ref_ = 0.0;
    double thrust_fwd_term_diag_ = 0.0;
    double thrust_att_term_diag_ = 0.0;
    double thrust_att_term_raw_diag_ = 0.0;
    double thrust_attitude_scale_diag_ = 1.0;
    double thrust_wheel_target_diag_ = 0.0;
    double thrust_cmd_x_diag_ = 0.0;
    double thrust_forward_speed_raw_diag_ = 0.0;
    double thrust_forward_speed_base_diag_ = 0.0;
    double thrust_forward_speed_predicted_diag_ = 0.0;
    double thrust_forward_speed_accel_diag_ = 0.0;
    double thrust_forward_speed_horizon_diag_ = 0.0;
    double thrust_forward_speed_delta_diag_ = 0.0;
    bool thrust_forward_speed_prediction_active_diag_ = false;
    double thrust_kinematics_stamp_diag_ = -1.0;
    double thrust_kinematics_age_diag_ = 0.0;
    double thrust_kinematics_dt_diag_ = 0.0;
    double thrust_kinematics_r_dot_diag_ = 0.0;
    double thrust_kinematics_shank_projection_diag_ = 0.0;
    double thrust_kinematics_raw_correction_diag_ = 0.0;
    double thrust_kinematics_applied_correction_diag_ = 0.0;
    bool thrust_kinematics_valid_diag_ = false;
    std::array<double, 4> thrust_kinematics_q_diag_{};
    Eigen::Matrix3d thrust_kinematics_rotation_diag_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d thrust_kinematics_relative_body_diag_ = Eigen::Vector3d::Zero();
    double thrust_com_sample_stamp_diag_ = 0.0;
    int thrust_com_valid_diag_ = 0;

    // 空中落点规划。inverse_kinematics 的 target_x>0 表示“机身在轮子前方”，
    // 即轮子相对机身后移。速度 capture 项只会减少这部分后移量。
    double landing_wheel_back_bias_ = 0.085;
    double landing_capture_gain_ = 1.60;
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
    double landing_capture_offset_ = 0.0; // 速度导致的前置修正量
    double landing_target_x_ = 0.0;       // 真正传给 IK 的第三参数
    double landing_capture_comp_ = 0.0;   // 仅诊断：atan2(target_x, L_TOUCH)
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
    double flight_hip_speed_limit_ = 16.0;
    double flight_knee_speed_limit_ = 20.0;
    double flight_hip_acc_limit_ = 450.0;
    double flight_knee_acc_limit_ = 500.0;
    double flight_hip_pos_limit_ = 1.52;
    double flight_knee_pos_limit_ = 1.5708;

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
    double squat_duration_current_ = 0.50;
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
    double thrust_knee_velocity_limit_ = 30.0;
    bbot_jump::ThrustVelocityReference thrust_velocity_reference_;
    double thrust_torque_margin_ = 0.95;
    bool sim_relax_thrust_limits_ = false;
    bbot_jump::JointEffortLimits active_effort_limits_{75.0, 60.0};
    std::array<double, 4> ground_gravity_torque_{};
    double thrust_force_unlimited_request_ = 0.0;
    double thrust_knee_pd_left_ = 0.0;
    bool last_thrust_position_yield_ = false;
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
    std::string jump_log_path_;
    std::string jump_summary_path_;
    bool auto_return_balance_ = true;
    bool jump_attempted_ = false;
    double takeoff_com_z_ = 0.0;
    double takeoff_com_vz_ = 0.0;
    double takeoff_com_vx_ = 0.0;
    double com_takeoff_latched_vz_ = 0.0;
    double com_takeoff_confirmed_vz_ = 0.0;
    double thrust_start_com_z_ = 0.0;
    double max_com_z_during_jump_ = 0.0;
    bool tuck_entered_ = false;
    bool extend_entered_ = false;
    std::string protective_reason_;
    double touchdown_time_ = -1.0;
    double recovery_time_ = -1.0;
    double balance_return_time_ = -1.0;
    std::string exit_code_ = "NOT_STARTED";
    double post_jump_balance_stable_timer_ = 0.0;

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
    double touchdown_height_ = 0.0; // 触地瞬间实测机身高度，缓冲高度参考的起点
    double touchdown_buffer_target_height_ = 0.34;
    double touchdown_buffer_settle_duration_ = 0.18;
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ground_input_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_frame_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_effort_pub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_client_;
    bool effort_mode_active_ = false;
    bool leg_mode_switch_pending_ = false;
    bool thrust_effort_owned_ = false;
    bool thrust_output_locked_ = false;
    double effort_switch_request_stamp_ = -1.0;
    double effort_switch_ack_stamp_ = -1.0;
    // -1: no request, 0: pending, 1: accepted, 2: rejected, 3: callback error.
    int effort_switch_result_ = -1;
    rclcpp::TimerBase::SharedPtr timer_;

    KeyboardReader keyboard_;
    bbot_kinematics::Kinematics kinematics_;
    rclcpp::Time last_time_;
    // Observation only: relate the 5 ms wall timer to accepted simulation-clock updates.
    std::chrono::steady_clock::time_point last_control_wall_time_{};
    bool control_wall_time_valid_ = false;
    uint64_t timer_calls_since_control_ = 0;
    uint64_t timer_calls_for_update_diag_ = 0;
    double control_wall_dt_ms_diag_ = -1.0;
    double control_sim_dt_ms_diag_ = 0.0;
    rclcpp::Time start_time_;
    double control_log_timestamp_sec_ = -1.0;

    std::string data_path_;
    std::ofstream jump_log_file_;

    // ── 模式与跳跃触发 ──
    void trigger_jump()
    {
        if (!bbot_jump::ground_input_jump_allowed(ground_input_experiment_))
        {
            jump_reject_reason_diag_ = "ground_input_experiment_disables_jump";
            RCLCPP_WARN(this->get_logger(), "[ground_input] 跳跃请求拒绝：实验模式只允许站立输入采集");
            return;
        }
        if (current_state_ != bbot_jump::STATE_BALANCE)
        {
            jump_reject_reason_diag_ = std::string("当前状态不允许起跳: ") +
                bbot_jump::state_to_string(current_state_);
            RCLCPP_WARN(this->get_logger(), "[跳跃请求忽略] 当前不在 BALANCE 自平衡状态 (当前: %s)",
                        bbot_jump::state_to_string(current_state_));
            return;
        }

        if (!odom_received_ || this->now().seconds() - last_world_odom_time_ > 0.10)
        {
            jump_reject_reason_diag_ = "世界里程计未就绪或过期";
            RCLCPP_WARN(this->get_logger(), "[跳跃请求忽略] 世界里程计未就绪或过期，无法判定离地/触地");
            return;
        }

        double now_sec = this->now().seconds();
        if (!centroidal_velocity_valid_ || !capture_world_valid_ ||
            !centroidal_lean_rate_.valid(now_sec) ||
            !centroidal_balance_.valid || (now_sec - centroidal_height_.stamp() > 0.080))
        {
            jump_reject_reason_diag_ = "COM或对齐角速度观测无效/过期";
            RCLCPP_WARN(this->get_logger(),
                        "[跳跃请求忽略] COM质心观测器未就绪或过期 "
                        "(vertical=%d horizontal=%d balance=%d age=%.3f s)，禁止起跳",
                        centroidal_velocity_valid_ ? 1 : 0,
                        capture_world_valid_ ? 1 : 0,
                        centroidal_balance_.valid ? 1 : 0,
                        now_sec - centroidal_height_.stamp());
            return;
        }

        // PRE_JUMP 会主动建立前向速度与前倾工作点；触发瞬间只拒绝已经明显失稳的状态。
        const bool takeoff_safe =
            std::abs(pitch_ - balance_offset_) < 0.25 &&
            std::abs(pitch_rate_) < 2.0 && std::abs(x_dot_) < 0.30;
        if (!takeoff_safe)
        {
            jump_reject_reason_diag_ = "起跳前姿态/速度未站稳";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[跳跃请求忽略] 触发瞬间运动过大: pitch=%.3f gyro=%.3f xdot=%.3f",
                                 pitch_, pitch_rate_, x_dot_);
            return;
        }

        jump_reject_reason_diag_.clear();
        // A prior jump's handoff lock cannot be cleared by callbacks or by
        // returning to BALANCE; only a newly accepted jump reaches here.
        thrust_effort_owned_ = false;
        thrust_output_locked_ = false;

        Eigen::Vector3d jump_heading_world = odom_body_rotation_.col(1);
        jump_heading_world.z() = 0.0;
        jump_forward_axis_world_.setZero();
        jump_forward_axis_valid_ = false;
        if (jump_heading_world.norm() > 0.5)
        {
            jump_forward_axis_world_ = jump_heading_world.head<2>().normalized();
            jump_forward_axis_valid_ = true;
        }

        const bool continuing_effort_support = post_landing_effort_support_ && effort_mode_active_;
        jump_pitch_ref_ = balance_offset_ + jump_pitch_offset_;
        active_jump_pitch_ref_ = balance_offset_;
        active_jump_pitch_rate_ref_ = 0.0;
        pre_jump_hold_height_ = bbot_jump::clamp_value(current_z_, L_SQUAT_, L_MAX_);
        pre_jump_stable_timer_ = 0.0;
        pre_jump_effort_capture_ = continuing_effort_support;
        effort_support_handoff_preserved_ = false;
        pre_jump_capture_armed_ = !continuing_effort_support;
        pre_jump_capture_ready_timer_ = 0.0;
        pre_jump_roll_in_elapsed_ = 0.0;
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
        thrust_forward_speed_predictor_.reset();
        thrust_forward_speed_raw_diag_ = 0.0;
        thrust_forward_speed_base_diag_ = 0.0;
        thrust_forward_speed_predicted_diag_ = 0.0;
        thrust_forward_speed_accel_diag_ = 0.0;
        thrust_forward_speed_horizon_diag_ = 0.0;
        thrust_forward_speed_delta_diag_ = 0.0;
        thrust_forward_speed_prediction_active_diag_ = false;
        thrust_wheel_kinematics_observer_.reset();
        thrust_kinematics_stamp_diag_ = -1.0;
        thrust_kinematics_age_diag_ = 0.0;
        thrust_kinematics_dt_diag_ = 0.0;
        thrust_kinematics_r_dot_diag_ = 0.0;
        arrest_dynamics_feedforward_scope_diag_ = false;
        arrest_dynamics_feedforward_guard_diag_ =
            static_cast<int>(bbot_jump::ArrestDynamicsGuard::DisabledOrWrongPhase);
        arrest_dynamics_feedforward_blend_diag_ = 0.0;
        arrest_dynamics_qdd_diag_.fill(0.0);
        arrest_dynamics_inertial_diag_.fill(0.0);
        arrest_dynamics_bias_diag_.fill(0.0);
        arrest_dynamics_raw_diag_.fill(0.0);
        arrest_dynamics_bounded_diag_.fill(0.0);
        arrest_dynamics_applied_diag_.fill(0.0);
        thrust_kinematics_shank_projection_diag_ = 0.0;
        thrust_kinematics_raw_correction_diag_ = 0.0;
        thrust_kinematics_applied_correction_diag_ = 0.0;
        thrust_kinematics_valid_diag_ = false;
        thrust_kinematics_q_diag_ = {};
        thrust_kinematics_rotation_diag_.setIdentity();
        thrust_kinematics_relative_body_diag_.setZero();
        protective_deploy_rate_limited_ = false;
        protective_deploy_replan_count_ = 0;

        jump_attempted_ = true;
        jump_summary_written_ = false;
        tuck_entered_ = false;
        extend_entered_ = false;
        protective_reason_.clear();
        touchdown_time_ = -1.0;
        recovery_time_ = -1.0;
        balance_return_time_ = -1.0;
        exit_code_ = "IN_PROGRESS";
        com_takeoff_latched_vz_ = 0.0;
        com_takeoff_confirmed_vz_ = 0.0;
        takeoff_com_z_ = 0.0;
        takeoff_com_vz_ = 0.0;
        takeoff_com_vx_ = 0.0;
        takeoff_speed_latch_.reset();
        thrust_start_com_z_ = centroidal_height_.height();
        max_com_z_during_jump_ = thrust_start_com_z_;
        post_jump_balance_stable_timer_ = 0.0;
        touchdown_torso_convergence_latched_ = false;
        touchdown_torso_convergence_start_time_ = -1.0;
        touchdown_torso_convergence_blend_ = 0.0;

        RCLCPP_INFO(this->get_logger(),
                    ">>> 收到跳跃指令！PRE_JUMP 建立 vx=%.3f；SQUAT 末段建立 pitch_rate=%.3f；"
                    "THRUST 目标 vx=%.3f m/s，pitch 从 %.3f 推向 %.3f rad <<<",
                    jump_forward_speed_, jump_takeoff_pitch_rate_, jump_takeoff_forward_speed_,
                    balance_offset_, jump_pitch_ref_);
        jump_cmd_accept_stamp_ = now_sec;
        current_state_ = bbot_jump::STATE_PRE_JUMP;
        state_start_time_ = now_sec;
        protective_landing_ = false;
        // Effort support is deliberately retained across PRE_JUMP/SQUAT for a
        // second hop.  The shared ground allocator then preserves gravity,
        // discrete leg feedback, torso allocation, and support slew limiting.
        if (!continuing_effort_support)
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
        touchdown_reverse_brake_active_ = false;
        touchdown_reverse_brake_consumed_ = false;
        touchdown_confirmation_.reset();
        flight_ff_force_start_ = 0.0;
        flight_trajectory_initialized_ = false;
        allocator_flight_plan_ = {};
        allocator_flight_plan_attempted_ = false;
        allocator_flight_previous_time_ = -1.0;
        flight_tuck_plan_check_ = -1;
        flight_extend_plan_check_ = -1;
        flight_round_trip_plan_ = {};
        effort_slew_initialized_ = false;
        airborne_confidence_count_ = 0;
        thrust_release_.reset();
        touchdown_catch_stable_time_ = 0.0;
        thrust_motion_elapsed_ = 0.0;
        thrust_attitude_blocked_ = false;
        thrust_gate_has_opened_ = false;
        thrust_attitude_stable_duration_.reset();
        thrust_gate_pitch_err_diag_ = 0.0;
        thrust_gate_rate_err_diag_ = 0.0;
        thrust_gate_stable_elapsed_diag_ = 0.0;
        thrust_gate_open_stamp_ = -1.0;
        effort_switch_request_stamp_ = -1.0;
        effort_switch_ack_stamp_ = -1.0;
        effort_switch_result_ = -1;
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
        thrust_reference_handoff_.reset();
        velocity_capture_session_.reset();
        thrust_reference_handoff_flight_done_ = false;
        jump_failure_reason_.clear();
        ++jump_id_;
        contact_takeoff_observer_.reset_for_jump(
            jump_id_, static_cast<int64_t>(std::llround(now_sec * 1.0e9)));

        // PRE_JUMP 从当前位置开始积分移动参考，避免继承 BALANCE 的旧位置误差。
        target_speed_const_ = jump_forward_speed_;
        target_speed_smoothed_ = x_dot_;
        target_yaw_rate_ = 0.0;
        target_x_ = x_;
        was_moving_ = true;
        vel_integral_ = 0.0;
    }

    void handle_mode_command(const std::string &cmd)
    {
        if (cmd == "jump" || cmd == "j" || cmd == "J")
        {
            trigger_jump();
        }
        else if (cmd == "standup" || cmd == "r" || cmd == "R")
        {
            current_state_ = bbot_jump::STATE_STANDUP;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
        }
        else if (cmd == "emergency" || cmd == "x" || cmd == "X")
        {
            current_state_ = bbot_jump::STATE_EMERGENCY;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
        }
        else if (cmd == "balance")
        {
            current_state_ = bbot_jump::STATE_BALANCE;
        }
    }

    void process_keyboard()
    {
        std::string seq = keyboard_.read_sequence();
        if (seq.empty())
            return;

        if (seq == "j" || seq == "J")
        {
            trigger_jump();
        }
        else if (seq == "w" || seq == "W")
        {
            target_speed_const_ = walk_speed_;
            target_yaw_rate_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 前进  speed=%.2f", target_speed_const_);
        }
        else if (seq == "s" || seq == "S")
        {
            target_speed_const_ = -walk_speed_;
            target_yaw_rate_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 后退  speed=%.2f", target_speed_const_);
        }
        else if (seq == "a" || seq == "A")
        {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = turn_speed_;
            RCLCPP_INFO(this->get_logger(), "[键盘] 左转  yaw=%.2f", target_yaw_rate_);
        }
        else if (seq == "d" || seq == "D")
        {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = -turn_speed_;
            RCLCPP_INFO(this->get_logger(), "[键盘] 右转  yaw=%.2f", target_yaw_rate_);
        }
        else if (seq == " ")
        {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            RCLCPP_INFO(this->get_logger(), "[键盘] 刹车停止");
        }
        else if (seq == "q" || seq == "Q")
        {
            if (current_state_ == bbot_jump::STATE_BALANCE)
            {
                target_height_ = bbot_jump::clamp_value(target_height_ + 0.01, L_MIN_, L_MAX_);
                RCLCPP_INFO(this->get_logger(), "[键盘] 升高  目标高度 → %.3f m", target_height_);
            }
        }
        else if (seq == "e" || seq == "E")
        {
            if (current_state_ == bbot_jump::STATE_BALANCE)
            {
                target_height_ = bbot_jump::clamp_value(target_height_ - 0.01, L_MIN_, L_MAX_);
                RCLCPP_INFO(this->get_logger(), "[键盘] 降低  目标高度 → %.3f m", target_height_);
            }
        }
        else if (seq == "r" || seq == "R")
        {
            current_state_ = bbot_jump::STATE_STANDUP;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 触发自适应起立恢复模式！");
        }
        else if (seq == "x" || seq == "X")
        {
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
        imu_relative_pitch_ = -roll;
        // Gazebo IMU orientation is relative to its initial orientation.
        // Compose with the spawn reference before using CAD world geometry.
        q = imu_world_reference_ * q;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 机器人 CAD 约定：前倾对应负 roll，取负号使前倾为正
        pitch_ = -roll;
        pitch_rate_raw_ = -msg->angular_velocity.x;
        if (velocity_capture_enable_)
            velocity_capture_history_.imu(rclcpp::Time(msg->header.stamp).seconds(),pitch_,pitch_rate_raw_);
        torso_imu_.update(rclcpp::Time(msg->header.stamp).seconds(), pitch_rate_raw_,
                          msg->linear_acceleration.y, msg->linear_acceleration.z);

        if (!pitch_rate_filter_init_)
        {
            pitch_rate_filt_ = pitch_rate_raw_;
            pitch_rate_filter_init_ = true;
        }
        else
        {
            pitch_rate_filt_ = bbot_jump::low_pass_filter(pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
        }
        pitch_rate_ = pitch_rate_filt_;

        acc_z_raw_ = msg->linear_acceleration.z;
        if (!acc_z_filter_init_)
        {
            acc_z_filt_ = acc_z_raw_;
            acc_z_filter_init_ = true;
        }
        else
        {
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
        if (joint_sample_time_ >= 0.0 && stamp > joint_sample_time_)
        {
            joint_sample_period_ = bbot_jump::low_pass_filter(
                bbot_jump::clamp_value(stamp - joint_sample_time_, 0.005, 0.030),
                joint_sample_period_, 0.2);
        }
        joint_sample_time_ = stamp;
        bool has_left = false, has_right = false;
        std::array<bool, 4> has_leg_position{};
        std::array<bool, 6> has_momentum_velocity{};
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "link_004_joint")
            {
                if (i < msg->position.size())
                    left_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size()) {
                    left_wheel_vel_ = msg->velocity[i];
                    has_momentum_velocity[4] = true;
                }
                has_left = true;
            }
            else if (msg->name[i] == "link_007_joint")
            {
                if (i < msg->position.size())
                    right_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size()) {
                    right_wheel_vel_ = msg->velocity[i];
                    has_momentum_velocity[5] = true;
                }
                has_right = true;
            }
            else if (msg->name[i] == "link_002_joint")
            {
                if (i < msg->position.size())
                {
                    hip_pos_left_ = msg->position[i];
                    has_leg_position[0] = true;
                }
                if (i < msg->velocity.size()) {
                    hip_vel_left_ = msg->velocity[i];
                    has_momentum_velocity[0] = true;
                }
                if (i < msg->effort.size())
                    hip_effort_left_ = msg->effort[i];
            }
            else if (msg->name[i] == "link_003_joint")
            {
                if (i < msg->position.size())
                {
                    knee_pos_left_ = msg->position[i];
                    has_leg_position[1] = true;
                }
                if (i < msg->velocity.size()) {
                    knee_vel_left_ = msg->velocity[i];
                    has_momentum_velocity[1] = true;
                }
                if (i < msg->effort.size())
                    knee_effort_left_ = msg->effort[i];
            }
            else if (msg->name[i] == "link_005_joint")
            {
                if (i < msg->position.size())
                {
                    hip_pos_right_ = msg->position[i];
                    has_leg_position[2] = true;
                }
                if (i < msg->velocity.size()) {
                    hip_vel_right_ = msg->velocity[i];
                    has_momentum_velocity[2] = true;
                }
                if (i < msg->effort.size())
                    hip_effort_right_ = msg->effort[i];
            }
            else if (msg->name[i] == "link_006_joint")
            {
                if (i < msg->position.size())
                {
                    knee_pos_right_ = msg->position[i];
                    has_leg_position[3] = true;
                }
                if (i < msg->velocity.size()) {
                    knee_vel_right_ = msg->velocity[i];
                    has_momentum_velocity[3] = true;
                }
                if (i < msg->effort.size())
                    knee_effort_right_ = msg->effort[i];
            }
        }

        if (std::all_of(has_leg_position.begin(), has_leg_position.end(), [](bool value)
                        { return value; }))
        {
            // 零时间戳不假装同步；仅真实消息时间戳可进入几何历史。
            if (sample_time > 0.0)
                takeoff_joint_history_.push(sample_time,
                                            {hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_});
        }

        if (sample_time > 0.0 &&
            std::all_of(has_leg_position.begin(), has_leg_position.end(), [](bool b) { return b; }) &&
            std::all_of(has_momentum_velocity.begin(), has_momentum_velocity.end(), [](bool b) { return b; }))
        {
            const bbot_jump::LandingJointSample capture_sample{sample_time,
                bbot_jump::JointVector(hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_),
                bbot_jump::JointVector(hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_),
                {left_wheel_vel_, right_wheel_vel_}};
            landing_joint_history_.push(capture_sample);
            if (velocity_capture_enable_) velocity_capture_history_.joints(capture_sample);
        }

        if (has_left && has_right)
        {
            if (!wheel_origin_set_)
            {
                left_wheel_pos_origin_ = left_wheel_pos_;
                right_wheel_pos_origin_ = right_wheel_pos_;
                wheel_origin_set_ = true;
                prev_z_time_ = this->now();
            }
            x_dot_raw_ = -wheel_radius_ * 0.5 * (left_wheel_vel_ + right_wheel_vel_);
            if (!x_dot_filter_init_)
            {
                x_dot_filt_ = x_dot_raw_;
                x_dot_filter_init_ = true;
            }
            else
            {
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
        rclcpp::Time now_t = sample_time > 0.0 ? rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) : this->now();
        double dt_z = (now_t - prev_z_time_).seconds();
        if (dt_z > 0.001)
        {
            current_z_dot_raw_ = (z_calc - prev_z_) / dt_z;
            current_z_dot_raw_ = bbot_jump::clamp_value(current_z_dot_raw_, -3.0, 3.0);
            if (!z_dot_filter_init_)
            {
                current_z_dot_ = current_z_dot_raw_;
                z_dot_filter_init_ = true;
            }
            else
            {
                current_z_dot_ = bbot_jump::low_pass_filter(
                    current_z_dot_raw_, current_z_dot_, 0.22);
            }
            prev_z_ = z_calc;
            prev_z_time_ = now_t;
        }
        current_z_ = z_calc;
    }

    // ── 主控制循环 (200Hz) ──
    void record_reference_ground_pulse_contact_frame(
        const bbot_jump::CompleteGroundContactFrame &frame)
    {
        const bool takeover_active = ground_input_preload_sent_ || ground_motion_started_;
        const bool contact_stream_ok = ground_contact_stream_guard_.observe(
            frame.sequence, frame.sim_time_ns, frame.iteration, frame.dt_ns,
            frame.decoded && frame.frame_valid && frame.collision_pairs_allowed,
            frame.wheel_mask == 0x3, takeover_active);
        if (!contact_stream_ok && ground_contact_stream_guard_.failed()) {
            const std::string reason = ground_contact_stream_guard_.reason();
            if (ground_motion_started_ && !ground_motion_complete_) {
                fail_ground_motion(reason);
            } else if (ground_input_preload_sent_ &&
                       (ground_input_stage_ == "switching" ||
                        ground_input_stage_ == "effort_hold")) {
                ground_input_stage_ = "failed";
                ground_input_reason_ = reason;
                RCLCPP_ERROR(this->get_logger(),
                             "[ground_input] stage=failed reason=%s",
                             reason.c_str());
            }
        }
        if (thrust_support_allocator_enable_ || ground_input_experiment_)
            allocator_contact_history_.push(frame);
        const bool semantically_valid = frame.decoded && frame.frame_valid &&
                                        frame.collision_pairs_allowed;
        if (!semantically_valid)
        {
            reference_ground_pulse_contact_valid_ = false;
            reference_ground_pulse_contact_continuous_ = false;
            reference_ground_pulse_have_previous_ = false;
            reference_ground_pulse_contact_stamp_ns_ = 0;
            reference_ground_pulse_wheel_mask_ = 0;
            return;
        }
        const bool contiguous = reference_ground_pulse_have_previous_ &&
            frame.sequence == reference_ground_pulse_previous_sequence_ + 1 &&
            frame.iteration == reference_ground_pulse_previous_iteration_ + 1 &&
            frame.sim_time_ns - reference_ground_pulse_previous_stamp_ns_ ==
                bbot_jump::CompleteGroundContactWire::kExpectedDtNs &&
            frame.dt_ns == bbot_jump::CompleteGroundContactWire::kExpectedDtNs;
        reference_ground_pulse_contact_valid_ = true;
        reference_ground_pulse_contact_continuous_ = contiguous;
        reference_ground_pulse_sequence_ = frame.sequence;
        reference_ground_pulse_iteration_ = frame.iteration;
        reference_ground_pulse_contact_stamp_ns_ = frame.sim_time_ns;
        reference_ground_pulse_wheel_mask_ = frame.wheel_mask;
        reference_ground_pulse_previous_sequence_ = frame.sequence;
        reference_ground_pulse_previous_iteration_ = frame.iteration;
        reference_ground_pulse_previous_stamp_ns_ = frame.sim_time_ns;
        reference_ground_pulse_have_previous_ = true;
    }

    bool ground_input_entry_ready(double now_sec, std::string &reason)
    {
        bbot_jump::AllocatorContactSnapshot contact;
        const bool bilateral = allocator_contact_history_.snapshot(
            this->now().nanoseconds(), contact) && contact.wheel_mask == 0x3;
        const bool fresh_state = imu_received_ && odom_received_ && takeoff_odom_pose_valid_ &&
            capture_world_valid_ && centroidal_balance_.valid &&
            centroidal_lean_rate_.valid(now_sec) && centroidal_velocity_valid_ &&
            now_sec >= joint_sample_time_ && now_sec - joint_sample_time_ <= 0.020 &&
            torso_imu_.fresh(now_sec) && centroidal_height_.valid(now_sec);
        if (!fresh_state) { reason = "waiting_fresh_imu_joint_odom_com"; return false; }
        if (!bilateral) { reason = "waiting_continuous_bilateral_wheel_contact"; return false; }
        bool quiet = std::abs(pitch_ - balance_offset_) < 0.04 &&
            std::abs(pitch_rate_) < 0.15 && std::abs(ground_balance_angle()) < 0.10 &&
            std::abs(ground_balance_rate()) < 0.35 && std::abs(x_dot_) < 0.08 &&
            std::abs(capture_com_velocity_) < 0.08 &&
            std::abs(centroidal_height_.velocity()) < 0.03 &&
            std::abs(current_z_dot_) < 0.03;
        if (!quiet) { reason = "waiting_stable_stand_state"; return false; }
        reason = "stable_stand_bilateral_contact";
        return true;
    }

    bool ground_input_support_fresh(double now_sec)
    {
        bbot_jump::AllocatorContactSnapshot contact;
        const bool bilateral = allocator_contact_history_.snapshot(
            this->now().nanoseconds(), contact) && contact.wheel_mask == 0x3;
        const bool fresh_state = imu_received_ && odom_received_ && takeoff_odom_pose_valid_ &&
            capture_world_valid_ && centroidal_balance_.valid &&
            centroidal_lean_rate_.valid(now_sec) && centroidal_velocity_valid_ &&
            now_sec >= joint_sample_time_ && now_sec - joint_sample_time_ <= 0.020 &&
            torso_imu_.fresh(now_sec) && centroidal_height_.valid(now_sec);
        return bbot_jump::ground_input_effort_hold_valid(
            effort_mode_active_, leg_mode_switch_pending_, fresh_state, bilateral);
    }

    bool ground_motion_law_active() const
    {
        return ground_motion_experiment_ && ground_motion_started_ &&
               !ground_motion_failed_;
    }

    bool ground_input_has_protected_outputs() const
    {
        // Failure changes the stage name, but does not relinquish the
        // preloaded/held actuators. Early sensor/clock exits must still stop
        // wheels and preserve bounded support after a callback marks failure.
        return ground_input_experiment_ && ground_input_preload_sent_;
    }

    void queue_ground_motion_event(const std::string &event,
                                   const std::string &reason)
    {
        GroundMotionEvent pending;
        pending.trial_id = ground_motion_trial_id_;
        pending.replay_id = ground_motion_trial_id_ > 0 ?
            (ground_motion_trial_id_ + 1) / 2 : 0;
        pending.phase = ground_motion_phase_;
        pending.event = event;
        pending.reason = reason;
        pending.q = ground_motion_qref_;
        ground_motion_pending_events_.push_back(std::move(pending));
    }

    void fail_ground_motion(const std::string &reason)
    {
        if (ground_motion_failed_ || ground_motion_complete_) return;
        ground_motion_failed_ = true;
        ground_motion_stage_ = "failed";
        ground_motion_reason_ = reason;
        ground_input_stage_ = "failed";
        ground_input_reason_ = "ground_motion_" + reason;
        ground_motion_phase_ = "fault";
        queue_ground_motion_event("fault", reason);
        RCLCPP_ERROR(this->get_logger(),
                     "[ground_motion] stage=failed reason=%s",
                     reason.c_str());
    }

    void begin_ground_motion_trial(double now_sec, int trial_id,
                                   const std::array<double, 4> &start_q)
    {
        ground_motion_trial_id_ = trial_id;
        ground_motion_trial_direction_ = (trial_id % 2 == 1) ? 1 : -1;
        ground_motion_trial_anchor_ = start_q;
        ground_motion_trial_start_time_ = now_sec;
        ground_motion_stop_start_time_ = -1.0;
        ground_motion_stop_quiet_time_ = 0.0;
        ground_motion_excitation_steps_.fill(0);
        ground_motion_stage_ = "acceleration";
        ground_motion_phase_ = "acceleration";
        ground_motion_reason_ = "fixed_low_speed_probe";
        queue_ground_motion_event("trial_start", "fixed_low_speed_probe");
        queue_ground_motion_event("acceleration_start", "fixed_low_speed_probe");
    }

    void update_ground_motion_experiment(double dt, double now_sec)
    {
        if (!ground_motion_experiment_ || ground_motion_failed_ ||
            ground_motion_complete_) return;
        if (!ground_motion_start_requested_ && !ground_motion_started_) return;

        if (ground_contact_motion_experiment_) {
            ground_engine_selected_valid_ = ground_engine_history_.snapshot(
                this->now().nanoseconds(),
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count(),
                ground_engine_selected_);
            if (ground_engine_selected_valid_) {
                ground_engine_selected_valid_ = allocator_contact_history_.snapshot(
                    ground_engine_selected_.stamp_ns, ground_engine_contact_selected_) &&
                    ground_engine_contact_selected_.stamp_ns == ground_engine_selected_.stamp_ns &&
                    ground_engine_contact_selected_.iteration == ground_engine_selected_.iteration &&
                    ground_engine_contact_selected_.wheel_mask == 0x3;
            }
            if (!ground_engine_selected_valid_) {
                ground_motion_reason_ = "waiting_fresh_direct_engine_state";
                ground_motion_ready_elapsed_ = 0.0;
                if (ground_motion_started_) fail_ground_motion("direct_engine_source_stale_or_missing");
                else if (ground_motion_start_request_time_ >= 0.0 &&
                         now_sec - ground_motion_start_request_time_ > 2.0)
                    fail_ground_motion("pre_motion_direct_engine_source_timeout");
                return;
            }
        }

        if (!ground_motion_started_) {
            std::string guard_reason;
            const bool fresh = ground_input_support_fresh(now_sec);
            const bool quiet = fresh && current_state_ == bbot_jump::STATE_BALANCE &&
                std::abs(pitch_ - balance_offset_) < 0.04 &&
                std::abs(pitch_rate_) < 0.15 && std::abs(x_dot_) < 0.08 &&
                std::abs(capture_com_velocity_) < 0.08 &&
                std::abs(centroidal_height_.velocity()) < 0.03 &&
                std::abs(current_z_dot_) < 0.03 &&
                std::abs(hip_vel_left_) < 0.005 && std::abs(knee_vel_left_) < 0.005 &&
                std::abs(hip_vel_right_) < 0.005 && std::abs(knee_vel_right_) < 0.005;
            const bool engine_quiet = !ground_contact_motion_experiment_ ||
                (ground_engine_selected_valid_ &&
                 ground_engine_selected_.v.segment<4>(3).cwiseAbs().maxCoeff() <= .005);
            if (quiet && engine_quiet) ground_motion_ready_elapsed_ += std::max(dt, 0.0);
            else ground_motion_ready_elapsed_ = 0.0;
            ground_motion_reason_ = quiet ? "waiting_stable_entry_dwell" :
                (fresh ? "waiting_quiet_stand" : "waiting_fresh_effort_support");
            if (ground_motion_ready_elapsed_ >= 0.25) {
                ground_motion_anchor_ = {{hip_pos_left_, knee_pos_left_,
                                          hip_pos_right_, knee_pos_right_}};
                if (ground_contact_motion_experiment_) {
                    for (int i = 0; i < 4; ++i) ground_motion_anchor_[i] = ground_engine_selected_.q[3+i];
                    ground_engine_pitch_anchor_ = ground_engine_selected_.q[2];
                }
                ground_motion_trial_anchor_ = ground_motion_anchor_;
                ground_motion_qref_ = ground_motion_anchor_;
                ground_motion_vref_.fill(0.0);
                ground_motion_aref_.fill(0.0);
                ground_motion_baseline_effort_ = {{actual_tau_hip_left_, actual_tau_knee_left_,
                    actual_tau_hip_right_, actual_tau_knee_right_}};
                ground_motion_pitch_anchor_ = pitch_;
                ground_motion_blend_start_time_ = now_sec;
                ground_motion_blend_alpha_ = 0.0;
                ground_motion_started_ = true;
                ground_motion_law_active_ = true;
                ground_motion_stage_ = "blending";
                ground_motion_phase_ = "support_blend";
                ground_motion_reason_ = "two_second_support_law_blend";
                queue_ground_motion_event("law_blend_start", ground_motion_reason_);
                return;
            }
            if (ground_motion_start_request_time_ >= 0.0 &&
                now_sec - ground_motion_start_request_time_ > 2.0)
                fail_ground_motion("pre_motion_stability_timeout");
            return;
        }

        bbot_jump::GroundMotionGuardInput guard;
        guard.effort_active = effort_mode_active_;
        guard.switch_pending = leg_mode_switch_pending_;
        bbot_jump::AllocatorContactSnapshot freshContact;
        guard.bilateral_contact = allocator_contact_history_.snapshot(
            this->now().nanoseconds(), freshContact) && freshContact.continuous &&
            freshContact.wheel_mask == 0x3;
        guard.fresh_state = imu_received_ && odom_received_ && takeoff_odom_pose_valid_ &&
            capture_world_valid_ && centroidal_balance_.valid &&
            centroidal_lean_rate_.valid(now_sec) && centroidal_velocity_valid_ &&
            now_sec >= joint_sample_time_ && now_sec - joint_sample_time_ <= 0.020 &&
            torso_imu_.fresh(now_sec) && centroidal_height_.valid(now_sec);
        guard.anchor = ground_motion_anchor_;
        guard.q = {{hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_}};
        guard.v = {{hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_}};
        guard.pitch_delta = pitch_ - ground_motion_pitch_anchor_;
        guard.pitch_rate = pitch_rate_;
        if (ground_contact_motion_experiment_) {
            guard.fresh_state = guard.fresh_state && ground_engine_selected_valid_;
            for (int i = 0; i < 4; ++i) {
                guard.q[i] = ground_engine_selected_.q[3+i];
                guard.v[i] = ground_engine_selected_.v[3+i];
            }
            guard.pitch_delta = ground_engine_selected_.q[2] - ground_engine_pitch_anchor_;
            guard.pitch_rate = ground_engine_selected_.v[2];
        }
        ground_motion_guard_valid_ = bbot_jump::ground_motion_probe_guard(guard,
                                                                          ground_motion_reason_);
        ground_motion_anchor_error_max_ = 0.0;
        ground_motion_joint_rate_max_ = 0.0;
        ground_motion_soft_margin_min_ = std::numeric_limits<double>::infinity();
        const std::array<double, 4> q = guard.q;
        const std::array<double, 4> v = guard.v;
        const std::array<double, 4> qmin{{-1.52, -1.56, -1.52, -1.56}};
        const std::array<double, 4> qmax{{1.52, 1.56, 1.52, 1.56}};
        for (std::size_t i = 0; i < 4; ++i) {
            ground_motion_anchor_error_max_ = std::max(ground_motion_anchor_error_max_,
                std::abs(q[i] - ground_motion_anchor_[i]));
            ground_motion_joint_rate_max_ = std::max(ground_motion_joint_rate_max_,
                std::abs(v[i]));
            ground_motion_soft_margin_min_ = std::min(ground_motion_soft_margin_min_,
                std::min(q[i] - qmin[i], qmax[i] - q[i]));
        }
        if (!ground_motion_guard_valid_) {
            fail_ground_motion(ground_motion_reason_);
            return;
        }

        if (ground_motion_stage_ == "blending") {
            const double blend_t = std::max(0.0, now_sec - ground_motion_blend_start_time_);
            const double u = bbot_jump::clamp_value(blend_t / 2.0, 0.0, 1.0);
            ground_motion_blend_alpha_ = u * u * (3.0 - 2.0 * u);
            ground_motion_qref_ = ground_motion_anchor_;
            ground_motion_vref_.fill(0.0);
            ground_motion_aref_.fill(0.0);
            if (u >= 1.0) {
                queue_ground_motion_event("law_blend_complete", "support_law_active");
                ground_motion_stage_ = "blend_settle";
                ground_motion_phase_ = "blend_settle";
                ground_motion_blend_settle_start_time_ = now_sec;
                ground_motion_stop_quiet_time_ = 0.0;
            }
            return;
        }

        if (ground_motion_stage_ == "blend_settle") {
            ground_motion_qref_ = ground_motion_anchor_;
            ground_motion_vref_.fill(0.0);
            ground_motion_aref_.fill(0.0);
            const bool settled = std::abs(pitch_ - ground_motion_pitch_anchor_) < 0.04 &&
                std::abs(pitch_rate_) < 0.15 && std::abs(x_dot_) < 0.08 &&
                std::abs(capture_com_velocity_) < 0.08 &&
                std::abs(centroidal_height_.velocity()) < 0.03 &&
                std::abs(hip_vel_left_) < 0.005 && std::abs(knee_vel_left_) < 0.005 &&
                std::abs(hip_vel_right_) < 0.005 && std::abs(knee_vel_right_) < 0.005;
            const bool engine_settled = !ground_contact_motion_experiment_ ||
                (ground_engine_selected_valid_ &&
                 ground_engine_selected_.v.segment<4>(3).cwiseAbs().maxCoeff() <= .005);
            ground_motion_stop_quiet_time_ = settled && engine_settled ?
                ground_motion_stop_quiet_time_ + std::max(dt, 0.0) : 0.0;
            if (ground_motion_stop_quiet_time_ >= 0.25) {
                begin_ground_motion_trial(now_sec, 1, ground_motion_anchor_);
            } else if (now_sec - ground_motion_blend_settle_start_time_ > 2.0) {
                fail_ground_motion("support_blend_not_stable");
            }
            return;
        }

        if (ground_motion_stage_ == "acceleration" ||
            ground_motion_stage_ == "cruise" ||
            ground_motion_stage_ == "normal_stop") {
            const double elapsed = now_sec - ground_motion_trial_start_time_;
            if (!std::isfinite(elapsed) || elapsed < 0.0) {
                fail_ground_motion("simulation_clock_rollback");
                return;
            }
            const auto reference = bbot_jump::ground_motion_probe_reference(
                ground_motion_trial_anchor_, ground_motion_trial_direction_, elapsed);
            if (!reference.valid) {
                fail_ground_motion("invalid_motion_reference");
                return;
            }
            ground_motion_qref_ = reference.q;
            ground_motion_vref_ = reference.v;
            ground_motion_aref_ = reference.a;
            if (elapsed >= 0.5 && ground_motion_stage_ == "acceleration") {
                ground_motion_stage_ = "cruise";
                ground_motion_phase_ = "cruise";
                queue_ground_motion_event("cruise_start", "peak_rate_reached");
            }
            if (elapsed >= 1.0 && ground_motion_stage_ != "normal_stop") {
                ground_motion_stage_ = "normal_stop";
                ground_motion_stop_start_time_ = now_sec;
                ground_motion_stop_quiet_time_ = 0.0;
                ground_motion_phase_ = "normal_stop";
                queue_ground_motion_event("normal_stop_trigger", "reference_deceleration");
            }
            if (elapsed >= bbot_jump::kGroundMotionDuration) {
                ground_motion_stage_ = "stopping";
                ground_motion_stop_quiet_time_ = 0.0;
                ground_motion_phase_ = "normal_stop";
            }
            ground_motion_reason_ = "fixed_low_speed_probe";
            const std::array<double, 4> minimumExcitation{{.005,.010,.005,.010}};
            for (std::size_t i = 0; i < 4; ++i)
                if (std::abs(v[i]) >= minimumExcitation[i])
                    ++ground_motion_excitation_steps_[i];
        }

        if (ground_motion_stage_ == "stopping") {
            const bool stopped = ground_contact_motion_experiment_ ?
                (ground_engine_selected_valid_ && ground_engine_selected_.v.segment<4>(3).cwiseAbs().maxCoeff() <= .005) :
                (std::abs(hip_vel_left_) <= .005 && std::abs(knee_vel_left_) <= .005 &&
                 std::abs(hip_vel_right_) <= .005 && std::abs(knee_vel_right_) <= .005);
            ground_motion_stop_quiet_time_ = stopped ?
                ground_motion_stop_quiet_time_ + std::max(dt, 0.0) : 0.0;
            if (ground_motion_stop_quiet_time_ >= 0.25) {
                // Preserve the successful measured stop outcome even when the
                // raw-sample excitation qualification later rejects this trial.
                queue_ground_motion_event("stop_complete", "all_four_joint_rates_settled");
                const bool excited = std::all_of(ground_motion_excitation_steps_.begin(),
                    ground_motion_excitation_steps_.end(), [](int n) { return n >= 30; });
                if (!excited) {
                    fail_ground_motion("insufficient_joint_excitation");
                    return;
                }
                const auto endpoint = bbot_jump::ground_motion_probe_reference(
                    ground_motion_trial_anchor_, ground_motion_trial_direction_,
                    bbot_jump::kGroundMotionDuration);
                if (!endpoint.valid) {
                    fail_ground_motion("invalid_trial_endpoint");
                    return;
                }
                if (ground_motion_trial_id_ >= 4) {
                    ground_motion_complete_ = true;
                    ground_motion_stage_ = "complete";
                    ground_motion_phase_ = "campaign";
                    ground_motion_reason_ = "four_probe_trials_complete";
                    queue_ground_motion_event("campaign_complete", ground_motion_reason_);
                    return;
                }
                begin_ground_motion_trial(now_sec, ground_motion_trial_id_ + 1,
                                          endpoint.q);
            } else if (ground_motion_stop_start_time_ >= 0.0 &&
                       now_sec - ground_motion_stop_start_time_ > 2.0) {
                fail_ground_motion("normal_stop_timeout");
                return;
            }
        }
    }

    void write_ground_motion_after_leg_publish()
    {
        write_ground_motion_record(true);
    }

    void write_ground_motion_without_leg_publish()
    {
        write_ground_motion_record(false);
    }

    void write_ground_motion_record(bool leg_publish_occurred)
    {
        if (!ground_motion_experiment_) return;
        if (!leg_publish_occurred && ground_motion_fault_logged_) return;
        const int replay = ground_motion_trial_id_ > 0 ?
            (ground_motion_trial_id_ + 1) / 2 : 0;
        const int64_t simNs = capture_control_ns_;
        const auto wallNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (phase_events_log_.is_open()) {
            for (const auto &event : ground_motion_pending_events_) {
                phase_events_log_ << std::setprecision(17) << event.trial_id << ','
                    << event.replay_id << ',' << event.phase << ',' << event.event << ','
                    << ++ground_motion_event_id_ << ',' << simNs << ',' << wallNs << ',';
                if (leg_publish_occurred)
                    phase_events_log_ << command_publication_id_ << ",leg,";
                else
                    phase_events_log_ << "nan,none,";
                phase_events_log_ << event.reason;
                for (double qref : event.q) phase_events_log_ << ',' << qref;
                phase_events_log_ << ',' << last_wheel_cmd_x_ << ',' << target_yaw_rate_ << '\n';
            }
            if (!ground_motion_pending_events_.empty()) phase_events_log_.flush();
        }
        ground_motion_pending_events_.clear();

        if (motion_trace_log_.is_open()) {
            bbot_jump::AllocatorContactSnapshot contact;
            const bool contactValid = allocator_contact_history_.snapshot(
                this->now().nanoseconds(), contact);
            const bool jointSourceValid = std::isfinite(joint_sample_time_) && joint_sample_time_ >= 0.0;
            const bool imuSourceValid = std::isfinite(torso_imu_.stamp()) && torso_imu_.stamp() >= 0.0;
            const bool odomSourceValid = std::isfinite(takeoff_odom_stamp_) && takeoff_odom_stamp_ >= 0.0;
            const bool normalStop = ground_motion_stage_ == "normal_stop" ||
                ground_motion_stage_ == "stopping";
            motion_trace_log_ << std::setprecision(17) << ground_motion_trial_id_ << ','
                << replay << ',' << ground_motion_phase_ << ',' << ground_motion_stage_
                << ',' << ground_motion_reason_ << ',' << simNs << ',' << normalStop << ','
                << ground_motion_failed_ << ',' << ground_motion_failed_ << ','
                << ground_motion_complete_ << ',' << effort_mode_active_ << ','
                << leg_mode_switch_pending_ << ',' << contactValid << ','
                << (contactValid && contact.continuous) << ','
                << (contactValid ? static_cast<int>(contact.wheel_mask) : 0) << ','
                << jointSourceValid << ',';
            if (jointSourceValid) motion_trace_log_ << static_cast<int64_t>(std::llround(joint_sample_time_ * 1e9));
            else motion_trace_log_ << "nan";
            motion_trace_log_ << ',' << imuSourceValid << ',';
            if (imuSourceValid) motion_trace_log_ << static_cast<int64_t>(std::llround(torso_imu_.stamp() * 1e9));
            else motion_trace_log_ << "nan";
            motion_trace_log_ << ',' << odomSourceValid << ',';
            if (odomSourceValid) motion_trace_log_ << static_cast<int64_t>(std::llround(takeoff_odom_stamp_ * 1e9));
            else motion_trace_log_ << "nan";
            motion_trace_log_ << ',' << contactValid << ',';
            if (contactValid) motion_trace_log_ << contact.stamp_ns;
            else motion_trace_log_ << "nan";
            motion_trace_log_ << ',';
            if (contactValid) motion_trace_log_ << contact.sequence;
            else motion_trace_log_ << "nan";
            motion_trace_log_ << ',';
            if (leg_publish_occurred) {
                motion_trace_log_ << command_publication_id_ << ',' << capture_control_ns_
                    << ',' << last_command_publish_ns_ << ',' << last_command_publish_end_ns_
                    << ',' << last_command_wall_publish_ns_ << ',' << last_command_wall_publish_end_ns_;
            } else {
                motion_trace_log_ << "nan," << capture_control_ns_ << ",nan,nan,nan,nan";
            }
            motion_trace_log_ << ',' << (pitch_ - ground_motion_pitch_anchor_) << ',' << pitch_rate_
                << ',' << ground_motion_anchor_error_max_ << ',' << ground_motion_joint_rate_max_
                << ',' << ground_motion_soft_margin_min_;
            for (double x : ground_motion_qref_) motion_trace_log_ << ',' << x;
            for (double x : ground_motion_vref_) motion_trace_log_ << ',' << x;
            for (double x : ground_motion_aref_) motion_trace_log_ << ',' << x;
            for (double x : ground_motion_trial_anchor_) motion_trace_log_ << ',' << x;
            motion_trace_log_ << ',' << hip_pos_left_ << ',' << knee_pos_left_
                << ',' << hip_pos_right_ << ',' << knee_pos_right_
                << ',' << hip_vel_left_ << ',' << knee_vel_left_
                << ',' << hip_vel_right_ << ',' << knee_vel_right_
                << ',' << left_wheel_vel_ << ',' << right_wheel_vel_
                << ',' << last_wheel_cmd_x_ << ',' << target_yaw_rate_;
            for (double x : ground_motion_static_feedforward_) motion_trace_log_ << ',' << x;
            for (double x : ground_motion_joint_feedback_) motion_trace_log_ << ',' << x;
            motion_trace_log_ << ',' << ground_motion_torso_static_ff_
                << ',' << ground_motion_torso_request_ << ',' << actual_tau_hip_left_
                << ',' << actual_tau_knee_left_ << ',' << actual_tau_hip_right_
                << ',' << actual_tau_knee_right_ << '\n';
            motion_trace_log_.flush();
        }
        if (!leg_publish_occurred) ground_motion_fault_logged_ = true;
    }

    double publish_ground_motion_height_effort()
    {
        if (ground_contact_motion_experiment_) {
            // Offline qualification is mandatory before connecting the new
            // law. Never let this opt-in silently fall through to old PD.
            fail_ground_motion("contact_motion_offline_qualification_pending");
            lock_owned_outputs("contact_motion_offline_qualification_pending");
            return std::numeric_limits<double>::quiet_NaN();
        }
        const auto mean_height = 0.5 * (
            kinematics_.calculate_com_height(pitch_, ground_motion_qref_[0],
                                              ground_motion_qref_[1]) +
            kinematics_.calculate_com_height(pitch_, ground_motion_qref_[2],
                                              ground_motion_qref_[3]));
        double force_target = TOTAL_MASS_ * 0.5 * 9.81 +
            K_Z_BUFFER_ * (mean_height - current_z_) - D_Z_BUFFER_ * current_z_dot_;
        force_target = bbot_jump::clamp_value(force_target, 0.0, F_Z_BUFFER_MAX_);
        if (!height_force_initialized_) {
            height_force_per_leg_ = force_target;
            height_force_initialized_ = true;
        }
        height_force_per_leg_ += bbot_jump::clamp_value(
            force_target - height_force_per_leg_, -1500.0 * 0.005, 1500.0 * 0.005);
        const double forcePerLeg = height_force_per_leg_;

        double jhLeft, jkLeft, jhRight, jkRight;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_,
                                      jhLeft, jkLeft);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_,
                                      jhRight, jkRight);
        // The common Jz^T Fz term is deliberately sent for both hips and both
        // knees; the private publisher path adds gravity once and adds torso
        // correction without replacing that common support component.
        publish_effort_leg_control_lr(
            ground_motion_qref_[0], ground_motion_qref_[1],
            ground_motion_qref_[2], ground_motion_qref_[3],
            ground_motion_vref_[0], ground_motion_vref_[1],
            ground_motion_vref_[2], ground_motion_vref_[3],
            forcePerLeg * jhLeft, forcePerLeg * jkLeft,
            forcePerLeg * jhRight, forcePerLeg * jkRight,
            25.0, 3.5, 45.0, 6.0);
        return 2.0 * forcePerLeg;
    }

    void update_ground_input_experiment(double dt, double now_sec)
    {
        if (!ground_input_experiment_) return;
        if (ground_input_stage_ == "effort_hold") {
            if (!ground_input_support_fresh(now_sec)) {
                ground_input_stage_ = "failed";
                ground_input_reason_ = (!effort_mode_active_ || leg_mode_switch_pending_)
                    ? "effort_mode_lost_after_handoff" : "stale_state_or_bilateral_contact_lost";
                RCLCPP_ERROR(this->get_logger(), "[ground_input] stage=failed reason=%s",
                             ground_input_reason_.c_str());
            }
            return;
        }
        if (ground_input_stage_ == "failed") return;
        if (!ground_input_requested_) return;
        if (current_state_ != bbot_jump::STATE_BALANCE) {
            ground_input_stable_time_ = 0.0;
            ground_input_reason_ = "waiting_balance_state";
            return;
        }
        if (ground_input_stage_ == "switching") {
            if (effort_mode_active_ && !leg_mode_switch_pending_) {
                // These are local wheel/COM coordinates at entry, not a fake
                // touchdown event or a world-origin target.
                touchdown_x_ref_ = x_;
                target_x_ = x_;
                capture_v_ref_ = 0.0;
                capture_world_active_ = false;
                Eigen::Vector2d heading = odom_body_rotation_.col(1).head<2>();
                if (heading.norm() > 0.5) {
                    jump_forward_axis_world_ = heading.normalized();
                    jump_forward_axis_valid_ = true;
                }
                post_landing_effort_support_ = true;
                ground_input_stage_ = "effort_hold";
                ground_input_reason_ = "effort_mode_confirmed_support_handoff";
                post_jump_balance_stable_timer_ = 0.0;
                RCLCPP_INFO(this->get_logger(),
                            "[ground_input] stage=effort_hold reason=%s contact_seq=%llu contact_ns=%lld",
                            ground_input_reason_.c_str(),
                            static_cast<unsigned long long>(reference_ground_pulse_sequence_),
                            static_cast<long long>(reference_ground_pulse_contact_stamp_ns_));
                return;
            }
            if (!leg_mode_switch_pending_ && effort_switch_result_ >= 2) {
                ground_input_stage_ = "failed";
                ground_input_reason_ = "effort_controller_switch_rejected";
                RCLCPP_ERROR(this->get_logger(), "[ground_input] stage=failed reason=%s",
                             ground_input_reason_.c_str());
            } else if (now_sec - effort_switch_request_stamp_ > 1.0) {
                ground_input_stage_ = "failed";
                ground_input_reason_ = "effort_controller_switch_timeout";
                RCLCPP_ERROR(this->get_logger(), "[ground_input] stage=failed reason=%s",
                             ground_input_reason_.c_str());
            }
            return;
        }

        std::string reason;
        const bool ready = ground_input_entry_ready(now_sec, reason);
        ground_input_stable_time_ = ready ? ground_input_stable_time_ + dt : 0.0;
        ground_input_reason_ = reason;
        if (!ready || ground_input_stable_time_ < 0.50) return;
        if (!switch_ctrl_client_->service_is_ready()) {
            ground_input_reason_ = "stable_gate_passed_waiting_switch_service";
            return;
        }

        // Preload the existing Effort height/torso support while Position
        // remains active, then preload the measured pose into Position before
        // requesting the strict controller switch.
        publish_effort_height_control(current_height_, 0.0,
            K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_, 25.0, 3.5, 45.0, 6.0, true);
        publish_position_leg_control_lr(hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_);
        touchdown_x_ref_ = x_;
        target_x_ = x_;
        capture_v_ref_ = 0.0;
        ground_input_stage_ = "switching";
        ground_input_reason_ = "stable_gate_passed_effort_support_preloaded";
        ground_input_preload_sent_ = true;
        request_effort_controller();
        RCLCPP_INFO(this->get_logger(),
                    "[ground_input] stage=switching reason=%s stable_s=%.3f contact_seq=%llu contact_ns=%lld",
                    ground_input_reason_.c_str(), ground_input_stable_time_,
                    static_cast<unsigned long long>(reference_ground_pulse_sequence_),
                    static_cast<long long>(reference_ground_pulse_contact_stamp_ns_));
    }

    void lock_owned_outputs(const std::string &reason)
    {
        thrust_output_locked_ = thrust_output_locked_ || thrust_effort_owned_;
        if (ground_motion_experiment_ && ground_motion_started_ &&
            !ground_motion_failed_ && !ground_motion_complete_)
            fail_ground_motion(reason);
        if (ground_input_experiment_ &&
            (ground_input_stage_ == "effort_hold" || ground_input_stage_ == "switching")) {
            ground_input_stage_ = "failed";
            ground_input_reason_ = reason;
        }
        publish_wheel_cmd(0.0, 0.0, true);
        if (!effort_mode_active_ || leg_mode_switch_pending_) {
            if (ground_motion_failed_) write_ground_motion_without_leg_publish();
            return;
        }
        if (ground_input_experiment_ && ground_input_stage_ == "failed") {
            // Preserve the last finite bounded support output without invoking
            // stale-measurement gravity, trajectory, torso, or PD feedback.
            const std::array<double, 4> last{{actual_tau_hip_left_, actual_tau_knee_left_,
                actual_tau_hip_right_, actual_tau_knee_right_}};
            std_msgs::msg::Float64MultiArray safe;
            safe.data.resize(4, 0.0);
            const std::array<double, 4> limits{{75.0, 60.0, 75.0, 60.0}};
            for (std::size_t i = 0; i < last.size(); ++i)
                safe.data[i] = std::isfinite(last[i]) ?
                    bbot_jump::clamp_value(last[i], -limits[i], limits[i]) : 0.0;
            publish_recorded_command(leg_effort_pub_, safe, "leg", safe.data,
                                     NAN, NAN);
            write_ground_motion_after_leg_publish();
        } else {
            std_msgs::msg::Float64MultiArray safe_zero;
            safe_zero.data = {0.0, 0.0, 0.0, 0.0};
            publish_recorded_command(leg_effort_pub_, safe_zero, "leg", safe_zero.data, NAN, NAN);
        }
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[output-ownership] locked outputs after sensor/control failure: %s", reason.c_str());
    }

    void control_loop()
    {
        // Early sensor-dropout rows have no later control sample to supply a
        // clock key. Capture the current simulator key before that return;
        // normal due-control samples replace it with their exact `now` sample.
        capture_control_ns_ = this->now().nanoseconds();
        if (!imu_received_ || !wheel_origin_set_) {
            if (bbot_jump::controller_has_protected_output_owner(
                    thrust_effort_owned_, ground_input_has_protected_outputs()))
                lock_owned_outputs("required_imu_or_wheel_origin_missing");
            return;
        }
        ++timer_calls_since_control_;

        process_keyboard();

        rclcpp::Time now = this->now();
        capture_control_ns_ = now.nanoseconds();
        double now_sec = now.seconds();
        double previous_control_sec = last_time_.seconds();
        double dt = 0.0;
        const bool control_due = bbot_jump::advance_control_time(now_sec, previous_control_sec, dt);
        if (!control_due)
        {
            if (now < last_time_)
            {
                last_time_ = now;
                if (bbot_jump::controller_has_protected_output_owner(
                        thrust_effort_owned_, ground_input_has_protected_outputs()))
                    lock_owned_outputs("simulation_clock_rollback");
            }
            return;
        }
        last_time_ = now;
        thrust_support_reference_diag_ = {};
        thrust_support_imu_stamp_diag_ = 0.0;
        thrust_support_joint_stamp_diag_ = 0.0;
        thrust_support_com_stamp_diag_ = 0.0;
        thrust_support_hip_servo_left_diag_ = 0.0;
        thrust_support_hip_servo_right_diag_ = 0.0;
        control_log_timestamp_sec_ = bbot_jump::control_sample_log_timestamp(
            now_sec, start_time_.seconds());
        const auto wall_now = std::chrono::steady_clock::now();
        if (control_wall_time_valid_)
        {
            control_wall_dt_ms_diag_ =
                std::chrono::duration<double, std::milli>(wall_now - last_control_wall_time_).count();
        }
        last_control_wall_time_ = wall_now;
        control_wall_time_valid_ = true;
        timer_calls_for_update_diag_ = timer_calls_since_control_;
        timer_calls_since_control_ = 0;
        control_sim_dt_ms_diag_ = dt * 1000.0;
        handoff_reference_mode_diag_ = 0;
        handoff_reference_valid_diag_ = false;
        handoff_reference_limited_diag_ = false;
        handoff_force_suppressed_diag_ = false;
        handoff_reference_target_diag_.fill(0.);
        handoff_reference_applied_diag_.fill(0.);
        handoff_reference_rate_diag_.fill(0.);
        thrust_momentum_reference_active_diag_ = false;
        thrust_momentum_reference_blend_diag_ = 0.0;
        thrust_momentum_reference_vz_diag_ = 0.0;
        thrust_momentum_reference_h_diag_ = 0.0;
        thrust_momentum_reference_hip_delta_diag_ = 0.0;
        thrust_momentum_reference_knee_delta_diag_ = 0.0;
        thrust_momentum_reference_hip_feedback_l_diag_ = 0.0;
        thrust_momentum_reference_hip_feedback_r_diag_ = 0.0;
        landing_momentum_diag_ = {};
        landing_momentum_stamp_diag_ = -1.0;
        bbot_jump::LandingJointSample aligned_momentum_sample;
        if (landing_joint_history_.exact(torso_imu_.stamp(), now_sec, aligned_momentum_sample))
        {
            landing_momentum_diag_ = bbot_jump::landing_momentum(
                aligned_momentum_sample.q, aligned_momentum_sample.v,
                aligned_momentum_sample.wheel_rate, pitch_rate_raw_, body_mass_);
            if (landing_momentum_diag_.valid)
                landing_momentum_stamp_diag_ = aligned_momentum_sample.stamp;
        }
        arrest_dynamics_feedforward_scope_diag_ = false;
        arrest_dynamics_feedforward_guard_diag_ =
            static_cast<int>(bbot_jump::ArrestDynamicsGuard::DisabledOrWrongPhase);
        arrest_dynamics_feedforward_blend_diag_ = 0.0;
        arrest_dynamics_qdd_diag_.fill(0.0);
        arrest_dynamics_inertial_diag_.fill(0.0);
        arrest_dynamics_bias_diag_.fill(0.0);
        arrest_dynamics_raw_diag_.fill(0.0);
        arrest_dynamics_bounded_diag_.fill(0.0);
        arrest_dynamics_applied_diag_.fill(0.0);
        // Per-control-sample diagnostics must not leak across hops or phases.
        thrust_release_fast_rate_correction_active_diag_ = false;
        thrust_fast_rate_correction_scope_diag_ = false;
        thrust_fast_rate_correction_blend_diag_ = 0.0;
        thrust_release_fast_rate_correction_requested_diag_ = 0.0;
        thrust_release_fast_rate_correction_applied_diag_ = 0.0;
        thrust_forward_speed_raw_diag_ = 0.0;
        thrust_forward_speed_base_diag_ = 0.0;
        thrust_forward_speed_predicted_diag_ = 0.0;
        thrust_forward_speed_accel_diag_ = 0.0;
        thrust_forward_speed_horizon_diag_ = 0.0;
        thrust_forward_speed_delta_diag_ = 0.0;
        thrust_forward_speed_prediction_active_diag_ = false;
        thrust_kinematics_stamp_diag_ = -1.0;
        thrust_kinematics_age_diag_ = 0.0;
        thrust_kinematics_dt_diag_ = 0.0;
        thrust_kinematics_r_dot_diag_ = 0.0;
        thrust_kinematics_shank_projection_diag_ = 0.0;
        thrust_kinematics_raw_correction_diag_ = 0.0;
        thrust_kinematics_applied_correction_diag_ = 0.0;
        thrust_kinematics_valid_diag_ = false;
        thrust_kinematics_q_diag_ = {};
        thrust_kinematics_rotation_diag_.setIdentity();
        thrust_kinematics_relative_body_diag_.setZero();
        initial_balance_wheel_slew_active_diag_ = false;
        initial_balance_wheel_raw_target_diag_ = 0.0;
        initial_balance_wheel_applied_cmd_diag_ = 0.0;

        // v6.7：机身姿态与整机重心是不同状态。腿在落地压缩时会改变
        // 重心相对轮轴的位置和速度；所有落地轮控阶段共用这一状态。
        centroidal_balance_ = bbot_jump::centroidal_balance_state(
            {hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_},
            {hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_},
            pitch_, pitch_rate_, body_mass_);
        centroidal_legacy_lean_rate_ = centroidal_balance_.rate;
        if (odom_received_ && takeoff_odom_pose_valid_)
            centroidal_lean_rate_.update(takeoff_odom_stamp_, now_sec,
                                         odom_body_rotation_, takeoff_joint_history_, body_mass_);
        if (centroidal_lean_rate_.valid(now_sec))
            centroidal_balance_.rate = centroidal_lean_rate_.rate();
        if (odom_received_ && takeoff_odom_pose_valid_)
            centroidal_height_.update(takeoff_odom_stamp_, now_sec, gazebo_world_z_,
                                      odom_world_z_in_body_, takeoff_joint_history_, body_mass_);
        centroidal_velocity_valid_ = takeoff_odom_pose_valid_ && centroidal_height_.valid(now_sec);

        if (odom_received_ && takeoff_odom_pose_valid_)
            centroidal_world_.update(takeoff_odom_stamp_, now_sec, odom_base_position_,
                                     odom_body_rotation_, takeoff_joint_history_, body_mass_);
        Eigen::Vector3d heading = odom_body_rotation_.col(1);
        heading.z() = 0.0;
        capture_world_valid_ = takeoff_odom_pose_valid_ && centroidal_world_.valid(now_sec) &&
                               heading.norm() > 0.5 && centroidal_balance_.valid &&
                               torso_imu_.fresh(now_sec) && now_sec >= joint_sample_time_ &&
                               now_sec - joint_sample_time_ <= 0.080;
        if (capture_world_valid_)
            capture_com_velocity_ = centroidal_world_.forward_velocity(heading.normalized());
        if (current_state_ == bbot_jump::STATE_THRUST) {
            if (capture_world_valid_ && jump_forward_axis_valid_) {
                const auto & com_position = centroidal_world_.position();
                thrust_forward_speed_predictor_.observe(
                    centroidal_world_.stamp(),
                    {com_position.x(), com_position.y(), com_position.z()},
                    {jump_forward_axis_world_.x(), jump_forward_axis_world_.y()});
            } else {
                thrust_forward_speed_predictor_.reset();
            }
            if (thrust_wheel_kinematics_compensation_enable_ &&
                effort_mode_active_ && !leg_mode_switch_pending_ &&
                takeoff_odom_pose_valid_ && jump_forward_axis_valid_) {
                const auto & estimate = thrust_wheel_kinematics_observer_.update(
                    takeoff_odom_stamp_, now_sec, odom_body_rotation_,
                    takeoff_joint_history_, jump_forward_axis_world_, body_mass_, wheel_radius_);
                if (estimate.stamp >= 0.0) {
                    thrust_kinematics_stamp_diag_ = estimate.stamp;
                    thrust_kinematics_age_diag_ = estimate.age;
                    thrust_kinematics_dt_diag_ = estimate.dt;
                    thrust_kinematics_r_dot_diag_ = estimate.r_dot;
                    thrust_kinematics_shank_projection_diag_ = estimate.shank_projection;
                    thrust_kinematics_raw_correction_diag_ = estimate.raw_correction;
                    thrust_kinematics_valid_diag_ = estimate.fresh && estimate.frame_valid;
                    thrust_kinematics_q_diag_ = estimate.q;
                    thrust_kinematics_rotation_diag_ = estimate.rotation;
                    thrust_kinematics_relative_body_diag_ = estimate.com_minus_axle_body;
                    if (thrust_kinematics_valid_diag_)
                        thrust_kinematics_applied_correction_diag_ = estimate.correction;
                }
            } else {
                thrust_wheel_kinematics_observer_.reset();
            }
        } else {
            thrust_forward_speed_predictor_.reset();
            thrust_wheel_kinematics_observer_.reset();
        }
        capture_world_active_ = false;
        update_capture_velocity_reference(dt);
        update_ground_input_experiment(dt, now_sec);
        update_ground_motion_experiment(dt, now_sec);
        if (ground_input_log_.is_open()) {
            bbot_jump::AllocatorContactSnapshot contact;
            const bool contact_valid = allocator_contact_history_.snapshot(
                now.nanoseconds(), contact);
            ground_input_log_ << std::setprecision(17) << now.nanoseconds()
                << ',' << ground_input_stage_ << ',' << ground_input_reason_
                << ',' << ground_input_stable_time_ << ',' << effort_mode_active_
                << ',' << leg_mode_switch_pending_ << ',' << post_landing_effort_support_
                << ',' << contact_valid << ',' << (contact_valid && contact.continuous)
                << ',' << (contact_valid ? static_cast<int>(contact.wheel_mask) : 0)
                << ',' << (contact_valid ? contact.sequence : 0)
                << ',' << (contact_valid ? contact.stamp_ns : 0)
                << ',' << torso_imu_.stamp() << ',' << joint_sample_time_ << ',' << takeoff_odom_stamp_
                << ',' << pitch_ << ',' << pitch_rate_ << ',' << ground_balance_angle()
                << ',' << ground_balance_rate() << ',' << x_dot_ << ',' << capture_com_velocity_
                << ',' << (centroidal_velocity_valid_ ? centroidal_height_.velocity() : NAN)
                << ',' << current_height_ << ',' << current_z_
                << ',' << hip_pos_left_ << ',' << knee_pos_left_ << ',' << hip_pos_right_ << ',' << knee_pos_right_
                << ',' << hip_vel_left_ << ',' << knee_vel_left_ << ',' << hip_vel_right_ << ',' << knee_vel_right_
                << ',' << left_wheel_vel_ << ',' << right_wheel_vel_ << '\n';
            ground_input_log_.flush();
        }

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
            if (effort_mode_active_)
            {
                publish_effort_leg_control(hip_pos_left_, knee_pos_left_, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            }
            else
            {
                publish_position_leg_control(hip_pos_left_, knee_pos_left_);
            }
            // 该状态原本不进日志，落地止损(+fall-guard-v6.27)把这里当终止态用，
            // 没有这几行就看不到"轮速清零之后机身是否真的停下来"。
            log_data(0.0, 0.0, 0.0, 0.0);
            break;
        }

        num_++;
    }

    // ── 阶段 0：变高度 LQR 自平衡 ──
    void run_state_balance(double dt)
    {
        if (ground_input_experiment_ && ground_input_stage_ == "failed") {
            lock_owned_outputs(ground_input_reason_);
            log_data(0.0, 0.0, 0.0, TOTAL_MASS_ * 9.81);
            return;
        }
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
        if (target_speed_smoothed_ < target_speed_const_)
        {
            target_speed_smoothed_ += target_speed_step;
            if (target_speed_smoothed_ > target_speed_const_)
                target_speed_smoothed_ = target_speed_const_;
        }
        else if (target_speed_smoothed_ > target_speed_const_)
        {
            target_speed_smoothed_ -= target_speed_step;
            if (target_speed_smoothed_ < target_speed_const_)
                target_speed_smoothed_ = target_speed_const_;
        }
        double target_speed = target_speed_smoothed_;

        // 4. 位置参考积分
        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            if (was_moving_)
            {
                target_x_ = x_;
                was_moving_ = false;
            }
        }
        else
        {
            target_x_ += target_speed * dt;
            was_moving_ = true;
        }

        // 开机启动平稳过渡与姿态优先仲裁
        if (!balance_started_)
        {
            balance_start_time_ = this->now().seconds();
            balance_started_ = true;
            target_x_ = x_;
        }
        const double startup_elapsed = this->now().seconds() - balance_start_time_;

        double dynamic_target_pitch = post_landing_gyro_reduced_ ? post_landing_pitch_ref_ : balance_offset_;

        // 姿态未稳定或速度较大时，target_x_ 持续跟随当前位置 x_，
        // 绝不允许位置环拉扯干扰倒立摆俯仰平衡
        const bool balance_not_settled = (std::abs(pitch_ - dynamic_target_pitch) > 0.035) ||
                                         (std::abs(pitch_rate_) > 0.15) ||
                                         (std::abs(x_dot_) > 0.25) ||
                                         (startup_elapsed < 1.5);
        if (balance_not_settled)
        {
            target_x_ = x_;
        }

        // 位置环增益软启动淡入 (仅对位置误差 pos_error 生效，速度阻尼 k_x_dot 始终全额生效以抑制漂移)
        double pos_gain_scale = 1.0;
        if (startup_elapsed < 1.5)
        {
            pos_gain_scale = 0.0;
        }
        else if (startup_elapsed < 3.0)
        {
            pos_gain_scale = (startup_elapsed - 1.5) / 1.5;
        }

        // 状态误差计算 (实际值 - 目标值)
        double pos_error = x_ - target_x_;
        double vel_error = x_dot_ - target_speed;
        double gyro_val = pitch_rate_;
        double theta_error = 0.0;
        double u_pitch = 0.0;
        double cmd_x = 0.0;
        const double gyro_gain_scale =
            post_landing_gyro_reduced_ ? 0.50 : 1.0;

        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(pos_gain_scale * current_gain_.k_x * pos_error +
                        current_gain_.k_x_dot * vel_error +
                        current_gain_.k_theta * theta_error +
                        gyro_gain_scale * current_gain_.k_theta_dot * gyro_val);
            cmd_x = -u_pitch * cmd_scale_;
            vel_integral_ = 0.0;
        }
        else
        {
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

        cmd_x = bbot_jump::clamp_value(cmd_x, -2.5, 2.5);

        const bool initial_position_support = !post_landing_effort_support_;
        if (initial_position_support)
        {
            // Keep the legacy LQR target and +/-2.5 bound, but rate-limit its
            // initial Position-support command to the same 8 m/s^2 used by
            // the established ground wheel path. This is a target slew, not
            // a wheel-speed or measured-force limit.
            initial_balance_wheel_slew_active_diag_ = true;
            initial_balance_wheel_raw_target_diag_ = cmd_x;
            cmd_x = bbot_jump::initial_balance_wheel_command(
                cmd_x, last_wheel_cmd_x_, dt, true, 8.0);
        }

        if (post_landing_effort_support_)
        {
            // RECOVERY and post-landing Effort BALANCE must use the same
            // COM-aware wheel law. Switching to the pre-jump pitch-only LQR
            // here caused an immediate command jump after a stable recovery.
            double p_term = 0.0;
            double d_term = 0.0;
            double capture_state = 0.0;
            double capture_guard = 0.0;
            const double hold_target = compute_post_brake_hold_wheel_target(
                p_term, d_term, capture_state, capture_guard);
            const double max_accel = capture_world_active_ ? 8.0 :
                (std::abs(x_dot_) < 0.20 && std::abs(capture_state) < 0.12) ? 4.0 : 7.5;
            const double max_step = max_accel * std::max(dt, 0.001);
            cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                hold_target - last_wheel_cmd_x_, -max_step, max_step);
        }

        // 落地后平移速度与变化率限制
        if (post_landing_translation_feedback_)
        {
            cmd_x = bbot_jump::clamp_value(cmd_x, -1.0, 1.0);
            const double max_cmd_step = 10.0 * dt;
            cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                                            cmd_x - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        }

        if (pre_jump_balance_handoff_)
        {
            const double requested_cmd = cmd_x;
            cmd_x = bbot_jump::rolling_abort_wheel_command(cmd_x, last_wheel_cmd_x_, dt);
            const bool stopped = std::abs(target_speed) < 0.005 &&
                                 std::abs(x_dot_) < 0.04 && std::abs(pitch_rate_) < 0.15;
            if (stopped && std::abs(requested_cmd - cmd_x) < 1e-6)
                pre_jump_balance_handoff_ = false;
        }

        if (initial_balance_wheel_slew_active_diag_)
            initial_balance_wheel_applied_cmd_diag_ = cmd_x;

        // 落地软接管计时
        if (post_landing_balance_soft_start_)
        {
            const double balance_elapsed = this->now().seconds() - balance_entry_time_;
            if (balance_elapsed >= 1.0)
            {
                post_landing_balance_soft_start_ = false;
            }
        }
        publish_wheel_cmd(cmd_x, target_yaw_rate_);

        // 8Hz 遥测打印
        if (num_ % 25 == 0)
        {
            const double term_x = pos_gain_scale * current_gain_.k_x * pos_error;
            const double term_xdot = current_gain_.k_x_dot * vel_error;
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
                      << " pitch_rate=" << pitch_rate_ << '\n';
            if (post_landing_effort_support_)
            {
                std::cout << "  applied_hold: capture=" << capture_world_active_
                          << " attitude=" << wheel_attitude_cmd_diag_
                          << " velocity=" << wheel_vel_term_diag_
                          << " position=" << wheel_pos_term_diag_
                          << " target=" << wheel_cmd_target_diag_
                          << " cmd_x=" << cmd_x << std::endl;
            }
            else
            {
                std::cout << "  terms: x=" << term_x
                          << " v=" << term_xdot
                          << " pitch=" << term_theta
                          << " gyro=" << term_theta_dot
                          << " cmd_x=" << cmd_x << std::endl;
            }
        }

        // 腿部逆运动学与重力矩计算
        bbot_kinematics::IKSolution ik_bal =
            kinematics_.inverse_kinematics(current_height_, 0.0);
        bbot_kinematics::JointTorques g_torques =
            kinematics_.compute_gravity_torques(
                0.0, ik_bal.theta_hip, ik_bal.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;

        if (post_landing_effort_support_)
        {
            // 跳后稳态力矩支撑
            request_effort_controller();
            if (effort_mode_active_)
            {
                if (ground_motion_law_active()) {
                    support_force_total = publish_ground_motion_height_effort();
                } else {
                    // BALANCE retains its original support law when the
                    // isolated motion probe is disabled.
                    support_force_total = publish_effort_height_control(
                        current_height_, 0.0,
                        K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                        25.0, 3.5, 45.0, 6.0, true);
                }
            }
        }
        else
        {
            // 起跳前默认位置控制模式
            if (ground_input_stage_ == "switching")
                publish_position_leg_control_lr(hip_pos_left_, knee_pos_left_,
                                                hip_pos_right_, knee_pos_right_);
            else {
                const double q_hip_cmd = ik_bal.theta_hip + 0.008;
                const double q_knee_cmd = ik_bal.theta_knee - 0.040;
                publish_position_leg_control(q_hip_cmd, q_knee_cmd);
            }
        }

        log_data(cmd_x,
                 g_torques.hip_torque * 0.5,
                 g_torques.knee_torque * 0.5,
                 support_force_total);

        const bool balance_quiet = std::abs(pitch_ - balance_offset_) < 0.22 &&
                                   std::abs(pitch_rate_) < 1.5 && std::abs(x_dot_) < 0.25 &&
                                   std::abs(x_ - target_x_) < 0.25;
        balance_settle_count_ = balance_quiet ? std::min(balance_settle_count_ + 1, 1000) : 0;

        if (post_landing_effort_support_)
        {
            // 验收判据：|pitch error|<0.04 rad, |pitch rate|<0.15 rad/s, |vx|<0.08 m/s, |vz|<0.03 m/s
            const bool steady_pitch = std::abs(pitch_ - balance_offset_) < 0.04;
            const bool steady_gyro = std::abs(pitch_rate_) < 0.15;
            const bool steady_vx = std::abs(x_dot_) < 0.08;
            const bool steady_vz = std::abs(current_z_dot_) < 0.03 ||
                                   (centroidal_velocity_valid_ && std::abs(centroidal_height_.velocity()) < 0.03);

            if (centroidal_lean_rate_.valid(this->now().seconds()) &&
                steady_pitch && steady_gyro && steady_vx && steady_vz)
            {
                post_jump_balance_stable_timer_ += dt;
                if (post_jump_balance_stable_timer_ >= 1.0 && !jump_summary_written_)
                {
                    const double now_sec = this->now().seconds();
                    balance_return_time_ = now_sec - balance_entry_time_;
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [BALANCE] 落地后在 Effort BALANCE 连续稳定 1.0s (return_time=%.3fs)，跳跃全流程验收成功！<<<",
                                balance_return_time_);
                    write_jump_summary(true, "");
                }
            }
            else
            {
                post_jump_balance_stable_timer_ = 0.0;
            }
        }
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
        if (!effort_mode_active_)
        {
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);
        }

        // PRE_JUMP 直接复用 BALANCE 已有的“速度外环 -> 俯仰参考 -> 姿态内环”结构。
        // 不再把 k_x / k_x_dot 直接作用到持续移动的 target_x_ 上：上一版中
        // 位置/速度状态反馈会压过 -target_speed 前馈，使 cmd 长时间保持正值，
        // 实测 x_dot 因而一直停留在 0 附近甚至反向，永远到不了 +0.20 m/s。
        // A second hop may leave BALANCE only after the existing COM capture
        // has continuously held the robot quiet for 0.50 s.  Missing/stale
        // world or joint data resets this timer; it never counts as readiness.
        const bool capture_ready = pre_jump_effort_capture_ && capture_world_valid_ &&
            centroidal_lean_rate_.valid(now_sec) &&
            centroidal_balance_.valid && centroidal_velocity_valid_ &&
            std::abs(ground_balance_angle()) < 0.10 &&
            std::abs(ground_balance_rate()) < 0.35 &&
            std::abs(capture_com_velocity_) < 0.08 &&
            std::abs(centroidal_height_.velocity()) < 0.03 &&
            std::abs(pitch_rate_) < 0.35 && std::abs(x_dot_) < 0.08;
        if (pre_jump_effort_capture_ && !pre_jump_capture_armed_)
        {
            pre_jump_capture_ready_timer_ = capture_ready ? pre_jump_capture_ready_timer_ + dt : 0.0;
            if (pre_jump_capture_ready_timer_ >= 0.50)
                pre_jump_capture_armed_ = true;
        }
        const bool second_hop_ready = !pre_jump_effort_capture_ || pre_jump_capture_armed_;
        const double commanded_speed = second_hop_ready ? jump_forward_speed_ : 0.0;
        const double target_speed_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < commanded_speed)
        {
            target_speed_smoothed_ = std::min(
                target_speed_smoothed_ + target_speed_step, commanded_speed);
        }
        else if (target_speed_smoothed_ > commanded_speed)
        {
            target_speed_smoothed_ = std::max(
                target_speed_smoothed_ - target_speed_step, commanded_speed);
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
        if (pre_jump_effort_capture_ && !pre_jump_capture_armed_)
            active_jump_pitch_ref_ = post_landing_pitch_ref_;
        else
            active_jump_pitch_ref_ = bbot_jump::clamp_value(
                balance_offset_ + kp_v * vel_error_v + ki_v * vel_integral_, -0.2, 0.2);
        if (pre_jump_effort_capture_ && pre_jump_capture_armed_)
            pre_jump_roll_in_elapsed_ += dt;
        // PRE_JUMP 仍要求近似零角速度，避免过早把机器人推入持续前倒。
        active_jump_pitch_rate_ref_ = 0.0;

        if (effort_mode_active_)
        {
            // Publish after this cycle's moving pitch reference is available so
            // the Effort torso allocator never applies a one-frame-old target.
            if (pre_jump_effort_capture_)
                publish_effort_height_control(pre_jump_hold_height_, 0.0,
                    K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                    25.0, 3.5, 45.0, 6.0, true);
            else
                publish_effort_height_control(pre_jump_hold_height_, 0.0,
                    450.0, 75.0, 220.0, 25.0, 4.0, 45.0, 6.0, true);
        }

        const double pitch_error = pitch_ - active_jump_pitch_ref_;
        const double pitch_rate_error = pitch_rate_ - active_jump_pitch_rate_ref_;
        const double u_pitch = -(
            current_gain_.k_theta * pitch_error +
            current_gain_.k_theta_dot * pitch_rate_error);

        double cmd_target = -u_pitch * cmd_scale_ - target_speed;
        cmd_target = bbot_jump::clamp_value(cmd_target, -1.20, 1.20);
        double cmd_x;
        if (pre_jump_effort_capture_)
        {
            const double desired = capture_world_valid_
                ? landing_capture_target(cmd_target) : last_wheel_cmd_x_;
            const double max_step = 5.0 * std::max(dt, 0.001);
            cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                desired - last_wheel_cmd_x_, -max_step, max_step);
        }
        else
        {
            const double max_cmd_step = 5.0 * std::max(dt, 0.001);
            cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        }
        publish_wheel_cmd(cmd_x, 0.0);

        const bool ready_now = second_hop_ready && bbot_jump::rolling_prepare_ready(
            x_dot_, jump_forward_speed_, target_speed_smoothed_,
            pitch_ - active_jump_pitch_ref_, pitch_rate_,
            pre_jump_speed_tolerance_, pre_jump_pitch_tolerance_, pre_jump_rate_limit_) &&
            centroidal_balance_.valid && centroidal_lean_rate_.valid(now_sec);
        pre_jump_stable_timer_ = ready_now ? (pre_jump_stable_timer_ + dt) : 0.0;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[PRE_JUMP] t=%.3f vx=%.3f/%.3f pitch=%.3f/%.3f gyro=%.3f xerr=%.3f stable=%.3f/%.3f cmd=%.3f",
            elapsed, x_dot_, jump_forward_speed_, pitch_, active_jump_pitch_ref_, pitch_rate_,
            pos_error, pre_jump_stable_timer_, pre_jump_stable_duration_, cmd_x);

        log_data(cmd_x, g_torques.hip_torque * 0.5,
                 g_torques.knee_torque * 0.5, TOTAL_MASS_ * 9.81);

        if (pre_jump_stable_timer_ >= pre_jump_stable_duration_)
        {
            // 进入 SQUAT 时重新锚定移动位置参考，只保留速度/姿态工作点，避免位置误差阶跃。
            target_x_ = x_;
            state_start_time_ = now_sec;
            current_state_ = bbot_jump::STATE_SQUAT;
            // 锁存速度外环已经建立的俯仰参考，不能在SQUAT首帧退回静态零点。
            squat_pitch_start_ref_ = active_jump_pitch_ref_;
            const double squat_start_height = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, L_MAX_);
            current_height_ = squat_start_height;
            squat_duration_current_ = pre_jump_effort_capture_ ? 1.30 * T_SQUAT_ : T_SQUAT_;
            quintic_traj_.init(now_sec, squat_duration_current_,
                               squat_start_height, 0.0, 0.0,
                               L_SQUAT_, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(),
                        ">>> PRE_JUMP 达标：vx=%.3f, pitch=%.3f, gyro=%.3f。进入 moving-SQUAT <<<",
                        x_dot_, pitch_, pitch_rate_);
            return;
        }

        const bool effort_capture_unsafe = pre_jump_effort_capture_ && pre_jump_capture_armed_ &&
            capture_world_valid_ &&
            (std::abs(ground_balance_angle()) > 0.18 ||
             std::abs(ground_balance_rate()) > 1.0 || std::abs(capture_com_velocity_) > 0.75);
        if (elapsed >= pre_jump_timeout_ || effort_capture_unsafe)
        {
            jump_failure_reason_ = effort_capture_unsafe
                ? "Effort PRE_JUMP COM捕获越界，安全中止"
                : "PRE_JUMP未能建立滚动起跳工作点";
            RCLCPP_WARN(this->get_logger(),
                        "[PRE_JUMP中止] %.2fs 内未稳定到目标 (vx=%.3f/%.3f pitch=%.3f/%.3f gyro=%.3f)，进入 RECOVERY 安全捕获",
                        elapsed, x_dot_, jump_forward_speed_, pitch_, active_jump_pitch_ref_, pitch_rate_);
            if (pre_jump_effort_capture_)
            {
                current_state_ = bbot_jump::STATE_RECOVERY;
                recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_STABILIZE;
                post_landing_effort_support_ = true;
                pre_jump_effort_capture_ = false;
                recovery_stable_timer_ = 0.0;
                recovery_time_ = now_sec;
                target_speed_const_ = 0.0;
                target_speed_smoothed_ = x_dot_;
            }
            else
            {
                // Preserve the original Position-mode gentle stop for a first hop.
                current_state_ = bbot_jump::STATE_BALANCE;
                pre_jump_balance_handoff_ = true;
                target_speed_const_ = 0.0;
            }
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
        constexpr double kIkRateLookahead = 0.010;
        const double z_rate_sample = bbot_jump::clamp_value(
            des_z + des_v * kIkRateLookahead, L_SQUAT_, L_MAX_);
        const auto ik_rate_sample = kinematics_.inverse_kinematics(z_rate_sample, 0.0);
        const double squat_hip_rate_ref = bbot_jump::clamp_value(
            (ik_rate_sample.theta_hip - ik_sq.theta_hip) / kIkRateLookahead, -3.0, 3.0);
        const double squat_knee_rate_ref = bbot_jump::clamp_value(
            (ik_rate_sample.theta_knee - ik_sq.theta_knee) / kIkRateLookahead, -3.0, 3.0);
        bbot_kinematics::JointTorques g_torques = kinematics_.compute_gravity_torques(0.0, ik_sq.theta_hip, ik_sq.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;
        if (!effort_mode_active_)
        {
            publish_position_leg_control(ik_sq.theta_hip, ik_sq.theta_knee);
        }

        // moving-SQUAT：PRE_JUMP 已经建立前向速度，此阶段只让起跳俯仰参考
        // 从准备末帧的俯仰参考平滑过渡到 jump_pitch_ref_。target_x_ 继续积分只用于诊断；
        // 不再使用 k_x / k_x_dot 反馈，否则持续移动参考会再次与速度前馈相互对抗。
        target_x_ += jump_forward_speed_ * dt;
        interpolate_lqr_gain();
        const double pos_error = x_ - target_x_;
        const double squat_progress = bbot_jump::clamp_value(
            elapsed / squat_duration_current_, 0.0, 1.0);
        active_jump_pitch_ref_ = bbot_jump::rolling_reference_blend(
            squat_pitch_start_ref_, jump_pitch_ref_, squat_progress);

        // v5.3：前 65% 下蹲只建立几何前倾；最后 35% 才平滑建立正的俯仰角速度。
        // 这样 THRUST 接管时机身已经“向前转”，而不是又被 D 项刹到 pitch_rate≈0。
        constexpr double rate_build_start = 0.65;
        const double rate_phase = bbot_jump::clamp_value(
            (squat_progress - rate_build_start) / (1.0 - rate_build_start), 0.0, 1.0);
        const double rate_blend = rate_phase * rate_phase * (3.0 - 2.0 * rate_phase);
        active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_ * rate_blend;
        // The Effort second-hop path may wait after the squat trajectory for
        // the torso to settle. Keep the angle target at its launch value, but
        // smoothly remove the positive angular-rate target during that dwell.
        // Otherwise the controller continues to drive forward while the gate
        // is waiting for a quiet takeoff configuration.
        if (pre_jump_effort_capture_ && elapsed >= squat_duration_current_)
        {
            const double dwell_blend = bbot_jump::clamp_value(
                (elapsed - squat_duration_current_) / 0.10, 0.0, 1.0);
            const double smooth_dwell = dwell_blend * dwell_blend * (3.0 - 2.0 * dwell_blend);
            active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_ * (1.0 - smooth_dwell);
        }

        if (effort_mode_active_)
        {
            // Keep support and torso allocation on the same dynamic reference
            // that drives the wheel controller during this SQUAT sample.
            support_force_total = publish_effort_height_control(
                des_z, des_v,
                550.0, 90.0, 220.0,
                25.0, 4.0, 45.0, 6.0,
                true, squat_hip_rate_ref, squat_knee_rate_ref);
        }

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
        if (pre_jump_effort_capture_ && capture_world_valid_)
            cmd_target = landing_capture_target(cmd_target);
        const double max_cmd_step = 6.0 * std::max(dt, 0.001);
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
        const bool traj_done = (elapsed >= squat_duration_current_);
        const bool settled = (std::abs(current_z_dot_) < 0.25 && std::abs(current_z_ - L_SQUAT_) < 0.060);
        // The retained Effort path enters THRUST with its torso still carrying
        // landing/rearm angular momentum. Do not release the launch gate while
        // that reference is still moving in the wrong direction. Preserve the
        // established Position-first gate and its prior tolerance.
        const double takeoff_rate_tolerance = pre_jump_effort_capture_
            ? std::min(jump_takeoff_pitch_rate_tolerance_, 0.10)
            : jump_takeoff_pitch_rate_tolerance_;
        const double takeoff_pitch_tolerance = pre_jump_effort_capture_
            ? 0.030
            : 0.080;
        const bool motion_ready =
            std::abs(x_dot_ - jump_forward_speed_) <= 0.12 &&
            std::abs(pitch_ - jump_pitch_ref_) <= takeoff_pitch_tolerance &&
            std::abs(pitch_rate_ - active_jump_pitch_rate_ref_) <=
                takeoff_rate_tolerance;
        const bool forced_ready = (elapsed >= squat_duration_current_ + 0.15 &&
                                   std::abs(current_z_ - L_SQUAT_) < 0.10 &&
                                   motion_ready);
        // Give the strict Effort continuation attitude gate a bounded extra
        // settling interval after the squat trajectory; the gate itself is
        // unchanged, and Position-first keeps its original timeout.
        const double squat_timeout_after_trajectory = pre_jump_effort_capture_ ? 1.20 : 0.35;
        const bool squat_timeout = (elapsed >= squat_duration_current_ + squat_timeout_after_trajectory);

        if (((traj_done && settled) && motion_ready) || forced_ready)
        {
            RCLCPP_INFO(this->get_logger(), ">>> 蓄力完成%s (t=%.3fs, z=%.3f, v=%.2f)！启动阶段 2：全力爆发弹射推地 (THRUST)... <<<",
                        forced_ready && !settled ? "（使用可控速度兜底）" : "",
                        elapsed, current_z_, current_z_dot_);
            current_state_ = bbot_jump::STATE_THRUST;
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            thrust_trajectory_initialized_ = false;
            thrust_reference_seed_pending_ = false;
            thrust_velocity_reference_ = {};
            thrust_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
            thrust_force_command_per_leg_ = thrust_force_per_leg_;
            velocity_reached_count_ = 0;
            thrust_release_.reset();
            thrust_motion_elapsed_ = 0.0;
            thrust_extension_scale_ = 1.0;
        thrust_attitude_blocked_ = true;
            thrust_gate_has_opened_ = false;
            thrust_attitude_stable_duration_.reset();
            thrust_block_recovery_count_ = 0;
            thrust_block_elapsed_ = 0.0;
            active_jump_pitch_rate_ref_ = jump_takeoff_pitch_rate_;
            // 保留 moving-SQUAT 的最后轮速命令，THRUST 不得人为制造水平速度阶跃。

            // Position-first must prewrite gravity support before its async
            // controller switch.  An Effort continuation is already active;
            // replacing its COM/support allocation with a generic zero-speed
            // 80/8 command for one frame would break continuity.
            effort_support_handoff_preserved_ = bbot_jump::preserve_effort_support_handoff(
                pre_jump_effort_capture_, effort_mode_active_);
            if (!effort_support_handoff_preserved_)
            {
                publish_effort_leg_control(ik_sq.theta_hip, ik_sq.theta_knee, 0.0, 0.0,
                                           -g_torques.hip_torque * 0.5, -g_torques.knee_torque * 0.5,
                                           80.0, 8.0, 80.0, 8.0);
                request_effort_controller();
            }

            prev_q_hip_des_ = ik_sq.theta_hip;
            prev_q_knee_des_ = ik_sq.theta_knee;
        }
        else if (squat_timeout)
        {
            // Report the gate that actually timed out.  Height can be settled
            // while the strict attitude/speed gate remains closed.
            std::string reason = "SQUAT timeout";
            if (!settled)
                reason += " height(z=" + std::to_string(current_z_) +
                          ",zdot=" + std::to_string(current_z_dot_) + ")";
            if (std::abs(x_dot_ - jump_forward_speed_) > 0.12)
                reason += " vx_error=" + std::to_string(x_dot_ - jump_forward_speed_);
            if (std::abs(pitch_ - jump_pitch_ref_) > takeoff_pitch_tolerance)
                reason += " pitch_error=" + std::to_string(pitch_ - jump_pitch_ref_);
            if (std::abs(pitch_rate_ - active_jump_pitch_rate_ref_) > takeoff_rate_tolerance)
                reason += " pitch_rate_error=" + std::to_string(pitch_rate_ - active_jump_pitch_rate_ref_);
            abort_jump_to_recovery(now_sec, reason.c_str());
        }
    }

    // ── 阶段 2：爆发推地 (THRUST) ──
    void run_state_thrust(double now_sec, double dt)
    {
        contact_takeoff_confirmed_diag_ = false;
        reference_ground_pulse_guard_diag_ =
            static_cast<int>(bbot_jump::ReferenceGroundPulseGuard::WrongPhase);
        reference_ground_pulse_gate_elapsed_diag_ = 0.0;
        reference_ground_pulse_shape_diag_ = 0.0;
        reference_ground_pulse_requested_left_diag_ = 0.0;
        reference_ground_pulse_requested_right_diag_ = 0.0;
        reference_ground_pulse_applied_left_diag_ = 0.0;
        reference_ground_pulse_applied_right_diag_ = 0.0;
        contact_takeoff_observer_.refresh_source(this->now().nanoseconds());
        const auto output_action = bbot_jump::thrust_output_action(
            thrust_effort_owned_, effort_mode_active_, leg_mode_switch_pending_);
        if (thrust_output_locked_ ||
            output_action == bbot_jump::ThrustOutputAction::LockAfterHandoff) {
            thrust_output_locked_ = true;
            lock_owned_outputs(leg_mode_switch_pending_ ?
                "effort_controller_switch_pending_after_ownership" :
                "effort_mode_lost_after_ownership");
            return;
        }
        if (output_action == bbot_jump::ThrustOutputAction::AwaitInitialEffort)
        {
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
        thrust_effort_owned_ = true;

        if (!thrust_trajectory_initialized_)
        {
            double ground_geom_left = 0.0, ground_geom_right = 0.0;
            if (!aligned_takeoff_geometry(now_sec, ground_geom_left, ground_geom_right))
            {
                // 保持切换前已经写入的支撑力矩，等待最近odom时刻的关节样本。
                // 不能用不同时间的几何标定地面，否则整个跳跃带着固定偏差。
                if (now_sec - state_start_time_ > 0.20)
                    abort_jump_to_recovery(now_sec, "起跳缺少可对齐的里程计/关节时间戳");
                return;
            }
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            thrust_start_z_ = current_z_;
            thrust_start_world_z_ = gazebo_world_z_;
            // 接地时标定世界高度与 FK 的常量偏差；FK 只用于几何间隙。
            ground_height_offset_ = gazebo_world_z_ - 0.5 * (ground_geom_left + ground_geom_right);
            takeoff_confirmation_.reset();
            takeoff_speed_latch_.reset();
            takeoff_rise_observed_ = false;
            airborne_confidence_count_ = 0;
            last_world_descent_time_ = -1.0;
            contact_window_ = false;
            max_world_z_during_jump_ = gazebo_world_z_;
            quintic_traj_.init(0.0, thrust_duration_, current_z_, 0.0, 0.0,
                               H_TAKEOFF_, 0.8, 0.0);
            // SQUAT's final IK target and this newly initialized THRUST path's
            // actual starting height are different references. Seed the first
            // thrust IK before taking any finite difference across phases.
            const auto thrust_seed_ik = kinematics_.inverse_kinematics(current_z_, 0.0);
            prev_q_hip_des_ = thrust_seed_ik.theta_hip;
            prev_q_knee_des_ = thrust_seed_ik.theta_knee;
            thrust_reference_seed_pending_ = true;
            thrust_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
            thrust_force_command_per_leg_ = thrust_force_per_leg_;
            velocity_reached_count_ = 0;
            thrust_motion_elapsed_ = 0.0;
            thrust_extension_scale_ = 1.0;
            thrust_attitude_blocked_ = true;
            // 这里仍是 THRUST 初始姿态 gate，而不是运行中的硬故障 block。
            thrust_gate_has_opened_ = false;
            thrust_attitude_stable_duration_.reset();
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
        thrust_gate_pitch_err_diag_ = pitch_err;
        thrust_gate_rate_err_diag_ = pitch_rate_err;
        const double balance_pitch_err = pitch_ - balance_offset_;

        // 门控围绕“目标正俯仰角速度”判断，而不是要求角速度接近 0。
        constexpr double thrust_gate_pitch_limit = 0.08;
        constexpr double thrust_gate_duration = 0.020;
        const bool attitude_stable =
            (std::abs(pitch_err) <= thrust_gate_pitch_limit &&
             std::abs(pitch_rate_err) <= jump_takeoff_pitch_rate_tolerance_);
        const int64_t stable_elapsed_ns = thrust_attitude_stable_duration_.update(
            attitude_stable, this->now().nanoseconds());
        const bool stable_duration_met =
            stable_elapsed_ns >= static_cast<int64_t>(thrust_gate_duration * 1.0e9);
        thrust_gate_stable_elapsed_diag_ = static_cast<double>(stable_elapsed_ns) * 1.0e-9;

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
            (pitch_err > 0.16 || balance_pitch_err > 0.28 || pitch_rate_ > 1.60);
        // 运行中 hard block 的解锁条件不再要求重新追到 +0.45 rad/s；
        // 只要回到可恢复的近零角速度区间即可继续，避免“一旦 block 就永远解不开”。
        const bool hard_block_recovery_ok =
            (std::abs(pitch_err) <= 0.10 &&
             pitch_rate_ > -0.35 && pitch_rate_ < 0.80 &&
             balance_pitch_err > -0.03 && balance_pitch_err < 0.24);

        if (thrust_attitude_blocked_)
        {
            thrust_block_elapsed_ += dt;
            if (!thrust_gate_has_opened_)
            {
                // THRUST 首次启动仍使用原来的动态起跳工作点 gate。
                if (thrust_block_elapsed_ > 0.20)
                {
                    abort_jump_to_recovery(now_sec, "THRUST初始姿态门控超过0.20s");
                    return;
                }
                if (stable_duration_met)
                {
                    thrust_attitude_blocked_ = false;
                    thrust_gate_has_opened_ = true;
                    thrust_gate_open_stamp_ = now_sec;
                    contact_takeoff_observer_.open_gate(
                        static_cast<int64_t>(std::llround(now_sec * 1.0e9)));
                    thrust_block_elapsed_ = 0.0;
                    thrust_block_recovery_count_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                                "[推地门控] 动态起跳姿态达标 (pitch=%.3f ref=%.3f err=%.3f "
                                "gyro=%.3f/%.3f)，推进推地轨迹 (motion_t=%.3f s)",
                                pitch_, active_jump_pitch_ref_, pitch_err,
                                pitch_rate_, active_jump_pitch_rate_ref_, thrust_motion_elapsed_);
                }
            }
            else
            {
                // 运行中的硬阻塞采用更宽松、物理上可恢复的解锁条件。
                if (hard_block_recovery_ok)
                {
                    thrust_block_recovery_count_++;
                }
                else
                {
                    thrust_block_recovery_count_ = 0;
                }
                if (thrust_block_recovery_count_ * dt >= 0.015)
                {
                    thrust_attitude_blocked_ = false;
                    thrust_block_elapsed_ = 0.0;
                    thrust_block_recovery_count_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                                "[推地门控] 硬姿态阻塞已恢复 (pitch=%.3f err=%.3f gyro=%.3f)，继续推地",
                                pitch_, pitch_err, pitch_rate_);
                }
                else if (thrust_block_elapsed_ > thrust_hard_block_timeout_)
                {
                    abort_jump_to_recovery(now_sec, "推地硬姿态阻塞超时");
                    return;
                }
            }
        }
        else
        {
            if (hard_backward_tendency || forward_tendency_excess)
            {
                thrust_attitude_blocked_ = true;
                thrust_block_elapsed_ = 0.0;
                thrust_attitude_stable_duration_.reset();
                thrust_block_recovery_count_ = 0;
                RCLCPP_WARN(this->get_logger(),
                            "[推地门控] 硬姿态趋势超限 backward=%d forward=%d "
                            "(pitch=%.3f jump_err=%.3f balance_err=%.3f gyro=%.3f)，短时冻结推地",
                            hard_backward_tendency ? 1 : 0, forward_tendency_excess ? 1 : 0,
                            pitch_, pitch_err, balance_pitch_err, pitch_rate_);
            }
            else
            {
                // soft backward 不冻结 motion time。它只是降低当前额外推力，
                // 让姿态控制获得时间纠正，同时避免机器人在接地状态“卡死成一条长腿”。
                thrust_motion_elapsed_ += dt;
                if (soft_backward_rate)
                {
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
        const double vertical_velocity = centroidal_velocity_valid_ ? centroidal_height_.velocity() : 0.0;
        thrust_vertical_velocity_ = vertical_velocity;
        const bool thrust_speed_armed = thrust_gate_has_opened_ &&
                                        thrust_motion_elapsed_ > 0.0;
        if (takeoff_speed_latch_.update_with_state(
                centroidal_height_.stamp(), now_sec, thrust_speed_armed,
                vertical_velocity, target_takeoff_velocity_,
                centroidal_height_.height(),
                capture_world_valid_ ? capture_com_velocity_ : x_dot_))
        {
            com_takeoff_latched_vz_ = takeoff_speed_latch_.velocity();
            velocity_takeoff_reached_ = true;
            if (actual_takeoff_velocity_ <= 0.0)
                actual_takeoff_velocity_ = com_takeoff_latched_vz_;
        }
        const double velocity_error = target_takeoff_velocity_ - vertical_velocity;
        const bool grounded_after_nominal_stroke =
            thrust_motion_elapsed_ >= thrust_duration_;
        const double feedback_force = centroidal_velocity_valid_ ? mass_per_leg * thrust_velocity_kp_ * velocity_error : 0.0;
        double extra_force = base_force * (thrust_peak_ratio_ - 1.0) * shape;
        const double feedback_ramp = bbot_jump::clamp_value(thrust_motion_elapsed_ / 0.06, 0.0, 1.0);
        double fb_force = feedback_ramp * feedback_force;

        // 推地末段仅在真实大后仰时平滑削减额外推力，避免为追速度恶化倾角，同时防止正常伸腿小扰动误清推力
        if (s > 0.4)
        {
            double penalty = 0.0;
            if (pitch_err < -0.04)
                penalty += (-pitch_err - 0.04) / 0.08;
            if (pitch_rate_ < -0.50)
                penalty += (-pitch_rate_ - 0.50) / 0.80;
            // 前倾同样需要削减额外冲量，不能等到硬阻塞阈值才处理。
            if (pitch_err > 0.06)
                penalty += (pitch_err - 0.06) / 0.08;
            if (pitch_rate_err > 0.40)
                penalty += (pitch_rate_err - 0.40) / 0.80;
            double thrust_pitch_attenuation = bbot_jump::clamp_value(1.0 - penalty, 0.2, 1.0);
            // soft-backward 只把额外推力降到约 45~70%，绝不把推进状态机锁住。
            if (soft_backward_rate)
            {
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
        thrust_release_.update(now_sec, s, vertical_velocity, target_takeoff_velocity_,
                               thrust_force_per_leg_, thrust_release_velocity_ratio_);
        const double terminal_brake_blend = thrust_release_.brake_blend(
            now_sec, speed_brake_blend * phase_blend * knee_speed_u);

        double F_z_request = base_force;
        if (!thrust_attitude_blocked_)
        {
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
                velocity_error > 0.0 && !thrust_release_.active())
            {
                const double speed_deficit = bbot_jump::clamp_value(
                    velocity_error / std::max(0.20, target_takeoff_velocity_),
                    0.70, 1.0);
                F_z_request += base_force * grounded_launch_boost_ratio_ *
                               speed_deficit;
            }
        }
        else
        {
            // 出现后仰趋势阻塞时：若机身已经在高速上升 (z_dot > 0.8)，
            // 立即削减支撑力，促使干净离地，切断地面反作用力对机身持续注入的后仰力矩
            if (vertical_velocity > 0.60)
            {
                const double unload_factor = bbot_jump::clamp_value(1.0 - thrust_block_elapsed_ / 0.03, 0.0, 1.0);
                F_z_request *= unload_factor;
            }
        }

        // 速度达到目标后撤去额外推力；保留小于重力的短暂支撑，避免末端
        // 力矩突变，下一状态由腾空腿部轨迹接管。
        if (centroidal_velocity_valid_ && velocity_error <= 0.0)
        {
            F_z_request = base_force;
        }

        // 轨迹完成后不再按时间卸载。若轮子仍接地，维持速度反馈推力；
        // 只有连续失重确认后才能退出 THRUST，避免产生“假腾空”。

        const double travel_scale = std::min({bbot_jump::joint_extension_scale(hip_pos_left_, hip_vel_left_),
                                              bbot_jump::joint_extension_scale(knee_pos_left_, knee_vel_left_),
                                              bbot_jump::joint_extension_scale(hip_pos_right_, hip_vel_right_),
                                              bbot_jump::joint_extension_scale(knee_pos_right_, knee_vel_right_)});
        if (travel_scale < thrust_extension_scale_)
        {
            if (thrust_extension_scale_ == 1.0)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[末端行程保护] 提前卸力 scale=%.2f hip=%.3f/%.2f knee=%.3f/%.2f",
                            travel_scale, hip_pos_left_, hip_vel_left_, knee_pos_left_, knee_vel_left_);
            }
            thrust_extension_scale_ = travel_scale;
        }
        F_z_request *= thrust_extension_scale_;
        if (thrust_release_.active())
            F_z_request = std::min({F_z_request, thrust_release_.force_limit(now_sec), thrust_force_per_leg_});

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
        const double qdh_nominal = grounded_thrust_hold ? bbot_jump::thrust_extension_velocity(hip_vel_left_, 3.0, 1.0, thrust_extension_scale_) : bbot_jump::sampled_joint_reference_rate(
            q_hip_des, prev_q_hip_des_, dt, thrust_extension_scale_, 3.0,
            thrust_reference_seed_pending_);
        const double qdk_nominal = grounded_thrust_hold ? bbot_jump::thrust_extension_velocity(knee_vel_left_, 6.0, -1.0, thrust_extension_scale_) : bbot_jump::sampled_joint_reference_rate(
            q_knee_des, prev_q_knee_des_, dt, thrust_extension_scale_, 6.0,
            thrust_reference_seed_pending_);
        // 速度释放事件通常比几何离地确认早数十毫秒。达到目标速度后推力已经
        // 单调卸载，此时必须同步刹住残余伸腿速度；若仍等到 wheels_airborne，
        // 膝会在确认窗口内撞到行程端点，并把高速髋带入 TUCK。
        const double grounded_brake_factor = bbot_jump::thrust_brake_factor(
            last_wheels_airborne_, thrust_release_.active(), terminal_brake_blend);
        const double hip_extension_velocity_scale =
            bbot_jump::thrust_hip_extension_velocity_scale(
                pre_jump_effort_capture_, grounded_brake_factor);
        double qdh = hip_extension_velocity_scale * qdh_nominal;
        const double legacy_qdk = (1.0 - 0.75 * grounded_brake_factor) * qdk_nominal;
        const double q_hip_des_r = grounded_thrust_hold ? hip_pos_right_ : q_hip_des;
        const double q_knee_des_r = grounded_thrust_hold ? knee_pos_right_ : q_knee_des;
        const double qdh_nominal_r = grounded_thrust_hold
                                         ? bbot_jump::thrust_extension_velocity(hip_vel_right_, 3.0, 1.0,
                                                                                thrust_extension_scale_)
                                         : qdh_nominal;
        double qdh_r = hip_extension_velocity_scale * qdh_nominal_r;
        const double qdh_for_knee_reference = bbot_jump::thrust_knee_reference_velocity_scale(
                                                  grounded_brake_factor) * qdh_nominal;
        const double qdh_r_for_knee_reference = bbot_jump::thrust_knee_reference_velocity_scale(
                                                     grounded_brake_factor) * qdh_nominal_r;
        const double legacy_qdk_r = grounded_thrust_hold ? (1.0 - 0.75 * grounded_brake_factor) *
                                                               bbot_jump::thrust_extension_velocity(knee_vel_right_, 6.0, -1.0, thrust_extension_scale_)
                                                         : legacy_qdk;
        double qdk = legacy_qdk, qdk_r = legacy_qdk_r;
        const bool fresh_thrust_geometry = centroidal_velocity_valid_ &&
                                           now_sec >= joint_sample_time_ && now_sec - joint_sample_time_ <= 0.080 &&
                                           torso_imu_.fresh(now_sec);
        if (!thrust_attitude_blocked_ && fresh_thrust_geometry)
        {
            // 用与竖直力反馈相同的目标速度反解膝参考，而不是沿用固定
            // H_TAKEOFF_ 的 IK 差分速度（约 -3 rad/s）。后者会在 COM 仍处
            // 1.44/1.98 m/s 时把 -10 rad/s 的膝当误差刹掉。
            thrust_velocity_reference_ = bbot_jump::thrust_velocity_reference(
                {hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_},
                pitch_, body_mass_, feedback_ramp * target_takeoff_velocity_,
                // Keep the knee inverse on the original extension reference
                // while the Effort hip PD independently brakes its velocity
                // target.  This isolates the hip release effect from the
                // knee-Jacobian velocity inversion.
                qdh_for_knee_reference, qdh_r_for_knee_reference,
                active_jump_pitch_rate_ref_, thrust_knee_velocity_limit_);
            if (thrust_velocity_reference_.valid)
            {
                qdk = bbot_jump::protected_thrust_knee_velocity(
                    thrust_velocity_reference_.knee_velocity,
                    thrust_extension_scale_, grounded_brake_factor);
                qdk_r = qdk;
            }
        }
        // The experimental inverse preserves the existing supported COM-velocity
        // task, including its travel/release protection. It only operates on
        // freshly paired IMU/joints and real bilateral contact. This is a
        // velocity reference; the resulting external momentum change still
        // needs physical validation.
        const double nominal_qdh = qdh, nominal_qdh_r = qdh_r;
        const bbot_jump::MomentumThrustScope momentum_scope{
            thrust_momentum_reference_enable_, true, thrust_gate_has_opened_,
            thrust_attitude_blocked_, effort_mode_active_, leg_mode_switch_pending_,
            reference_ground_pulse_contact_valid_, reference_ground_pulse_contact_continuous_,
            reference_ground_pulse_wheel_mask_ == 0x3, landing_momentum_diag_.valid,
            now_sec, reference_ground_pulse_contact_stamp_ns_ * 1e-9,
            landing_momentum_stamp_diag_, thrust_motion_elapsed_};
        const double momentum_blend = bbot_jump::momentum_thrust_blend(momentum_scope);
        bbot_jump::LandingJointSample momentum_sample;
        if (momentum_blend > 0.0 &&
            landing_joint_history_.exact(torso_imu_.stamp(), now_sec, momentum_sample))
        {
            const auto & mq = momentum_sample.q;
            const auto & mv = momentum_sample.v;
            const auto geometry = bbot_jump::centroidal_geometry({mq[0],mq[1],mq[2],mq[3]},body_mass_);
            const Eigen::Vector3d vertical(0,-std::sin(pitch_),std::cos(pitch_));
            const Eigen::RowVector4d j = vertical.transpose()*(geometry.com_jacobian-geometry.axle_jacobian);
            const auto relative = bbot_jump::rotate_about_hip(-pitch_,geometry.com-geometry.axle);
            const double nominal_vz = j[0]*qdh+j[1]*qdk+j[2]*qdh_r+j[3]*qdk_r-
                relative.y()*pitch_rate_raw_;
            const std::array<double,2> absolute_wheel_rates{
                -pitch_rate_raw_+mv[0]+mv[1]+momentum_sample.wheel_rate[0],
                -pitch_rate_raw_+mv[2]+mv[3]+momentum_sample.wheel_rate[1]};
            const auto reference = bbot_jump::momentum_thrust_reference(
                mq,pitch_,pitch_rate_raw_,absolute_wheel_rates,body_mass_,nominal_vz,.55,
                flight_hip_speed_limit_,thrust_knee_velocity_limit_);
            if (reference.valid)
            {
                const double new_qdh=(1-momentum_blend)*qdh+momentum_blend*reference.hip_velocity;
                const double new_qdh_r=(1-momentum_blend)*qdh_r+momentum_blend*reference.hip_velocity;
                const double new_qdk=(1-momentum_blend)*qdk+momentum_blend*reference.knee_velocity;
                const double new_qdk_r=(1-momentum_blend)*qdk_r+momentum_blend*reference.knee_velocity;
                // Preserve all existing terminal/travel constraints after interpolation.
                // Do not replace a rejected inverse with a clipped solution.
                const bool travel_allowed = thrust_extension_scale_ >= 1.0 && !thrust_release_.active();
                if (travel_allowed)
                {
                    thrust_momentum_reference_active_diag_=true;
                    thrust_momentum_reference_blend_diag_=momentum_blend;
                    thrust_momentum_reference_vz_diag_=nominal_vz;
                    thrust_momentum_reference_h_diag_=.55;
                    thrust_momentum_reference_hip_delta_diag_=new_qdh-qdh;
                    thrust_momentum_reference_knee_delta_diag_=new_qdk-qdk;
                    qdh=new_qdh;qdh_r=new_qdh_r;qdk=new_qdk;qdk_r=new_qdk_r;
                }
            }
        }

        if (thrust_support_coordination_enable_ &&
            current_state_ == bbot_jump::STATE_THRUST)
        {
            bbot_jump::LandingJointSample support_sample;
            const bool joint_pair = landing_joint_history_.exact(
                torso_imu_.stamp(), now_sec, support_sample);
            const bool com_pair = joint_pair && capture_world_valid_ &&
                centroidal_velocity_valid_ && centroidal_height_.valid(now_sec) &&
                centroidal_world_.valid(now_sec) &&
                std::abs(centroidal_height_.stamp()-centroidal_world_.stamp()) <= 0.001+1.0e-9 &&
                now_sec-centroidal_height_.stamp() <= 0.020+1.0e-9 &&
                now_sec-centroidal_world_.stamp() <= 0.020+1.0e-9 &&
                std::abs(centroidal_height_.stamp()-torso_imu_.stamp()) <= 0.010+1.0e-9 &&
                std::abs(centroidal_height_.stamp()-support_sample.stamp) <= 0.010+1.0e-9;
            double nominal_vz = 0.0;
            if (joint_pair)
            {
                const auto & q = support_sample.q;
                const auto geom = bbot_jump::centroidal_geometry(
                    {q[0], q[1], q[2], q[3]}, body_mass_);
                const Eigen::Vector3d vertical(0.0, -std::sin(pitch_), std::cos(pitch_));
                const Eigen::RowVector4d jz = vertical.transpose()*
                    (geom.com_jacobian-geom.axle_jacobian);
                const auto rel = bbot_jump::rotate_about_hip(-pitch_, geom.com-geom.axle);
                const std::array<double,4> nominal{qdh,qdk,qdh_r,qdk_r};
                Eigen::Map<const Eigen::Vector4d> rates(nominal.data());
                nominal_vz = jz.dot(rates)-rel.y()*pitch_rate_raw_;
            }
            const double imu_stamp = torso_imu_.stamp();
            const double joint_stamp = joint_pair ? support_sample.stamp : 0.0;
            const double com_stamp = com_pair ? centroidal_height_.stamp() : 0.0;
            std::array<double,4> support_q{};
            if (joint_pair) {
                for (std::size_t i=0;i<support_q.size();++i)
                    support_q[i] = support_sample.q[static_cast<Eigen::Index>(i)];
            }
            thrust_support_imu_stamp_diag_ = imu_stamp;
            thrust_support_joint_stamp_diag_ = joint_stamp;
            thrust_support_com_stamp_diag_ = com_stamp;
            const bbot_jump::ThrustSupportInput support_input{
                thrust_support_coordination_enable_,
                current_state_ == bbot_jump::STATE_THRUST,
                thrust_gate_has_opened_ && thrust_motion_elapsed_ > 0.0,
                effort_mode_active_, leg_mode_switch_pending_, thrust_attitude_blocked_,
                thrust_extension_scale_ >= 1.0 && !thrust_release_.active(),
                reference_ground_pulse_contact_valid_,
                reference_ground_pulse_contact_continuous_,
                reference_ground_pulse_wheel_mask_, com_pair,
                now_sec, reference_ground_pulse_contact_stamp_ns_*1.0e-9,
                imu_stamp, joint_stamp, com_stamp, thrust_motion_elapsed_,
                support_q,
                {qdh,qdk,qdh_r,qdk_r}, pitch_, pitch_rate_raw_, body_mass_,
                nominal_vz, 0.010, 3.0, 0.30,
                flight_hip_speed_limit_, thrust_knee_velocity_limit_};
            thrust_support_reference_diag_ =
                bbot_jump::thrust_support_coordination(support_input);
            if (thrust_support_reference_diag_.active)
            {
                qdh = thrust_support_reference_diag_.qdot[0];
                qdk = thrust_support_reference_diag_.qdot[1];
                qdh_r = thrust_support_reference_diag_.qdot[2];
                qdk_r = thrust_support_reference_diag_.qdot[3];
            }
        }

        if (thrust_reference_handoff_enable_)
        {
            const bool support = bbot_jump::handoff_support_allowed(
                reference_ground_pulse_contact_valid_, reference_ground_pulse_contact_continuous_,
                reference_ground_pulse_wheel_mask_, now_sec,
                reference_ground_pulse_contact_stamp_ns_ * 1e-9);
            handoff_force_suppressed_diag_ = !support;
            // A remembered velocity target is an internal servo reference,
            // never a stale contact force. The force ramp restarts from zero.
            if (!support) { F_z_request = 0.; thrust_force_per_leg_ = 0.; }
            const bool fresh = now_sec >= joint_sample_time_ && now_sec-joint_sample_time_ <= .020 &&
                now_sec >= torso_imu_.stamp() && now_sec-torso_imu_.stamp() <= .020;
            const std::array<double, 4> target{qdh,qdk,qdh_r,qdk_r};
            const double blend = thrust_momentum_reference_blend_diag_;
            const std::array<double, 2> feedback{
                blend * bbot_jump::clamp_value(kd_hip_thrust*(qdh-hip_vel_left_),-4.,4.),
                blend * bbot_jump::clamp_value(kd_hip_thrust*(qdh_r-hip_vel_right_),-4.,4.)};
            const auto handoff = thrust_reference_handoff_.update(now_sec, target,
                {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_}, feedback,
                fresh, effort_mode_active_ && !leg_mode_switch_pending_,
                thrust_extension_scale_ >= 1. && bbot_jump::handoff_actual_travel_safe(
                    {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_},
                    {hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_}),
                thrust_momentum_reference_active_diag_);
            handoff_reference_mode_diag_ = !support ? 3 : (thrust_release_.active() ? 2 : 1);
            handoff_reference_valid_diag_ = handoff.valid;
            handoff_reference_limited_diag_ = handoff.limited;
            handoff_reference_target_diag_ = target;
            handoff_reference_applied_diag_ = handoff.velocity;
            handoff_reference_rate_diag_ = handoff.rate;
            if (!handoff.valid)
            {
                handoff_reference_mode_diag_ = 6;
                abort_jump_to_recovery(now_sec, "实验交接参考数据/行程不可行");
                return;
            }
            qdh=handoff.velocity[0]; qdk=handoff.velocity[1];
            qdh_r=handoff.velocity[2]; qdk_r=handoff.velocity[3];
            thrust_momentum_reference_hip_feedback_l_diag_=handoff.hip_feedback[0];
            thrust_momentum_reference_hip_feedback_r_diag_=handoff.hip_feedback[1];
        }

        velocity_capture_reference_active_ = false;
        const std::array<double,4> capture_q{hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_};
        const std::array<double,4> capture_v{hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_};
        const auto capture_pair=velocity_capture_history_.paired(now_sec);
        const bool capture_fresh=capture_pair.valid && now_sec>=joint_sample_time_ &&
            now_sec-joint_sample_time_<=0.020 && now_sec>=torso_imu_.stamp() && now_sec-torso_imu_.stamp()<=0.020 &&
            centroidal_velocity_valid_ && centroidal_height_.valid(now_sec) &&
            now_sec>=centroidal_height_.stamp() && now_sec-centroidal_height_.stamp()<=0.020;
        const bool capture_support=bbot_jump::handoff_support_allowed(reference_ground_pulse_contact_valid_,
            reference_ground_pulse_contact_continuous_,reference_ground_pulse_wheel_mask_,
            now_sec,reference_ground_pulse_contact_stamp_ns_*1e-9);
        const double capture_latency=velocity_capture_delay_+std::max(0.0,now_sec-joint_sample_time_)+dt;
        const auto capture_stop=bbot_jump::velocity_capture_stopping_distance(capture_q,capture_v,
            velocity_capture_hip_decel_,velocity_capture_knee_decel_,capture_latency);
        bbot_jump::VelocityLaunchCaptureReference capture_inverse;
        if (velocity_capture_enable_ && capture_fresh) {
            const double rate_target=bbot_jump::clamp_value(
                0.05+1.5*(balance_offset_+flight_landing_pitch_bias_-capture_pair.pitch),-0.15,0.15);
            capture_inverse=bbot_jump::velocity_launch_capture_reference(capture_pair.joints.q,capture_pair.pitch,
                rate_target,thrust_release_.active()?std::max(0.0,vertical_velocity):feedback_ramp*target_takeoff_velocity_,
                0.08,capture_pair.joints.wheel_rate,body_mass_,flight_hip_speed_limit_,thrust_knee_velocity_limit_);
        }
        const std::array<double,4> capture_requested{capture_inverse.hip_velocity,capture_inverse.knee_velocity,
            capture_inverse.hip_velocity,capture_inverse.knee_velocity};
        bbot_jump::VelocityCaptureSessionSample capture_session;
        if (velocity_capture_enable_) {
            capture_session=velocity_capture_session_.update(now_sec,
                thrust_gate_has_opened_&&thrust_motion_elapsed_>0,capture_fresh,capture_support,
                effort_mode_active_&&!leg_mode_switch_pending_,thrust_attitude_blocked_,capture_inverse.valid,
                capture_requested,capture_v,capture_stop);
            // Ownership, rather than one inverse result, chooses the output path.
            velocity_capture_reference_active_=capture_session.owns_command;
            if (capture_session.owns_command) {
                qdh=capture_session.reference[0];qdk=capture_session.reference[1];
                qdh_r=capture_session.reference[2];qdk_r=capture_session.reference[3];
                if(capture_session.phase==bbot_jump::VelocityCapturePhase::Braking) {
                    F_z_request=0.0;thrust_force_per_leg_=0.0;
                }
            }
        }
        const bool capture_braking=capture_session.phase==bbot_jump::VelocityCapturePhase::Braking;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
            "[VELOCITY_CAPTURE] enabled=%d owner=%d phase=%d reason=%d inverse=%d pair=%.3f vh=%.3f vk=%.3f H=%.3f vz=%.3f",
            velocity_capture_enable_,capture_session.owns_command,static_cast<int>(capture_session.phase),
            static_cast<int>(capture_session.reason),capture_inverse.valid,capture_pair.joints.stamp,
            qdh,qdk,landing_momentum_diag_.total,vertical_velocity);
        prev_q_hip_des_ = q_hip_des;
        prev_q_knee_des_ = q_knee_des;
        thrust_reference_seed_pending_ = false;

        // 试验/调参前馈：当前模型中推地伸展对应 qdk<0。仅依据“将要发生的”
        // 膝伸展速度提前提供小幅正向髋补偿，抵消腿部内部反作用；有独立限幅，
        // 且在姿态门控阻塞时不允许它替代减推力保护。
        // 试验/调参值：前馈读的是"实际会被执行"的 qdk 而不是 legacy_qdk。v6.13 把膝
        // 参考速度从 -6 抬到 -8~-11.6，仍按 legacy 给就会低估反作用，实测离地
        // pitch_rate 从 -0.25 恶化到 -0.6~-0.96。
        const double knee_extension_speed = std::max(0.0, -qdk);
        last_thrust_reaction_ff_ = bbot_jump::clamp_value(
            K_LEG_REACTION_FF_THRUST_ * knee_extension_speed,
            0.0, TAU_LEG_REACTION_FF_MAX_);
        double tau_body_per_hip = bbot_jump::clamp_value(
            -0.5 * (K_BODY_P_THRUST_ * pitch_err +
                    K_BODY_D_THRUST_ * pitch_rate_err) +
                last_thrust_reaction_ff_,
            -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);

        // 髋关节在推地阶段的核心任务是姿态稳定(tau_body_per_hip)，构型保持只做低增益补偿；
        // 尤其在后仰趋势时，禁止髋关节PD产生负向拉扯力矩加剧后仰。
        const double pd_h_min = (pitch_rate_ < 0.0 || pitch_err < 0.0) ? 0.0 : -3.0;
        double pd_h_l = bbot_jump::clamp_value(
            kp_hip_thrust * (q_hip_des - hip_pos_left_) +
                kd_hip_thrust * (nominal_qdh - hip_vel_left_),
            pd_h_min, 3.0);
        const double terminal_knee_brake_l = grounded_brake_factor * bbot_jump::clamp_value(
                                                                        -1.6 * knee_vel_left_, -12.0, 12.0);
        // 力驱动行程中名义位置轨迹是伸展的下界，不是把已经伸开的膝往回拉的理由。
        // 推地段全程保留 min(0,error)，严禁在接近限位时用双侧 P 把膝关节往回拉（破坏竖直冲量并诱发剧烈后仰）。
        // 末端减速与限位缓冲完全由速度阻尼项 (kd 及 terminal_knee_brake) 承担。
        const bool thrust_position_yield = true;
        const double pd_k_l = capture_braking ? bbot_jump::clamp_value(-3.0*knee_vel_left_,-22.0,22.0) : bbot_jump::clamp_value(
            kp_knee_thrust * bbot_jump::thrust_knee_position_error(
                                 q_knee_des, knee_pos_left_, thrust_position_yield) +
                kd_knee_thrust * (qdk - knee_vel_left_) + terminal_knee_brake_l,
            -22.0, 22.0);
        double pd_h_r = bbot_jump::clamp_value(
            kp_hip_thrust * (q_hip_des_r - hip_pos_right_) +
                kd_hip_thrust * (nominal_qdh_r - hip_vel_right_),
            pd_h_min, 3.0);
        const double terminal_knee_brake_r = grounded_brake_factor * bbot_jump::clamp_value(
                                                                        -1.6 * knee_vel_right_, -12.0, 12.0);
        const double pd_k_r = capture_braking ? bbot_jump::clamp_value(-3.0*knee_vel_right_,-22.0,22.0) : bbot_jump::clamp_value(
            kp_knee_thrust * bbot_jump::thrust_knee_position_error(
                                 q_knee_des_r, knee_pos_right_, thrust_position_yield) +
                kd_knee_thrust * (qdk_r - knee_vel_right_) + terminal_knee_brake_r,
            -22.0, 22.0);
        if (velocity_capture_reference_active_)
        {
            tau_body_per_hip=0.0;
            last_thrust_reaction_ff_=0.0;
            pd_h_l=bbot_jump::clamp_value(kd_hip_thrust*(qdh-hip_vel_left_),
                                         -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
            pd_h_r=bbot_jump::clamp_value(kd_hip_thrust*(qdh_r-hip_vel_right_),
                                         -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        }
        else if (thrust_reference_handoff_enable_)
        {
            pd_h_l+=thrust_momentum_reference_hip_feedback_l_diag_;
            pd_h_r+=thrust_momentum_reference_hip_feedback_r_diag_;
        }
        else if (thrust_momentum_reference_active_diag_)
        {
            // A private bounded velocity servo alongside the unchanged body task.
            // This command is internal motor effort, not an external H-dot command.
            const double blend=thrust_momentum_reference_blend_diag_;
            thrust_momentum_reference_hip_feedback_l_diag_=blend*bbot_jump::clamp_value(
                kd_hip_thrust*(qdh-hip_vel_left_),-4.0,4.0);
            thrust_momentum_reference_hip_feedback_r_diag_=blend*bbot_jump::clamp_value(
                kd_hip_thrust*(qdh_r-hip_vel_right_),-4.0,4.0);
            pd_h_l+=thrust_momentum_reference_hip_feedback_l_diag_;
            pd_h_r+=thrust_momentum_reference_hip_feedback_r_diag_;
        }
        else if (thrust_support_reference_diag_.active)
        {
            // Bounded internal servo so the geometric joint-rate reference has
            // an actuator path. It remains inside the existing force allocator;
            // it is not an external contact-moment command.
            const double blend=thrust_support_reference_diag_.blend;
            thrust_support_hip_servo_left_diag_=blend*bbot_jump::clamp_value(
                kd_hip_thrust*(qdh-hip_vel_left_),-4.0,4.0);
            thrust_support_hip_servo_right_diag_=blend*bbot_jump::clamp_value(
                kd_hip_thrust*(qdh_r-hip_vel_right_),-4.0,4.0);
            pd_h_l+=thrust_support_hip_servo_left_diag_;
            pd_h_r+=thrust_support_hip_servo_right_diag_;
        }
        last_thrust_position_yield_ = thrust_position_yield;
        // 推地任务解耦：膝关节负责竖直冲量，髋关节只负责机身姿态。
        // 本机构的 J^T Fz 髋力矩方向会抵消后仰纠姿力矩；即使只分配 25%，
        // 实测离地前也会把 +15 N*m 左右的纠姿命令抵消掉大半，使机器人
        // 带着持续负 pitch_rate 离地。因此推地阶段不再把竖直力映射到髋。
        constexpr double thrust_hip_force_share = 0.0;
        const double tau_attitude_reserve = 8.0;
        const auto effort_limits = current_effort_limits();
        const double hip_budget = thrust_torque_margin_ * effort_limits.hip - tau_attitude_reserve;
        const double knee_budget = thrust_torque_margin_ * effort_limits.knee;
        // 对最终合力矩求预算。膝伸展 J<0 时，正向 PD 是反向制动，
        // 不能既从推力预算扣除一次，又在最终力矩上再抵消一次。
        const double force_limit_left = std::min(
            bbot_jump::signed_force_limit(thrust_hip_force_share * jh_left, tau_body_per_hip + pd_h_l, hip_budget),
            bbot_jump::signed_force_limit(jk_left, pd_k_l, knee_budget));
        const double force_limit_right = std::min(
            bbot_jump::signed_force_limit(thrust_hip_force_share * jh_right, tau_body_per_hip + pd_h_r, hip_budget),
            bbot_jump::signed_force_limit(jk_right, pd_k_r, knee_budget));
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

        // 推力卸载时膝关节的正向急刹会把等量反作用角动量注入机身。
        // 旧控制仅按期望伸膝速度给约 0.5~1.5 Nm 前馈，实测在膝刹车
        // 达 +22 Nm/腿时，离地角速度会在 30 ms 内掉到 -1.55 rad/s。
        // 用最终膝力矩估算该瞬态，并通过髋部做有界抵消。
        const double knee_brake_reaction_ff = velocity_capture_reference_active_ ? 0.0 :
            bbot_jump::thrust_knee_brake_reaction_compensation(
                tau_ff_knee_left + pd_k_l, tau_ff_knee_right + pd_k_r);
        tau_body_per_hip = bbot_jump::clamp_value(
            tau_body_per_hip + knee_brake_reaction_ff,
            -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        // The optional gate scope replaces the release-only application and uses
        // the same bounded signed difference after the initial THRUST motion gate,
        // with a 20 ms blend. Otherwise preserve the established release-only path.
        // Both rates use this same active reference; knee reaction feedforward stays
        // ahead of the correction and the existing total body cap remains last.
        const bool correction_effort_ready =
            !velocity_capture_reference_active_ && thrust_release_fast_rate_correction_enable_ &&
            bbot_jump::effort_release_rate_correction_eligible(
                effort_mode_active_, leg_mode_switch_pending_);
        const bool correction_imu_fresh = torso_imu_.fresh(now_sec);
        const double gate_correction_blend =
            bbot_jump::thrust_gate_fast_rate_correction_blend(
                correction_effort_ready && thrust_fast_rate_correction_from_gate_,
                current_state_ == bbot_jump::STATE_THRUST,
                effort_mode_active_, leg_mode_switch_pending_,
                thrust_gate_has_opened_, correction_imu_fresh,
                thrust_motion_elapsed_, 0.020);
        thrust_fast_rate_correction_scope_diag_ =
            thrust_fast_rate_correction_from_gate_ && correction_effort_ready &&
            current_state_ == bbot_jump::STATE_THRUST && thrust_gate_has_opened_ &&
            correction_imu_fresh && std::isfinite(thrust_motion_elapsed_) &&
            thrust_motion_elapsed_ >= 0.0;
        thrust_fast_rate_correction_blend_diag_ = gate_correction_blend;
        const bool use_gate_scope = thrust_fast_rate_correction_from_gate_;
        const bool release_correction_active =
            correction_effort_ready && !use_gate_scope &&
            current_state_ == bbot_jump::STATE_THRUST &&
            thrust_release_.active() && correction_imu_fresh;
        thrust_release_fast_rate_correction_active_diag_ =
            (thrust_fast_rate_correction_scope_diag_ && gate_correction_blend > 0.0) ||
            release_correction_active;
        if (thrust_release_fast_rate_correction_active_diag_)
        {
            const double correction_blend = use_gate_scope
                ? gate_correction_blend : thrust_release_.blend(now_sec);
            const double legacy_rate_error =
                pitch_rate_ - active_jump_pitch_rate_ref_;
            const double fast_rate_error =
                torso_imu_.rate() - active_jump_pitch_rate_ref_;
            thrust_release_fast_rate_correction_requested_diag_ =
                bbot_jump::thrust_release_fast_rate_correction(
                    true, true, true, true, correction_blend,
                    legacy_rate_error, fast_rate_error,
                    K_BODY_D_THRUST_, 8.0);
            const auto corrected = bbot_jump::apply_thrust_rate_correction(
                tau_body_per_hip,
                thrust_release_fast_rate_correction_requested_diag_,
                TAU_HIP_BODY_MAX_);
            tau_body_per_hip = corrected.command;
            thrust_release_fast_rate_correction_applied_diag_ = corrected.applied;
        }
        last_thrust_reaction_ff_ += knee_brake_reaction_ff;
        last_tau_body_per_hip_ = tau_body_per_hip;

        if (terminal_brake_blend > 0.05)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 80,
                "[THRUST_TERMINAL] s=%.2f blend=%.2f vz=%.2f/%.2f "
                "knee_v=%.2f/%.2f qdk=%.2f brake_tau=%.1f/%.1f F=%.1f",
                s, terminal_brake_blend, vertical_velocity, target_takeoff_velocity_,
                knee_vel_left_, knee_vel_right_, qdk,
                terminal_knee_brake_l, terminal_knee_brake_r, F_z_thrust);
        }

        // Diagnostic only: isolate late negative hip effort while preserving
        // knee travel braking and all default controller behavior.
        const double hip_request_l = tau_ff_hip_left + tau_body_per_hip + pd_h_l;
        const double hip_request_r = tau_ff_hip_right + tau_body_per_hip + pd_h_r;
        const bool imu_fresh_for_floor = torso_imu_.fresh(now_sec);
        const double baseline_hip_send_l = bbot_jump::thrust_terminal_hip_command(
            hip_request_l, thrust_terminal_hip_floor_ && !velocity_capture_reference_active_, thrust_release_.active(),
            takeoff_geometry_aligned_, wheel_clearance_, imu_fresh_for_floor, pitch_rate_raw_);
        const double baseline_hip_send_r = bbot_jump::thrust_terminal_hip_command(
            hip_request_r, thrust_terminal_hip_floor_ && !velocity_capture_reference_active_, thrust_release_.active(),
            takeoff_geometry_aligned_, wheel_clearance_, imu_fresh_for_floor, pitch_rate_raw_);
        const int64_t pulse_now_ns = this->now().nanoseconds();
        const double joint_age_sec = now_sec - joint_sample_time_;
        const bool pulse_joint_fresh = std::isfinite(joint_sample_time_) &&
            std::isfinite(joint_age_sec) && joint_age_sec >= -0.001 && joint_age_sec <= 0.020;
        const double pulse_gate_elapsed = thrust_gate_open_stamp_ >= 0.0
            ? now_sec - thrust_gate_open_stamp_ : -1.0;
        const bbot_jump::ReferenceGroundPulseInput pulse_input{
            reference_ground_torque_pulse_enable_,
            current_state_ == bbot_jump::STATE_THRUST,
            thrust_gate_has_opened_,
            thrust_attitude_blocked_,
            effort_mode_active_,
            leg_mode_switch_pending_,
            torso_imu_.fresh(now_sec) &&
                now_sec - torso_imu_.stamp() >= -0.001 &&
                now_sec - torso_imu_.stamp() <= 0.020,
            pulse_joint_fresh,
            reference_ground_pulse_contact_valid_,
            reference_ground_pulse_contact_continuous_,
            reference_ground_pulse_wheel_mask_,
            pulse_now_ns,
            reference_ground_pulse_contact_stamp_ns_,
            pulse_gate_elapsed};
        const auto pulse = bbot_jump::reference_ground_torque_pulse(pulse_input);
        reference_ground_pulse_guard_diag_ = static_cast<int>(pulse.guard);
        reference_ground_pulse_gate_elapsed_diag_ = pulse_gate_elapsed >= 0.0
            ? pulse_gate_elapsed : 0.0;
        reference_ground_pulse_shape_diag_ = pulse.shape;
        reference_ground_pulse_requested_left_diag_ = pulse.requested_offset_nm;
        reference_ground_pulse_requested_right_diag_ = pulse.requested_offset_nm;
        const double hip_request_with_pulse_l = hip_request_l + pulse.requested_offset_nm;
        const double hip_request_with_pulse_r = hip_request_r + pulse.requested_offset_nm;
        const double hip_send_l = bbot_jump::thrust_terminal_hip_command(
            hip_request_with_pulse_l, thrust_terminal_hip_floor_ && !velocity_capture_reference_active_, thrust_release_.active(),
            takeoff_geometry_aligned_, wheel_clearance_, imu_fresh_for_floor, pitch_rate_raw_);
        const double hip_send_r = bbot_jump::thrust_terminal_hip_command(
            hip_request_with_pulse_r, thrust_terminal_hip_floor_ && !velocity_capture_reference_active_, thrust_release_.active(),
            takeoff_geometry_aligned_, wheel_clearance_, imu_fresh_for_floor, pitch_rate_raw_);
        const auto effort_caps_for_pulse = current_effort_limits();
        const double final_base_l = bbot_jump::clamp_value(
            baseline_hip_send_l, -effort_caps_for_pulse.hip, effort_caps_for_pulse.hip);
        const double final_base_r = bbot_jump::clamp_value(
            baseline_hip_send_r, -effort_caps_for_pulse.hip, effort_caps_for_pulse.hip);
        thrust_hip_requested_left_diag_ = hip_request_l;
        thrust_hip_requested_right_diag_ = hip_request_r;
        thrust_hip_floor_applied_diag_ =
            hip_send_l != hip_request_with_pulse_l ||
            hip_send_r != hip_request_with_pulse_r;

        bool complete_contact_takeoff = false;
        int64_t complete_contact_frame_stamp_ns = 0;
        int64_t complete_contact_com_stamp_ns = 0;
        if (complete_contact_takeoff_confirmation_enable_ &&
            effort_mode_active_ && !leg_mode_switch_pending_ &&
            current_state_ == bbot_jump::STATE_THRUST &&
            thrust_gate_has_opened_ && thrust_motion_elapsed_ > 0.0)
        {
            const bool com_pair_aligned = capture_world_valid_ &&
                centroidal_velocity_valid_ && centroidal_height_.valid(now_sec) &&
                centroidal_world_.valid(now_sec) &&
                std::abs(centroidal_height_.stamp() - centroidal_world_.stamp()) <= 0.001;
            const int64_t com_stamp_ns = com_pair_aligned
                ? static_cast<int64_t>(std::llround(centroidal_height_.stamp() * 1.0e9)) : 0;
            const bbot_jump::CompleteTakeoffComEvidence com_evidence{
                com_pair_aligned, com_stamp_ns, vertical_velocity};
            const int64_t now_ns = this->now().nanoseconds();
            complete_contact_takeoff = contact_takeoff_observer_.confirmed(
                now_ns, static_cast<int64_t>(std::llround(thrust_gate_open_stamp_ * 1.0e9)),
                current_state_ == bbot_jump::STATE_THRUST,
                effort_mode_active_, leg_mode_switch_pending_,
                thrust_gate_has_opened_ && thrust_motion_elapsed_ > 0.0,
                thrust_release_.active() || takeoff_rise_observed_, com_evidence);
            contact_takeoff_confirmed_diag_ = complete_contact_takeoff;
            if (complete_contact_takeoff)
            {
                complete_contact_frame_stamp_ns = contact_takeoff_observer_.latest_stamp_ns();
                complete_contact_com_stamp_ns = com_stamp_ns;
            }
        }

        // Experimental bounded support allocation. It receives the already
        // travel/unload-protected force and only owns output after every
        // synchronized-state/contact guard and the constrained solve pass.
        thrust_support_allocation_diag_ = {};
        thrust_support_tasks_diag_ = {};
        thrust_support_allocator_guard_diag_ = thrust_support_allocator_enable_ ? 1 : 0;
        thrust_support_allocator_active_diag_ = false;
        thrust_support_allocator_legacy_active_diag_ = false;
        thrust_support_allocator_contact_hold_diag_ = false;
        thrust_support_contact_diag_={};
        thrust_support_actual_leg_command_diag_.fill(0.0);
        thrust_support_actual_wheel_command_diag_ = 0.0;
        thrust_support_com_rel_forward_diag_ = 0.0;
        thrust_support_com_rel_velocity_diag_ = 0.0;
        bool thrust_support_allocator_active = false;
        bbot_jump::LandingJointSample allocator_sample;
        const double allocator_imu_stamp = torso_imu_.stamp();
        const bool allocator_joint_pair = thrust_support_allocator_enable_ &&
            landing_joint_history_.exact(allocator_imu_stamp, now_sec, allocator_sample);
        bbot_jump::AllocatorContactSnapshot allocator_contact_snapshot;
        const bool allocator_contact_snapshot_valid = thrust_support_allocator_enable_ &&
            allocator_contact_history_.snapshot(static_cast<int64_t>(std::llround(now_sec*1e9)),
                                                allocator_contact_snapshot);
        thrust_support_contact_diag_=allocator_contact_snapshot;
        const bool allocator_contact = allocator_contact_snapshot_valid &&
            allocator_contact_snapshot.continuous &&
            allocator_contact_snapshot.wheel_mask == 0x3;
        const bool allocator_com_pair = allocator_joint_pair && capture_world_valid_ &&
            centroidal_velocity_valid_ && centroidal_height_.valid(now_sec) &&
            centroidal_world_.valid(now_sec) &&
            std::abs(centroidal_height_.stamp() - centroidal_world_.stamp()) <= 0.001 &&
            now_sec - centroidal_height_.stamp() <= 0.020 &&
            now_sec - centroidal_world_.stamp() <= 0.020 &&
            std::abs(centroidal_height_.stamp() - allocator_imu_stamp) <= 0.010 &&
            std::abs(centroidal_height_.stamp() - allocator_sample.stamp) <= 0.010;
        const bool allocator_h_pair = landing_momentum_diag_.valid &&
            std::abs(landing_momentum_stamp_diag_ - allocator_imu_stamp) <= 1e-9;
        const bool allocator_odom_fresh = odom_received_ && takeoff_odom_pose_valid_ &&
            world_pose_velocity_.valid() && jump_forward_axis_world_.head<2>().norm() > 0.5 &&
            now_sec >= takeoff_odom_stamp_ && now_sec - takeoff_odom_stamp_ <= 0.020 &&
            std::abs(takeoff_odom_stamp_ - allocator_imu_stamp) <= 0.010 &&
            std::abs(takeoff_odom_stamp_ - allocator_sample.stamp) <= 0.010;
        const bool allocator_phase = current_state_ == bbot_jump::STATE_THRUST &&
            thrust_gate_has_opened_ && thrust_motion_elapsed_ > 0.0 && !complete_contact_takeoff;
        if (thrust_support_allocator_enable_) {
            // Guard codes: 1 phase, 2 effort/switch, 3 bilateral contact,
            // 4 synchronized joint/COM/H/odom, 5 invalid model/task, 6 solve.
            if (!allocator_phase) thrust_support_allocator_guard_diag_ = 1;
            else if (!effort_mode_active_ || leg_mode_switch_pending_) thrust_support_allocator_guard_diag_ = 2;
            else if (!allocator_contact) {
                thrust_support_allocator_guard_diag_ = 3;
                thrust_support_allocator_contact_hold_diag_ = true;
            }
            else if (!allocator_joint_pair || !allocator_com_pair || !allocator_h_pair ||
                     !allocator_odom_fresh || thrust_attitude_blocked_) thrust_support_allocator_guard_diag_ = 4;
            else {
                bbot_jump::ThrustSupportAllocationInput allocator_input;
                auto &d = allocator_input.dynamics;
                d.body_mass = body_mass_;
                d.prediction_dt = 0.010;
                d.actuator_limits = {150.0, 150.0, 150.0, 150.0, 10.0, 10.0};
                d.actuator_speed_limits = {11.0, 15.0, 11.0, 15.0, 30.0, 30.0};
                d.joint_position_min = {-1.52, -1.56, -1.52, -1.56};
                d.joint_position_max = { 1.52,  1.56,  1.52,  1.56};
                d.q[2] = -pitch_;
                d.v[0] = jump_forward_axis_world_.head<2>().dot(
                    Eigen::Vector2d(gazebo_world_x_dot_, gazebo_world_y_dot_));
                d.v[1] = gazebo_world_z_dot_;
                d.v[2] = -pitch_rate_raw_;
                for (int i = 0; i < 4; ++i) {
                    d.q[3+i] = allocator_sample.q[i];
                    d.v[3+i] = allocator_sample.v[i];
                }
                d.v[7] = allocator_sample.wheel_rate[0];
                d.v[8] = allocator_sample.wheel_rate[1];
                d.q[7] = d.q[8] = 0.0;
                bbot_jump::ThrustSupportDynamicsModel support_model;
                const bool model_valid = bbot_jump::thrust_support_dynamics_model(d, support_model);
                const double relative_velocity = model_valid
                    ? support_model.com_axle_relative_forward_jacobian.dot(d.v) : 0.0;
                thrust_support_com_rel_forward_diag_ = model_valid
                    ? support_model.com_axle_relative_forward : 0.0;
                thrust_support_com_rel_velocity_diag_ = relative_velocity;
                const double remaining_window = std::clamp(thrust_duration_ - thrust_motion_elapsed_, 0.020, 0.120);
                const bool tasks_valid = model_valid && bbot_jump::make_thrust_support_task_reference(
                    F_z_thrust, TOTAL_MASS_, landing_momentum_diag_.total, thrust_support_target_h_,
                    remaining_window, support_model.com_axle_relative_forward,
                    relative_velocity, thrust_support_tasks_diag_) &&
                    bbot_jump::make_thrust_body_pitch_reference(pitch_,pitch_rate_raw_,
                        jump_pitch_ref_,d.prediction_dt,thrust_support_tasks_diag_);
                allocator_input.tasks = thrust_support_tasks_diag_;
                allocator_input.wheel_mode = bbot_jump::SupportWheelMode::BoundedWheelServo;
                allocator_input.wheel_servo_gain = wheel_servo_gain_;
                if (!tasks_valid) thrust_support_allocator_guard_diag_ = 5;
                else {
                    thrust_support_allocation_diag_ = bbot_jump::allocate_thrust_support(allocator_input);
                    if (!thrust_support_allocation_diag_.valid ||
                        !thrust_support_allocation_diag_.decision.allFinite() ||
                        !std::isfinite(thrust_support_allocation_diag_.allocated_linear_command))
                        thrust_support_allocator_guard_diag_ = 6;
                    else {
                        thrust_support_allocator_active = true;
                        thrust_support_allocator_guard_diag_ = 7;
                    }
                }
            }
        }

        // 2. 髋关节姿态稳定前馈补偿；PD 只做低增益构型保持。
        if (!complete_contact_takeoff)
        {
            if (thrust_support_allocator_active) {
                const auto &x = thrust_support_allocation_diag_.decision;
                const std::array<double,4> raw{x[9],x[10],x[11],x[12]};
                for (int i=0;i<4;++i) {
                    if (!std::isfinite(raw[i])) {
                        thrust_support_allocator_active=false;
                        thrust_support_allocator_guard_diag_=8;
                        break;
                    }
                    thrust_support_actual_leg_command_diag_[i] = std::clamp(raw[i],-150.0,150.0);
                }
                if (thrust_support_allocator_active &&
                    !publish_allocated_effort(thrust_support_actual_leg_command_diag_, now_sec)) {
                    thrust_support_allocator_active=false;
                    thrust_support_allocator_guard_diag_=2;
                }
                thrust_support_allocator_active_diag_ = thrust_support_allocator_active;
            }
            if (!thrust_support_allocator_active) {
                if (thrust_support_allocator_contact_hold_diag_) {
                    // A unilateral/stale contact is not a feasible bilateral
                    // support allocation. Hand off only the pre-existing
                    // terminal knee brake; suppress hip/force PD and wheel drive.
                    publish_effort_leg_control_lr(
                        hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, terminal_knee_brake_l, 0.0, terminal_knee_brake_r,
                        0.0, 0.0, 0.0, 0.0);
                } else {
                    thrust_support_allocator_legacy_active_diag_ = thrust_support_allocator_enable_;
                    publish_effort_leg_control_lr(
                        q_hip_des, q_knee_des, q_hip_des_r, q_knee_des_r,
                        qdh, qdk, qdh_r, qdk_r,
                        hip_send_l, tau_ff_knee_left + pd_k_l,
                        hip_send_r, tau_ff_knee_right + pd_k_r,
                        // 使用上方与力矩预算一致的限幅 PD，通用下发层不得重复叠加。
                        0.0, 0.0, 0.0, 0.0,
                        tau_body_per_hip, tau_body_per_hip);
                }
            }
            // These fields describe the published command difference after all
            // existing allocation and clamps; they are not measured motor torque.
            reference_ground_pulse_applied_left_diag_ =
                bbot_jump::reference_ground_pulse_applied_delta(final_base_l, actual_tau_hip_left_);
            reference_ground_pulse_applied_right_diag_ =
                bbot_jump::reference_ground_pulse_applied_delta(final_base_r, actual_tau_hip_right_);
        }

        if (velocity_capture_enable_) {
            velocity_capture_log_<<std::setprecision(17)<<now_sec<<','<<jump_id_<<','<<static_cast<int>(capture_session.phase)
                <<','<<static_cast<int>(capture_session.reason)<<','<<capture_session.owns_command<<','<<!capture_session.owns_command
                <<','<<(capture_pair.valid?capture_pair.joints.stamp:-1.0)<<','<<capture_inverse.valid<<','<<capture_fresh
                <<','<<capture_support<<','<<capture_stop.valid<<','<<capture_stop.brake<<','<<capture_latency
                <<','<<velocity_capture_hip_decel_<<','<<velocity_capture_knee_decel_;
            const std::array<double,4> capture_tau{actual_tau_hip_left_,actual_tau_knee_left_,actual_tau_hip_right_,actual_tau_knee_right_};
            for (const auto& part:{capture_q,capture_v,capture_requested,capture_session.reference,
                                 capture_stop.remaining,capture_stop.required,capture_tau})
                for(double x:part) velocity_capture_log_<<','<<x;
            velocity_capture_log_<<','<<F_z_thrust<<'\n';velocity_capture_log_.flush();
        }
        // rolling-THRUST：从接近速度继续加速到更高的离地前向速度。
        // 当前符号链中负 cmd 对应正 x_dot，因此速度不足时让命令进一步变负。
        // 当世界系质心平移速度有效时优先闭环质心前向速度，消除伸腿几何前倾造成的超调。
        const double raw_forward_speed = capture_world_valid_ ? capture_com_velocity_ : x_dot_;
        const auto forward_speed_prediction = thrust_forward_speed_predictor_.predict(
            now_sec, raw_forward_speed, capture_world_valid_,
            thrust_forward_speed_prediction_enable_);
        const double current_forward_speed = forward_speed_prediction.predicted_speed;
        const double forward_speed_error = jump_takeoff_forward_speed_ - current_forward_speed;
        const double thrust_forward_cmd_mag = bbot_jump::clamp_value(
            jump_takeoff_forward_speed_ +
                thrust_forward_velocity_kp_ * forward_speed_error,
            0.0, 0.85);
        // v6.4：名义伸腿时间结束不代表可以反向制动车轮。带前倾时
        // 撤回支撑点会加剧前倒；直到离地都保留与 PRE_JUMP 同符号的姿态反馈。
        interpolate_lqr_gain();
        const double thrust_fwd_term = -thrust_forward_cmd_mag;
        const double thrust_att_term_raw = 0.037 * (current_gain_.k_theta * pitch_err +
                                                    current_gain_.k_theta_dot * pitch_rate_err);
        const double thrust_attitude_scale = thrust_forward_attitude_taper_enable_
            ? bbot_jump::thrust_forward_attitude_scale(
                  current_forward_speed, jump_takeoff_forward_speed_, capture_world_valid_)
            : 1.0;
        const double thrust_att_term = bbot_jump::apply_thrust_forward_attitude_taper(
            thrust_att_term_raw, thrust_attitude_scale, thrust_forward_attitude_taper_enable_);
        double kinematic_correction = 0.0;
        if (thrust_wheel_kinematics_compensation_enable_ && effort_mode_active_ &&
            !leg_mode_switch_pending_ && thrust_kinematics_valid_diag_)
            kinematic_correction = thrust_kinematics_applied_correction_diag_;
        const double thrust_wheel_target = bbot_jump::thrust_ground_wheel_target_with_kinematics(
            thrust_forward_cmd_mag, thrust_att_term, kinematic_correction);
        const double cmd_x = bbot_jump::thrust_wheel_rate_limit(
            thrust_wheel_target, last_wheel_cmd_x_, dt, 8.0, thrust_wheel_max_decel_);
        thrust_fwd_term_diag_ = thrust_fwd_term;
        thrust_att_term_diag_ = thrust_att_term;
        thrust_att_term_raw_diag_ = thrust_att_term_raw;
        thrust_attitude_scale_diag_ = thrust_attitude_scale;
        thrust_wheel_target_diag_ = thrust_wheel_target;
        thrust_cmd_x_diag_ = cmd_x;
        thrust_forward_speed_raw_diag_ = forward_speed_prediction.raw_speed;
        thrust_forward_speed_base_diag_ = forward_speed_prediction.base_speed;
        thrust_forward_speed_predicted_diag_ = forward_speed_prediction.predicted_speed;
        thrust_forward_speed_accel_diag_ = forward_speed_prediction.acceleration;
        thrust_forward_speed_horizon_diag_ = forward_speed_prediction.horizon;
        thrust_forward_speed_delta_diag_ = forward_speed_prediction.delta_speed;
        thrust_forward_speed_prediction_active_diag_ = forward_speed_prediction.active;
        thrust_com_sample_stamp_diag_ = centroidal_world_.stamp();
        thrust_com_valid_diag_ = capture_world_valid_ ? 1 : 0;
        if (!complete_contact_takeoff) {
            if (thrust_support_allocator_active) {
                thrust_support_actual_wheel_command_diag_ =
                    thrust_support_allocation_diag_.allocated_linear_command;
                publish_wheel_cmd(thrust_support_actual_wheel_command_diag_, 0.0);
            } else if (thrust_support_allocator_contact_hold_diag_) {
                thrust_support_actual_wheel_command_diag_ = 0.0;
                // Zero target rate would brake a spinning wheel. Ask the bounded
                // servo explicitly for zero effort on the existing command path.
                publish_wheel_cmd(0.0, 0.0, true);
            } else publish_wheel_cmd(cmd_x, 0.0);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 80,
                             "[THRUST_COM_VREF v6.13] active=%d yield=%d vz=%.3f/%.3f Jk=%.4f knee_v=%.2f/%.2f "
                             "pd=%.2f F=%.1f ff=%.2f hip_tau=%.2f travel=%.2f brake=%.2f limited=%d",
                             thrust_velocity_reference_.valid ? 1 : 0, last_thrust_position_yield_ ? 1 : 0,
                             vertical_velocity,
                             thrust_velocity_reference_.com_velocity, thrust_velocity_reference_.knee_jacobian,
                             knee_vel_left_, qdk,
                             pd_k_l, F_z_thrust, last_thrust_reaction_ff_, tau_body_per_hip,
                             thrust_extension_scale_, terminal_brake_blend,
                             thrust_velocity_reference_.speed_limited ? 1 : 0);

        if (!complete_contact_takeoff)
        {
            last_thrust_force_ = F_z_thrust;
            thrust_mechanical_work_ += std::abs(F_z_thrust * vertical_velocity) * dt * 2.0;
        }

        // v6.3：同一odom时间戳的世界高度、机身姿态、插值关节构型。
        // 接地快速伸腿时，旧高度减新FK会制造厘米级负间隙，掩盖真实离地。
        double geom_left = 0.0, geom_right = 0.0;
        takeoff_geometry_aligned_ = aligned_takeoff_geometry(now_sec, geom_left, geom_right);
        takeoff_clearance_left_ = takeoff_geometry_aligned_ ? gazebo_world_z_ - ground_height_offset_ - geom_left : -1.0;
        takeoff_clearance_right_ = takeoff_geometry_aligned_ ? gazebo_world_z_ - ground_height_offset_ - geom_right : -1.0;
        wheel_clearance_ = std::min(takeoff_clearance_left_, takeoff_clearance_right_);
        if (vertical_velocity > 0.35 && gazebo_world_z_ - thrust_start_world_z_ > 0.012)
            takeoff_rise_observed_ = true;
        const bool accel_unloaded = acc_z_filt_ < 8.5 && acc_z_raw_ < 7.5;
        // 已观察到起跳上升后，不因临近顶点vz降低而永久禁止确认离地。
        // 普通净空仍需失重证据；两轮均超过20mm时可用连续几何确认，排除单次IMU冲击。
        const bool wheels_airborne = takeoff_confirmation_.update(
            takeoff_odom_stamp_, now_sec,
            takeoff_geometry_aligned_ && thrust_gate_has_opened_ && thrust_motion_elapsed_ > 0.0,
            takeoff_clearance_left_, takeoff_clearance_right_, accel_unloaded, takeoff_rise_observed_);
        airborne_confidence_count_ = takeoff_confirmation_.count();
        last_wheels_airborne_ = wheels_airborne || complete_contact_takeoff;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 70,
                             "[TAKEOFF_SYNC] t=%.3f aligned=%d stamp=%.3f age=%.3f clrL=%.4f clrR=%.4f "
                             "vz=%.2f az=%.1f/%.1f rise=%d conf=%d airborne=%d",
                             elapsed, takeoff_geometry_aligned_ ? 1 : 0, takeoff_odom_stamp_, now_sec - takeoff_odom_stamp_,
                             takeoff_clearance_left_, takeoff_clearance_right_, vertical_velocity, acc_z_filt_, acc_z_raw_,
                             takeoff_rise_observed_ ? 1 : 0, airborne_confidence_count_, wheels_airborne ? 1 : 0);
        if (!complete_contact_takeoff)
            log_data(cmd_x, 0.5 * (tau_ff_hip_left + tau_ff_hip_right),
                     0.5 * (tau_ff_knee_left + tau_ff_knee_right), F_z_thrust * 2.0);

        const bool velocity_takeoff = takeoff_speed_latch_.latched();
        // 速度或腿长变化均不能替代失重确认。只有轮子连续确认失重后，
        // 才允许进入 FLIGHT 并执行收腿/落地时序。
        const bool airborne_takeoff = wheels_airborne || complete_contact_takeoff;
        bool leg_collapsing = (elapsed > 0.04 && current_z_ < state_start_z_ - 0.025);
        bool thrust_timeout = (elapsed >= thrust_timeout_);
        bool attitude_ready = (std::abs(pitch_err) < 0.35 && std::abs(pitch_rate_) < 2.5);

        if (velocity_capture_enable_ && capture_braking && capture_fresh && capture_support &&
            std::all_of(capture_v.begin(),capture_v.end(),[](double v){return std::abs(v)<=0.50;}) &&
            !airborne_takeoff)
        {
            abort_jump_to_recovery(now_sec,"velocity capture 单次接管已卸力制动，进入恢复");
            return;
        }
        if (leg_collapsing)
        {
            abort_jump_to_recovery(now_sec, "推地期间腿长反向缩短");
            return;
        }

        if (thrust_timeout && !airborne_takeoff)
        {
            abort_jump_to_recovery(now_sec, "推地超时且轮子未确认离地");
            return;
        }

        // 姿态失稳保护。确认已经离地后立即停止推地；如果姿态不满足正常
        // 腾空门槛，则以 protective landing 进入 FLIGHT，优先展腿保命，
        // 不能继续在接地推力下等待姿态“变好”。
        if (thrust_timeout && !attitude_ready)
        {
            transition_to_protective_landing(now_sec, "离地前姿态或角速度未稳定");
            return;
        }

        if (airborne_takeoff)
        {
            takeoff_com_z_ = takeoff_speed_latch_.latched() ? takeoff_speed_latch_.height() : centroidal_height_.height();
            takeoff_com_vz_ = takeoff_speed_latch_.latched() ? takeoff_speed_latch_.velocity() : vertical_velocity;
            // Formal takeoff metrics use world-frame COM translation only.
            // A missing sample is recorded as invalid rather than silently
            // substituting wheel-derived chassis speed.
            takeoff_com_vx_ = takeoff_speed_latch_.latched() ? takeoff_speed_latch_.vx() : (capture_world_valid_ ? capture_com_velocity_ : -1.0);
            com_takeoff_confirmed_vz_ = vertical_velocity;
            actual_takeoff_velocity_ = takeoff_com_vz_;
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
                std::abs(takeoff_forward_speed_) > 0.05)
            {
                landing_forward_axis_x_ = gazebo_world_x_dot_ / world_horizontal_speed;
                landing_forward_axis_y_ = gazebo_world_y_dot_ / world_horizontal_speed;
                landing_forward_direction_sign_ =
                    (takeoff_forward_speed_ >= 0.0) ? 1.0 : -1.0;
                landing_forward_axis_valid_ = true;
            }
            // 滚动离地时轮子已经旋转。空中轮控必须围绕此基准做增量，
            // 不能把“目标0”解释成离地后立即刹轮，否则会额外注入俯仰冲量。
            air_wheel_baseline_ = last_wheel_cmd_x_;
            landing_support_start_time_ = -1.0;
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
            const auto end_v_hip = [](double v)
            {
                return bbot_jump::clamp_value(v, -2.0, 2.0);
            };
            const auto end_v_knee = [](double v)
            {
                return bbot_jump::clamp_value(v, -4.0, 4.0);
            };
            const auto end_q = [&](double q, double v0, double vf)
            {
                return bbot_jump::clamp_value(
                    q + 0.5 * (v0 + vf) * arrest_duration, -1.56, 1.56);
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

            const char *takeoff_reason = velocity_takeoff ? "达到目标离地速度并确认失重" : "已确认失重（速度不足）";
            if (!velocity_takeoff)
            {
                jump_failure_reason_ = "已离地但未达到目标速度";
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
            // 离地瞬时必然保留推地残余角速度，由接下来的 ATTITUDE_ARREST 阶段消除。
            // 只有当关节已严重超出物理行程范围时，才在离地时刻判定为保护起跳。
            const bool severe_leg_overtravel =
                std::abs(hip_pos_left_) > 1.52 || std::abs(hip_pos_right_) > 1.52 ||
                std::abs(knee_pos_left_) > 1.5708 || std::abs(knee_pos_right_) > 1.5708;
            const bool protective_takeoff = severe_attitude || severe_leg_overtravel;
            if (protective_takeoff)
            {
                jump_failure_reason_ = severe_attitude ? "离地姿态超标" : "离地关节行程超标";
                RCLCPP_WARN(this->get_logger(),
                            "[离地保护] 已确认离地，姿态或行程超标 "
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
            flight_tuck_plan_check_ = -1;
            flight_extend_plan_check_ = -1;
            flight_round_trip_plan_ = {};
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
            if (protective_takeoff)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[FLIGHT接管] protective_takeoff=1，但先执行 ATTITUDE_ARREST；"
                            "当前 hip_v=%.2f/%.2f knee_v=%.2f/%.2f，禁止高速腿直接展到落地构型",
                            hip_vel_left_, hip_vel_right_, knee_vel_left_, knee_vel_right_);
            }

            // 保持力矩控制模式
            request_effort_controller();
        }
        if (complete_contact_takeoff && current_state_ == bbot_jump::STATE_FLIGHT)
        {
            contact_takeoff_observer_.mark_taken(
                complete_contact_frame_stamp_ns, complete_contact_com_stamp_ns);
            run_state_flight(now_sec);
            contact_takeoff_confirmed_diag_ = false;
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
        const auto &p = kinematics_.get_params();
        constexpr double kHipBodyVerticalOffset = 0.07;
        constexpr double kCadWheelToHipLongitudinal = -0.01137221;
        const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
        const double phi2_0 = std::atan2(0.28210870, 0.19553796);

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
        const auto &p = kinematics_.get_params();
        double limit = landing_shank_abs_max_;
        if (p.l1 > 1e-6)
        {
            const double ratio = bbot_jump::clamp_value(
                landing_knee_axis_clearance_min_ / p.l1, 0.0, 0.999);
            const double clearance_limit = std::acos(ratio);
            limit = std::min(limit, clearance_limit);
        }
        return bbot_jump::clamp_value(limit, 0.35, 1.20);
    }

    // 落地接近阶段的最后一道几何保护：
    // theta_shank = phi2_0 + q_hip + q_knee - body_pitch。
    // 若期望小腿已经接近水平，只做有界的逐帧安全投影。真实关节已经
    // 在锥外时，一帧投影到边界不会让物理构型瞬移，只会制造膝目标和
    // 速度目标阶跃；完整轨迹的安全终点负责在触地前进入安全锥。
    bool enforce_wheel_first_target(
        double &q_hip, double &q_knee,
        double &qd_hip, double &qd_knee,
        double reference_pitch, double reference_pitch_rate) const
    {
        const double phi2_0 = std::atan2(0.28210870, 0.19553796);
        const double limit = landing_shank_limit();
        const auto projected = bbot_jump::bounded_wheel_first_target(
            q_hip, q_knee, qd_hip, qd_knee,
            reference_pitch, reference_pitch_rate, phi2_0, limit);
        q_knee = bbot_jump::clamp_value(projected.knee_position, -1.56, 1.56);
        qd_knee = projected.knee_velocity;
        return projected.active;
    }

    double body_height_above_wheel_ground(double hip, double knee) const
    {
        // URDF: base_link -> hip = (x, 0.125, -0.07), pitch = -roll。
        // FK 把机身偏移简化为固定 0.07；计算触地间隙时必须旋转此偏移。
        return kinematics_.calculate_com_height(pitch_, hip, knee) +
               0.125 * std::sin(pitch_) + 0.07 * (std::cos(pitch_) - 1.0);
    }

    bool aligned_takeoff_geometry(double now_sec, double &left, double &right) const
    {
        std::array<double, 4> q{};
        if (!takeoff_odom_pose_valid_ || takeoff_odom_stamp_ <= 0.0 ||
            now_sec - takeoff_odom_stamp_ > 0.080 || takeoff_odom_stamp_ > now_sec + 0.001 ||
            !takeoff_joint_history_.interpolate(takeoff_odom_stamp_, q))
            return false;
        const auto height = [&](double hip, double knee)
        {
            return kinematics_.calculate_com_height(takeoff_odom_pitch_, hip, knee) +
                   0.125 * std::sin(takeoff_odom_pitch_) + 0.07 * (std::cos(takeoff_odom_pitch_) - 1.0);
        };
        left = height(q[0], q[1]);
        right = height(q[2], q[3]);
        return true;
    }

    bool flight_leg_motion_ready() const
    {
        // 仅用于“是否可开始收腿”的安全准入，不应把已经充分伸开的
        // 起跳构型误判为只能展腿保护。上一轮真实离地时 knee≈-1.31 rad，
        // 旧的 1.10 rad 阈值使它永远无法进入 TUCK。这里保留距轨迹硬限
        // 位约 0.02 rad 的裕量。入口速度只排除明显失控；真正的安全性由
        // plan_flight_round_trip 对整段位置/速度/加速度逐点校验。实测离地
        // 髋速 4.38 rad/s 时完整轨迹仍可行，旧 4.0 门限却多耗掉约 25 ms。
        return std::abs(hip_vel_left_) <= 5.5 && std::abs(hip_vel_right_) <= 5.5 &&
               std::abs(knee_vel_left_) <= 8.0 && std::abs(knee_vel_right_) <= 8.0 &&
               std::abs(hip_pos_left_) <= 1.52 && std::abs(hip_pos_right_) <= 1.52 &&
               std::abs(knee_pos_left_) <= 1.5708 && std::abs(knee_pos_right_) <= 1.5708;
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
        if (fresh_world_v)
        {
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
            -0.12, landing_target_x_max_);
    }

    void latch_landing_capture_plan(double now_sec)
    {
        landing_capture_vx_ = predicted_landing_forward_velocity(now_sec);
        const double vx = (std::abs(landing_capture_vx_) >= landing_capture_speed_deadband_) ? landing_capture_vx_ : 0.0;
        const double h = bbot_jump::clamp_value(landing_capture_height_, 0.30, 0.50);
        landing_capture_omega_ = std::sqrt(9.81 / h);
        landing_capture_raw_offset_ =
            (landing_capture_omega_ > 1e-6) ? (vx / landing_capture_omega_) : 0.0;
        landing_capture_offset_ = landing_capture_gain_ * landing_capture_raw_offset_;
        landing_target_x_ = bbot_jump::clamp_value(
            landing_wheel_back_bias_ - landing_capture_offset_,
            -0.12, landing_target_x_max_);
        // 仅用于日志直观展示虚拟腿偏角，绝不再作为 body_pitch 传给 IK。
        landing_capture_comp_ = std::atan2(
            landing_target_x_, std::max(0.20, L_TOUCH_));
        landing_capture_planned_ = true;

        const auto landing_ik = inverse_kinematics_with_target_x(
            L_TOUCH_, balance_offset_ + flight_landing_pitch_bias_, landing_target_x_);
        const auto &p = kinematics_.get_params();
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

    // v6.14：把一次收展腿往返当作整体预算来规划。旧的固定 T_FLIGHT_TUCK_/
    // T_FLIGHT_EXTEND_ 只保证"名义时长"，在真实离地膝速很高时会在 0.10s 内
    // 要求 12 rad/s 级别的反向关节速度，落地前把机身再次甩翻。
    std::array<double,4> capture_tuck_target(const std::array<double,4>& q,
        const std::array<double,4>& v, double hip, double knee) const
    {
        if (velocity_capture_enable_) return {hip,knee,hip,knee};
        return {bbot_jump::reaction_safe_configuration_step(q[0],hip,0.14,v[0]),
                bbot_jump::reaction_safe_configuration_step(q[1],knee,0.04,v[1]),
                bbot_jump::reaction_safe_configuration_step(q[2],hip,0.14,v[2]),
                bbot_jump::reaction_safe_configuration_step(q[3],knee,0.04,v[3])};
    }

    bbot_jump::FlightRoundTripPlan plan_tuck_round_trip(double now_sec, double time_available) const
    {
        if (!flight_leg_motion_ready())
            return {};
        // 校验 begin_tuck 真正使用的实测边界，而不是 arrest 的预测参考。
        // 仅看 incoming 速度并不能说明一条接近伸直的腿能否在 0.10s 内收回去。
        const std::array<double, 4> q{
            hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_};
        const std::array<double, 4> v{
            hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_};
        const std::array<double, 4> a{};
        const double tuck_comp = bbot_jump::clamp_value(pitch_ - balance_offset_, -0.24, 0.24);
        const auto tuck = inverse_kinematics_with_target_x(L_RETRACT_, tuck_comp, 0.0);
        const auto land = inverse_kinematics_with_target_x(
            L_TOUCH_, balance_offset_ + flight_landing_pitch_bias_,
            preview_landing_target_x(now_sec));
        const std::array<double,4> mid=capture_tuck_target(q,v,tuck.theta_hip,tuck.theta_knee);
        const std::array<double, 4> end{
            land.theta_hip, land.theta_knee, land.theta_hip, land.theta_knee};
        // 预留与 TUCK 着陆死线相同的裕量。
        return bbot_jump::plan_flight_round_trip(q, v, a, mid, end,
                                                 T_FLIGHT_TUCK_, T_FLIGHT_EXTEND_,
                                                 time_available - landing_deploy_ready_margin_ - 0.020,
                                                 flight_hip_speed_limit_, flight_knee_speed_limit_,
                                                 flight_hip_acc_limit_, flight_knee_acc_limit_,
                                                 flight_hip_pos_limit_, flight_knee_pos_limit_);
    }

    bool normal_landing_plan_feasible(double duration) const
    {
        const std::array<double, 4> q{
            hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_};
        const std::array<double, 4> v{
            hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_};
        const std::array<double, 4> a{};
        const auto land = inverse_kinematics_with_target_x(
            L_TOUCH_, balance_offset_ + flight_landing_pitch_bias_, landing_target_x_);
        return bbot_jump::flight_segment_admissible(
            q, v, a,
            {land.theta_hip, land.theta_knee, land.theta_hip, land.theta_knee},
            duration,
            flight_hip_speed_limit_, flight_knee_speed_limit_,
            flight_hip_acc_limit_, flight_knee_acc_limit_,
            flight_hip_pos_limit_, flight_knee_pos_limit_);
    }

    // 以同一时刻的旧轨迹作为新轨迹边界，避免子阶段切换时重置目标。
    void sample_flight_joints(double now_sec, std::array<double, 4> &q,
                              std::array<double, 4> &v, std::array<double, 4> &a) const
    {
        if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST)
        {
            arrest_hip_left_traj_.evaluate(now_sec, q[0], v[0], a[0]);
            arrest_knee_left_traj_.evaluate(now_sec, q[1], v[1], a[1]);
            arrest_hip_right_traj_.evaluate(now_sec, q[2], v[2], a[2]);
            arrest_knee_right_traj_.evaluate(now_sec, q[3], v[3], a[3]);
        }
        else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY)
        {
            protective_hip_left_traj_.evaluate(now_sec, q[0], v[0], a[0]);
            protective_knee_left_traj_.evaluate(now_sec, q[1], v[1], a[1]);
            protective_hip_right_traj_.evaluate(now_sec, q[2], v[2], a[2]);
            protective_knee_right_traj_.evaluate(now_sec, q[3], v[3], a[3]);
        }
        else
        {
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
        if (flight_trajectory_initialized_)
            sample_flight_joints(now_sec, q, v, a);
        const auto ik = inverse_kinematics_with_target_x(
            height, bbot_jump::clamp_value(body_pitch_ref, -0.30, 0.30),
            target_x);
        const std::array<double, 4> end{ik.theta_hip, ik.theta_knee, ik.theta_hip, ik.theta_knee};
        std::array<bbot_jump::QuinticTrajectory *, 4> dest{
            &protective_hip_left_traj_, &protective_knee_left_traj_,
            &protective_hip_right_traj_, &protective_knee_right_traj_};
        if (protective)
            protective_deploy_rate_limited_ = false;
        for (size_t i = 0; i < q.size(); ++i)
        {
            auto &traj = protective ? *dest[i] : normal_flight_joint_traj_[i];
            double q_end = end[i];
            if (protective)
            {
                // 短腾空保护不能为追到理想着陆 IK 而在一两个采样周期内
                // 反向甩髋。零端速五次轨迹的峰值速度约为 1.875*Δq/T，
                // v6.1 提高到约50%的速度预算，并允许段末继续replan；
                // 这样既避免单帧反向甩腿，也不会永久停在中间长腿构型。
                const bool hip_joint = (i % 2) == 0;
                // 试验/调参值（对齐总结文档 v6.21）：5.0/7.0 在 duration=0.24s 时
                // 只允许髋走 0.5*5*0.24=0.60 rad，而实测离地髋角 1.19、着陆构型
                // 需要约 0.40，预算把大腿截断在中间位（PROTECTIVE_DEPLOY_PLAN
                // rate_limited=1），收腿一直拖到触地前才做完，反作用把机身推成
                // +0.52rad 前倾。8.0 给出 0.96 rad，够一程收完并把余量留给姿态环。
                const double speed_limit = 8.0;
                const double max_delta = 0.50 * speed_limit * std::max(0.05, duration);
                const double requested_delta = end[i] - q[i];
                double bounded_delta = bbot_jump::clamp_value(
                    requested_delta, -max_delta, max_delta);

                // 当髋仍沿反方向高速摆动时，短时间内强制反向会把等量
                // 角动量传给机身。保护段只把该速度刹到零、端点保持在
                // 当前角度；旧逻辑把端点继续沿错误方向外推，使下一次
                // 重规划需要更大的反向冲量。
                if (hip_joint && std::abs(v[i]) > 0.50 &&
                    bounded_delta * v[i] < 0.0)
                {
                    bounded_delta = 0.0;
                }
                q_end = q[i] + bounded_delta;
                protective_deploy_rate_limited_ =
                    protective_deploy_rate_limited_ ||
                    std::abs(q_end - end[i]) > 1e-6;
            }
            traj.init(now_sec, duration, q[i], v[i], a[i], q_end, 0.0, 0.0);
        }
    }

    // Private allocator experiment: fresh airborne boundary, direct computed
    // efforts, and contact-source handoff. The complete default path stays below.
    bool run_allocator_flight(double now_sec)
    {
        const int64_t now_ns=static_cast<int64_t>(std::llround(now_sec*1e9));
        const bool contact_fresh=reference_ground_pulse_contact_valid_ &&
            now_ns>=reference_ground_pulse_contact_stamp_ns_ &&
            now_ns-reference_ground_pulse_contact_stamp_ns_<=20000000;
        bbot_jump::LandingJointSample sample;
        const bool paired=landing_joint_history_.exact(torso_imu_.stamp(),now_sec,sample) &&
            landing_momentum_diag_.valid &&
            std::abs(landing_momentum_stamp_diag_-sample.stamp)<1e-9 &&
            torso_imu_.fresh(now_sec) && now_sec-sample.stamp<=.020 &&
            centroidal_height_.valid(now_sec) && now_sec-centroidal_height_.stamp()<=.020 &&
            std::abs(centroidal_height_.stamp()-sample.stamp)<=.010+1e-9;
        if(!allocator_flight_log_.is_open() && !jump_log_path_.empty()) {
            allocator_flight_log_.open(jump_log_path_+".allocator_flight.csv");
            allocator_flight_log_<<"time,joint_stamp,com_stamp,H,plan_valid,reason,duration,predicted_clearance,predicted_contact_pitch,hip_q,knee_q,hip_v,knee_v,wheel_target,command_hip,command_knee\n";
        }
        if(!allocator_flight_plan_attempted_) {
            if(!paired || !contact_fresh || reference_ground_pulse_wheel_mask_!=0 ||
               !effort_mode_active_ || leg_mode_switch_pending_) return false;
            allocator_flight_plan_attempted_=true;
            // COM is propagated over the explicitly logged source-stamp gap;
            // q/v/H remain from one exact joint/IMU sample in confirmed flight.
            const double gap=sample.stamp-centroidal_height_.stamp();
            const double com_z=centroidal_height_.height()+centroidal_height_.velocity()*gap-.5*9.81*gap*gap;
            const double com_vz=centroidal_height_.velocity()-9.81*gap;
            allocator_flight_plan_=bbot_jump::make_allocator_flight_plan(sample.stamp,sample.q,sample.v,
                sample.wheel_rate,pitch_,landing_momentum_diag_.total,com_z,com_vz,
                ground_height_offset_,body_mass_,kinematics_.get_params());
            allocator_flight_pitch_reference_=pitch_;
            allocator_flight_previous_time_=sample.stamp;
            RCLCPP_WARN(this->get_logger(),"[ALLOCATOR_FLIGHT_PLAN] valid=%d reason=%s H=%.4f T=%.3f clearance=%.3f wheel_amp=%.2f q=%.3f/%.3f v=%.2f/%.2f",
                allocator_flight_plan_.valid,allocator_flight_plan_.reason,landing_momentum_diag_.total,
                allocator_flight_plan_.duration,allocator_flight_plan_.peak_clearance,allocator_flight_plan_.wheel_amplitude,
                sample.q[0],sample.q[1],sample.v[0],sample.v[1]);
            if(allocator_flight_log_.is_open()) {
                allocator_flight_log_<<std::setprecision(17)<<now_sec<<','<<sample.stamp<<','<<centroidal_height_.stamp()<<','
                    <<landing_momentum_diag_.total<<','<<allocator_flight_plan_.valid<<','<<allocator_flight_plan_.reason<<','
                    <<allocator_flight_plan_.duration<<','<<allocator_flight_plan_.peak_clearance<<','<<allocator_flight_plan_.predicted_contact_pitch<<','
                    <<sample.q[0]<<','<<sample.q[1]<<','<<sample.v[0]<<','<<sample.v[1]<<",nan,nan,nan\n";
                allocator_flight_log_.flush();
            }
            if(!allocator_flight_plan_.valid) {protective_landing_=true;return false;}
        }
        if(!allocator_flight_plan_.valid) return false;
        if(contact_fresh && reference_ground_pulse_wheel_mask_!=0) {
            // Use the first available real contact witness for wheel/leg handoff.
            current_state_=bbot_jump::STATE_TOUCHDOWN_BUFFER;state_start_time_=now_sec;
            current_height_=.52; touchdown_buffer_initialized_=false;
            touchdown_torso_convergence_latched_=false;touchdown_torso_convergence_start_time_=-1.;
            touchdown_torso_convergence_blend_=0.; touchdown_stable_count_=0;protective_landing_=false;
            touchdown_x_ref_=x_;touchdown_x_latched_=true;target_x_=x_;was_moving_=false;
            touchdown_catch_active_=true;touchdown_reverse_brake_active_=false;touchdown_reverse_brake_consumed_=false;
            landing_fall_start_time_=-1.;touchdown_catch_stable_count_=0;touchdown_catch_stable_time_=0.;
            touchdown_settle_start_time_=-1.;touchdown_brake_active_=false;touchdown_brake_ready_count_=0;
            touchdown_brake_start_time_=-1.;touchdown_brake_cmd_ref_=0.;
            touchdown_joint_handoff_start_time_=now_sec;
            touchdown_hip_left_start_=hip_pos_left_;touchdown_knee_left_start_=knee_pos_left_;
            touchdown_hip_right_start_=hip_pos_right_;touchdown_knee_right_start_=knee_pos_right_;
            preload_touchdown_effort();request_effort_controller();return true;
        }
        if(now_sec>allocator_flight_plan_.start+allocator_flight_plan_.duration+.10 ||
           !paired || !contact_fresh || !effort_mode_active_ || leg_mode_switch_pending_) {
            allocator_flight_plan_.valid=false;protective_landing_=true;
            RCLCPP_WARN(this->get_logger(),"[ALLOCATOR_FLIGHT_PLAN] stale sensor/contact or effort switch; protected fallback");
            return false;
        }
        bbot_jump::JointVector vr,ar;const auto qr=allocator_flight_plan_.sample(now_sec,vr,ar);
        const double relative=now_sec-allocator_flight_plan_.start;
        double wheel_acc;const double w=allocator_flight_plan_.wheel(relative,&wheel_acc);
        const double ref_rate=bbot_jump::landing_pitch_rate_for_momentum(qr,vr,{w,w},
            allocator_flight_plan_.initial_H,body_mass_);
        const double ref_dt=std::clamp(now_sec-allocator_flight_previous_time_,0.,.020);
        allocator_flight_pitch_reference_+=ref_rate*ref_dt;allocator_flight_previous_time_=now_sec;
        // Wheel spin transfers internal momentum; it cannot remove total H.
        const auto hm=bbot_jump::landing_momentum(sample.q,sample.v,{0.,0.},0.,body_mass_);
        const double desired_pitch_rate=ref_rate+8.*(allocator_flight_pitch_reference_-pitch_);
        const double correction=(hm.pitch_inertia*(desired_pitch_rate-ref_rate))/(2.*bbot_jump::kSupportWheelAxialInertia);
        bbot_jump::SupportActuatorVector planned_motor;
        if(!bbot_jump::allocator_flight_curve_effort(qr,vr,ar,allocator_flight_pitch_reference_,
            ref_rate,w,wheel_acc,body_mass_,planned_motor)) {
            allocator_flight_plan_.valid=false;protective_landing_=true;return false;
        }
        const double wheel_ff=.5*(planned_motor[4]+planned_motor[5])/wheel_servo_gain_;
        const double target_rate=std::clamp(w+wheel_ff+correction,-2./.07,2./.07);
        const double cmd=.07*target_rate;
        std::array<double,2> wheel_effort;
        if(!bbot_jump::support_wheel_servo_effort(cmd,0.,sample.wheel_rate,wheel_servo_gain_,true,true,false,wheel_effort))return false;
        bbot_jump::JointVector acc=ar+100.*(qr-sample.q)+18.*(vr-sample.v);
        for(int i=0;i<4;++i) acc[i]=std::clamp(acc[i],i%2?-500.:-450.,i%2?500.:450.);
        bbot_jump::ThrustSupportDynamicsInput d;d.body_mass=body_mass_;d.q[2]=-pitch_;d.v[2]=-pitch_rate_raw_;
        d.q.segment<4>(3)=sample.q;d.v.segment<4>(3)=sample.v;
        d.v[7]=sample.wheel_rate[0];d.v[8]=sample.wheel_rate[1];
        bbot_jump::JointVector tau;
        if(!bbot_jump::allocator_flight_effort(d,acc,wheel_effort,tau))return false;
        const auto limits=current_effort_limits();
        for(int i=0;i<4;++i) {
            const double limit=i%2?limits.knee:limits.hip;
            if(std::abs(sample.q[i])>(i%2?1.56:1.52)||std::abs(sample.v[i])>(i%2?15.:11.)){
                allocator_flight_plan_.valid=false;protective_landing_=true;return false;
            }
            if(std::abs(tau[i])>limit+1e-6) {
                allocator_flight_plan_.valid=false;
                allocator_flight_plan_.reason="runtime_motor_limit";
                protective_landing_=true;
                RCLCPP_WARN(this->get_logger(),"[ALLOCATOR_FLIGHT_PLAN] runtime motor limit joint=%d requested=%.3f cap=%.3f",i,tau[i],limit);
                if(allocator_flight_log_.is_open()) {
                    allocator_flight_log_<<std::setprecision(17)<<now_sec<<','<<sample.stamp<<','<<centroidal_height_.stamp()<<','
                        <<landing_momentum_diag_.total<<",0,runtime_motor_limit,"<<allocator_flight_plan_.duration<<','
                        <<allocator_flight_plan_.peak_clearance<<','<<allocator_flight_plan_.predicted_contact_pitch
                        <<",0,0,0,0,0,0,0\n";allocator_flight_log_.flush();
                }
                return false;
            }
        }
        // Direct publication bypasses all legacy flight/ground PD and feedforward.
        hip_pos_cmd_left_=qr[0];knee_pos_cmd_left_=qr[1];
        hip_pos_cmd_right_=qr[2];knee_pos_cmd_right_=qr[3];
        joint_velocity_cmd_={vr[0],vr[1],vr[2],vr[3]};
        if(!publish_allocated_effort({tau[0],tau[1],tau[2],tau[3]},now_sec))return false;
        publish_wheel_cmd(cmd,0.);
        // The first 20 ms of the continuous C2 curve arrest inherited joint
        // motion; then tuck/hold, then deploy. All phases retain computed effort.
        flight_subphase_=relative<.020?bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST:
            relative<allocator_flight_plan_.deploy_start?bbot_jump::FLIGHT_SUBPHASE_TUCK:bbot_jump::FLIGHT_SUBPHASE_EXTEND;
        if(relative>=.020)tuck_entered_=true;
        if(relative>=allocator_flight_plan_.deploy_start)extend_entered_=true;
        tuck_started_=true;tuck_start_timestamp_=allocator_flight_plan_.start;
        normal_flight_joint_traj_=relative<allocator_flight_plan_.deploy_start?allocator_flight_plan_.tuck:allocator_flight_plan_.deploy;
        log_data(cmd,0.,0.,0.);
        if(allocator_flight_log_.is_open()) {
            allocator_flight_log_<<std::setprecision(17)<<now_sec<<','<<sample.stamp<<','<<centroidal_height_.stamp()<<','
                <<landing_momentum_diag_.total<<",1,executing,"<<allocator_flight_plan_.duration<<','<<allocator_flight_plan_.peak_clearance<<','
                <<allocator_flight_plan_.predicted_contact_pitch<<','<<qr[0]<<','<<qr[1]<<','<<vr[0]<<','<<vr[1]<<','<<target_rate<<','
                <<actual_tau_hip_left_<<','<<actual_tau_knee_left_<<'\n';allocator_flight_log_.flush();
        }
        return true;
    }

    // ── 阶段 3：腾空相控制 (FLIGHT) ──
    void run_state_flight(double now_sec)
    {
        if (thrust_support_allocator_enable_ && run_allocator_flight(now_sec))
            return;
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
            (ref_ratio > 0.0 && ref_ratio < 1.0) ? (6.0 * ref_ratio * (1.0 - ref_ratio) /
                                                    std::max(0.05, flight_pitch_transition_duration_))
                                                 : 0.0;
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
        auto estimate_remaining_to_touchdown = [&]() -> double
        {
            // base_link 会因空中收腿而上下移动，不能把该内部构型速度当成
            // 整机弹道速度。使用同时间戳的世界系整机 COM，并以目标落地
            // 构型的 COM 高度作为终点，避免 TUCK 中途虚报着陆死线。
            if (centroidal_height_.valid(now_sec))
            {
                const double landing_pitch = balance_offset_ + flight_landing_pitch_bias_;
                const auto landing = inverse_kinematics_with_target_x(
                    L_TOUCH_, landing_pitch, preview_landing_target_x(now_sec));
                const std::array<double, 4> landing_q{
                    landing.theta_hip, landing.theta_knee,
                    landing.theta_hip, landing.theta_knee};
                const auto geometry = bbot_jump::centroidal_geometry(landing_q, body_mass_);
                const double landing_com_world_z = ground_height_offset_ + wheel_radius_ +
                    bbot_jump::rotate_about_hip(
                        -landing_pitch, geometry.com - geometry.axle).z();
                return bbot_jump::ballistic_time_to_height(
                    centroidal_height_.height(), centroidal_height_.velocity(),
                    landing_com_world_z);
            }
            return 0.30;
        };

        auto begin_tuck = [&](const char *reason)
        {
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
                hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_};
            const std::array<double,4> qf=capture_tuck_target(q0,v0,tuck_ik.theta_hip,tuck_ik.theta_knee);

            for (size_t i = 0; i < 4; ++i)
            {
                normal_flight_joint_traj_[i].init(
                    now_sec, flight_round_trip_plan_.tuck_duration,
                    q0[i], v0[i], 0.0,
                    qf[i], 0.0, 0.0);
            }
            flight_trajectory_initialized_ = true;
            flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_TUCK;
            tuck_entered_ = true;
            tuck_start_time_ = now_sec;
            tuck_start_timestamp_ = now_sec;
            tuck_start_z_ = current_z_;
            tuck_started_ = true;
            tuck_traj_.init(now_sec, flight_round_trip_plan_.tuck_duration, tuck_start_z_, 0.0, 0.0,
                            L_RETRACT_, 0.0, 0.0);
            L_target = tuck_start_z_;
            RCLCPP_INFO(this->get_logger(),
                        ">>> [FEASIBLE_TUCK v6.14] %s：t=%.3fs remaining=%.3fs，"
                        "L %.3f -> %.3f m，T=%.3f+%.3fs；q=(%.3f,%.3f)->(%.3f,%.3f) <<<",
                        reason, elapsed, estimate_remaining_to_touchdown(),
                        tuck_start_z_, L_RETRACT_, flight_round_trip_plan_.tuck_duration,
                        flight_round_trip_plan_.extend_duration,
                        hip_pos_left_, knee_pos_left_, qf[0], qf[1]);
        };

        auto begin_protective_deploy = [&](double start_target)
        {
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

        auto begin_checked_tuck = [&](const char *reason)
        {
            const double remaining = estimate_remaining_to_touchdown();
            flight_round_trip_plan_ = plan_tuck_round_trip(now_sec, remaining);
            flight_tuck_plan_check_ = flight_round_trip_plan_.valid ? 1 : 0;
            if (flight_tuck_plan_check_ == 1)
            {
                begin_tuck(reason);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "[FLIGHT_PLAN v6.14] 收展腿轨迹超出速度/加速度/时间预算，转保护展腿 "
                            "remaining=%.3f q=(%.3f,%.3f) v=(%.2f,%.2f)",
                            remaining, hip_pos_left_, knee_pos_left_, hip_vel_left_, knee_vel_left_);
                begin_protective_deploy(current_height_);
            }
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
            const bool arrest_condition = protective_landing_ ? rotation_arrested : attitude_stable;
            if (arrest_condition)
            {
                attitude_arrest_stable_count_++;
            }
            else
            {
                attitude_arrest_stable_count_ = 0;
            }

            // v5.6：高速腿正是 ATTITUDE_ARREST 要处理的对象，不能再把
            // !flight_leg_motion_ready() 当成“立即展腿”的条件。否则离地膝速 -10rad/s
            // 时会直接绕过 arrest 轨迹。这里只对接近硬限位的构型立即保护；
            // 正常高速关节至少给 90ms 去沿 arrest 五次轨迹减速。
            const bool leg_position_unsafe =
                std::abs(hip_pos_left_) > 1.52 || std::abs(hip_pos_right_) > 1.52 ||
                std::abs(knee_pos_left_) > 1.5708 || std::abs(knee_pos_right_) > 1.5708;

            // v6.1：离地时如果“当前”关节速度已经低，不能再固定等65~110ms。
            // 上一轮真正离地时 hip_v≈1.1、knee_v≈-1.3，已经足够安全，却仍被
            // ATTITUDE_ARREST + protective 路径拖到整段不收腿。现在最早25ms即可进入TUCK。
            const double max_hip_speed = std::max(
                std::abs(hip_vel_left_), std::abs(hip_vel_right_));
            const double max_knee_speed = std::max(
                std::abs(knee_vel_left_), std::abs(knee_vel_right_));
            const bool tuck_joint_speed_safe =
                flight_leg_motion_ready() &&
                max_hip_speed <= (velocity_capture_enable_ ? 0.50 : 5.0) &&
                max_knee_speed <= (velocity_capture_enable_ ? 0.50 : 6.5);
            const double remaining_time = estimate_remaining_to_touchdown();
            const double tuck_time_need =
                T_FLIGHT_TUCK_ + T_FLIGHT_EXTEND_ + 0.025;
            const bool enough_time_for_tuck =
                remaining_time >= tuck_time_need;
            const bool early_tuck_attitude_ok =
                std::abs(air_pitch_err) <= 0.24 &&
                std::abs(air_pitch_rate_err) <= 1.10;

            // 这些只是入口条件。begin_checked_tuck 还会完整校验整条
            // 收腿/展腿指令轨迹的速度、加速度与时间预算。
            const bool early_tuck_ready =
                !protective_landing_ &&
                !leg_position_unsafe &&
                elapsed >= 0.015 &&
                tuck_joint_speed_safe &&
                early_tuck_attitude_ok &&
                enough_time_for_tuck;

            if (early_tuck_ready)
            {
                begin_checked_tuck("当前真实关节已低速，轨迹预算通过");
            }
            // 诊断只走控制台：149 列 CSV 有下游消费者，不加列。
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 80,
                                 "[ARREST_TUCK_GATE v6.14] ready=%d unsafe_pos=%d speed_ok=%d "
                                 "att_ok=%d time_ok=%d | hip_v=%.2f knee_v=%.2f rem=%.3f need=%.3f "
                                 "pe=%.3f re=%.3f | tuck_chk=%d T=%.3f+%.3f",
                                 early_tuck_ready ? 1 : 0, leg_position_unsafe ? 1 : 0,
                                 tuck_joint_speed_safe ? 1 : 0, early_tuck_attitude_ok ? 1 : 0,
                                 enough_time_for_tuck ? 1 : 0, max_hip_speed, max_knee_speed,
                                 remaining_time, tuck_time_need, air_pitch_err, air_pitch_rate_err,
                                 flight_tuck_plan_check_, flight_round_trip_plan_.tuck_duration,
                                 flight_round_trip_plan_.extend_duration);

            if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST)
            {
                // protective_takeoff 只代表当前真实姿态/关节仍不安全；最多给50ms减速，
                // 然后转入wheel-first保护轨迹，避免把所有运动拖到下降末段。
                const bool protective_deploy_due =
                    protective_landing_ && elapsed >= 0.050;
                const bool normal_arrest_timeout =
                    !protective_landing_ && elapsed >= (velocity_capture_enable_ ? 0.120 : 0.080);

                const bool arrest_exit_guard =
                    leg_position_unsafe || protective_deploy_due ||
                    (attitude_unstable && elapsed >= 0.065) ||
                    normal_arrest_timeout;
                // 正常跳若已经具备TUCK速度条件但只是姿态/ARREST时间到期，
                // 优先尝试原 round-trip 校验；硬位置危险仍保留这一既有优先级。
                const bool late_tuck_possible =
                    !protective_landing_ &&
                    tuck_joint_speed_safe &&
                    enough_time_for_tuck &&
                    std::abs(air_pitch_err) <= 0.28 &&
                    std::abs(air_pitch_rate_err) <= 1.25;
                const auto stable_arrest_action = bbot_jump::stable_arrest_action(
                    elapsed, attitude_arrest_stable_count_, protective_landing_,
                    enough_time_for_tuck, tuck_joint_speed_safe,
                    arrest_exit_guard, late_tuck_possible);
                if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST)
                {
                    if (stable_arrest_action == bbot_jump::StableArrestAction::protective_deploy)
                    {
                        if (protective_landing_ && !arrest_exit_guard)
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        ">>> [姿态角速度已刹住] t=%.3fs！开始保护展腿 <<<",
                                        elapsed);
                        }
                        else if (!arrest_exit_guard)
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        "[跳过收腿] 剩余飞行时间不足 (remaining=%.3f need=%.3f)，直接规划落地",
                                        remaining_time, tuck_time_need);
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "[腾空保护] arrest结束/超限 (elapsed=%.3fs, pitch=%.3f, gyro=%.3f, "
                                        "hip_v=%.2f knee_v=%.2f remaining=%.3f)，转入快速wheel-first展腿",
                                        elapsed, pitch_err, pitch_rate_,
                                        hip_vel_left_, knee_vel_left_, remaining_time);
                        }
                        begin_protective_deploy(current_height_);
                    }
                    else if (stable_arrest_action == bbot_jump::StableArrestAction::start_checked_tuck)
                    {
                        begin_checked_tuck(arrest_exit_guard
                                               ? "ARREST超时，轨迹预算通过"
                                               : "姿态、关节速度与轨迹预算均已满足");
                    }
                }
            }
            break;
        }

        case bbot_jump::FLIGHT_SUBPHASE_TUCK:
        {
            const bool tuck_finished =
                (now_sec - tuck_start_time_) >= flight_round_trip_plan_.tuck_duration;
            const double remaining_time = estimate_remaining_to_touchdown();
            // v6.1：低跳不等固定0.22s apex。若剩余时间只够“展腿+55ms裕量”，
            // TUCK一完成就立即EXTEND，避免短腿保持过久后又来不及wheel-first。
            const bool landing_deadline_reached =
                remaining_time <=
                flight_round_trip_plan_.extend_duration + landing_deploy_ready_margin_ + 0.020;

            if (attitude_unstable)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[腾空保护] 收腿期间姿态超限 (pitch=%.3f)，停止收腿转为展腿保护",
                            pitch_err);
                begin_protective_deploy(current_height_);
            }
            else if ((tuck_finished && (!velocity_capture_enable_ ||
                       (centroidal_height_.valid(now_sec) && centroidal_height_.velocity() <= 0.0))) ||
                     landing_deadline_reached)
            {
                // v6.2：低跳没有等待apex的余量。TUCK完成后立即EXTEND；
                // 若剩余时间提前触及着陆deadline，即使TUCK尚差几毫秒也立即转展腿。
                latch_landing_capture_plan(now_sec);
                // TUCK 的真实关节可能落后于参考，不能从虚构的收腿终点开始
                // 展腿。以当前实测 q/v 为连续边界，并在剩余 COM 弹道时间
                // 内搜索最短可行时长；这既不放宽速度/加速度上限，也不制造
                // 位置或速度阶跃。
                double feasible_extend_duration = 0.0;
                const double max_extend_duration =
                    remaining_time - landing_deploy_ready_margin_;
                // 使用剩余弹道预算内最长的可行段，而不是最短段。上一轮
                // 90 ms 轨迹让落后于参考的真实髋关节瞬间追到约 -11 rad/s；
                // 多出的 20~30 ms 可显著降低加速度和机身反作用，同时仍
                // 保留 landing_deploy_ready_margin_ 的触地前稳定时间。
                const double first_candidate = velocity_capture_enable_ ? T_FLIGHT_EXTEND_ :
                    std::floor(max_extend_duration / 0.005) * 0.005;
                for (double candidate = first_candidate;
                     velocity_capture_enable_ ? candidate <= max_extend_duration+1e-9 :
                                               candidate >= T_FLIGHT_EXTEND_-1e-9;
                     candidate += velocity_capture_enable_ ? 0.005 : -0.005)
                {
                    if (normal_landing_plan_feasible(candidate))
                    {
                        feasible_extend_duration = candidate;
                        break;
                    }
                }
                flight_extend_plan_check_ = feasible_extend_duration > 0.0 ? 1 : 0;
                if (flight_extend_plan_check_ == 0)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "[FLIGHT_PLAN v6.14] 当前真实关节状态无法在剩余时间内安全展腿，转保护展腿");
                    begin_protective_deploy(current_height_);
                    break;
                }
                flight_round_trip_plan_.extend_duration = feasible_extend_duration;
                // plan_flight_joints 在未初始化时读取当前真实 q/v；显式清除
                // 旧 TUCK 参考，保证新段首帧与执行器状态连续。
                flight_trajectory_initialized_ = false;
                plan_flight_joints(now_sec, flight_round_trip_plan_.extend_duration, L_TOUCH_,
                                   false,
                                   balance_offset_ + flight_landing_pitch_bias_,
                                   landing_target_x_);
                flight_subphase_ = bbot_jump::FLIGHT_SUBPHASE_EXTEND;
                extend_entered_ = true;
                extend_start_time_ = now_sec;

                extend_traj_.init(
                    now_sec,
                    flight_round_trip_plan_.extend_duration,
                    current_height_, 0.0, 0.0,
                    L_TOUCH_, 0.0, 0.0);

                L_target = current_height_;
                RCLCPP_INFO(this->get_logger(),
                            ">>> [TUCK->EXTEND] t=%.3fs remaining=%.3fs tuck_done=%d deadline=%d <<<",
                            elapsed, remaining_time,
                            tuck_finished ? 1 : 0,
                            landing_deadline_reached ? 1 : 0);
            }
            else
            {
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
            // EXTEND 已从当前实测 q/v 通过完整速度、加速度和行程校验。
            // 姿态超限时切换到另一条 protective 轨迹不能修正机身姿态，
            // 反而会重置关节边界并注入第二次扰动；保持已校验轨迹连续，
            // 由反作用轮和落地捕获处理姿态误差。
            double extend_z;
            double extend_v;
            double extend_acc;

            extend_traj_.evaluate(
                now_sec,
                extend_z,
                extend_v,
                extend_acc);

            L_target = extend_z;
            break;
        }

        case bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY:
        {
            if (!protective_deploy_initialized_)
            {
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
                protective_deploy_replan_count_ < 2)
            {
                const double remaining_time =
                    estimate_remaining_to_touchdown();
                const double usable =
                    remaining_time - landing_deploy_ready_margin_;
                if (usable >= 0.055)
                {
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

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 80,
                             "[FLIGHT_PLAN v6.14] sub=%s tuck_chk=%d ext_chk=%d T=%.3f+%.3f "
                             "L_cmd=%.3f z=%.3f vz=%.2f q_k=(%.2f,%.2f) "
                             "v_k=(%.2f,%.2f) v_h=(%.2f,%.2f) pitch=%.3f rate=%.2f",
                             bbot_jump::flight_subphase_to_string(flight_subphase_),
                             flight_tuck_plan_check_, flight_extend_plan_check_,
                             flight_round_trip_plan_.tuck_duration,
                             flight_round_trip_plan_.extend_duration,
                             L_target, current_z_, current_z_dot_,
                             knee_pos_left_, knee_pos_right_,
                             knee_vel_left_, knee_vel_right_,
                             hip_vel_left_, hip_vel_right_, pitch_, pitch_rate_);

        // 保持力矩控制模式
        const bool extend_finished =
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND &&
            (now_sec - extend_start_time_) >= flight_round_trip_plan_.extend_duration;

        const bool protective_deploy_finished =
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY &&
            protective_deploy_initialized_ &&
            (now_sec - protective_deploy_start_time_) >= protective_deploy_duration_active_;

        const bool legs_deployed =
            protective_deploy_finished ||
            extend_finished;
        if (!effort_mode_active_ && !leg_mode_switch_pending_)
        {
            request_effort_controller();
        }

        const bool landing_approach =
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND ||
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY;

        // 所有空中子阶段均从独立关节轨迹解析获得位置和速度。
        // 左右速度不能取平均，否则不对称离地时首帧即产生额外阻尼冲击。
        std::array<double, 4> q_des{}, qdot_des{}, qddot_des{};
        sample_flight_joints(now_sec, q_des, qdot_des, qddot_des);
        flight_trajectory_initialized_ = true;

        // v5.9：落地接近时显式保证 wheel-first。
        // 不管五次轨迹的瞬态如何，都不允许期望小腿接近水平。
        bool landing_geom_clamped = false;
        if (landing_approach)
        {
            landing_geom_clamped |= enforce_wheel_first_target(
                q_des[0], q_des[1], qdot_des[0], qdot_des[1],
                air_pitch_ref, air_pitch_rate_ref);
            landing_geom_clamped |= enforce_wheel_first_target(
                q_des[2], q_des[3], qdot_des[2], qdot_des[3],
                air_pitch_ref, air_pitch_rate_ref);

            const double phi2_0 = std::atan2(0.28210870, 0.19553796);
            const auto &p = kinematics_.get_params();
            const double shank_l = phi2_0 + q_des[0] + q_des[1] - air_pitch_ref;
            const double shank_r = phi2_0 + q_des[2] + q_des[3] - air_pitch_ref;
            const double knee_axis_l = p.l1 * std::cos(shank_l);
            const double knee_axis_r = p.l1 * std::cos(shank_r);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 80,
                "[LANDING_GEOM] sub=%s shankL=%.3f shankR=%.3f limit=%.3f "
                "kneeAxisL=%.3f kneeAxisR=%.3f clamped=%d deploy=%.3f/%.3f",
                bbot_jump::flight_subphase_to_string(flight_subphase_),
                shank_l, shank_r, landing_shank_limit(),
                knee_axis_l, knee_axis_r, landing_geom_clamped ? 1 : 0,
                (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY) ? (now_sec - protective_deploy_start_time_) : 0.0,
                protective_deploy_duration_active_);
        }

        // The underlying FLIGHT trajectory still starts at measured q/v.
        // During this short handoff only the velocity servo target is adapted;
        // it is not claimed to be the derivative of the unchanged q_des.
        if (thrust_reference_handoff_enable_ && !thrust_reference_handoff_flight_done_)
        {
            const auto before = thrust_reference_handoff_.sample();
            const double reference_dt = now_sec-thrust_reference_handoff_.stamp();
            const bool handoff_fresh = now_sec >= joint_sample_time_ && now_sec-joint_sample_time_ <= .020 &&
                now_sec >= torso_imu_.stamp() && now_sec-torso_imu_.stamp() <= .020;
            const std::array<double,4> actual_q{hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_};
            bool join = before.valid && handoff_fresh && effort_mode_active_ && !leg_mode_switch_pending_ &&
                elapsed >= .020 && reference_dt > 0. && reference_dt <= .020 &&
                bbot_jump::handoff_actual_travel_safe(actual_q,
                    {hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_});
            for (size_t i=0;i<4;++i)
                join = join && std::isfinite(qdot_des[i]) &&
                    std::abs(qdot_des[i]) <= (i%2 ? 13. : 11.) &&
                    std::abs(actual_q[i]+.030*qdot_des[i]) <= (i%2 ? 1.56 : 1.52);
            for (size_t i=0;i<4;++i)
                join = join && std::abs(qdot_des[i]-before.velocity[i]) <=
                    (i%2 ? 500. : 450.)*reference_dt;
            handoff_reference_target_diag_=qdot_des;
            if (join)
            {
                for (size_t i=0;i<4;++i)
                    handoff_reference_rate_diag_[i]=(qdot_des[i]-before.velocity[i])/reference_dt;
                handoff_reference_applied_diag_=qdot_des;
                handoff_reference_valid_diag_=true;
                handoff_reference_mode_diag_=5;
                thrust_reference_handoff_flight_done_=true;
                thrust_reference_handoff_.reset();
            }
            else
            {
                const bool fresh = now_sec >= joint_sample_time_ && now_sec-joint_sample_time_ <= .020 &&
                    now_sec >= torso_imu_.stamp() && now_sec-torso_imu_.stamp() <= .020;
                const auto handoff=thrust_reference_handoff_.update(now_sec,qdot_des,
                    {hip_pos_left_,knee_pos_left_,hip_pos_right_,knee_pos_right_},{0.,0.},
                    fresh,effort_mode_active_ && !leg_mode_switch_pending_,
                    bbot_jump::handoff_actual_travel_safe(actual_q,
                        {hip_vel_left_,knee_vel_left_,hip_vel_right_,knee_vel_right_}),false,13.);
                handoff_reference_valid_diag_=handoff.valid;
                handoff_reference_limited_diag_=handoff.limited;
                handoff_reference_mode_diag_=4;
                handoff_reference_applied_diag_=handoff.velocity;
                handoff_reference_rate_diag_=handoff.rate;
                if (!handoff.valid)
                {
                    handoff_reference_mode_diag_=6;
                    abort_jump_to_recovery(now_sec,"实验空中交接数据/行程不可行");
                    return;
                }
                qdot_des=handoff.velocity;
            }
        }

        last_thrust_force_ = 0.0;
        last_thrust_force_request_ = 0.0;
        last_thrust_force_limit_ = 0.0;
        last_tau_body_per_hip_ = 0.0;

        if (effort_mode_active_)
        {
            // 空中阶段：全程保持机身俯仰姿态稳定与角动量平衡
            double kp_hip_fl = 12.0;
            double kd_hip_fl = 1.5;
            double kp_knee_fl = 15.0;
            double kd_knee_fl = 1.5;
            if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST)
            {
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
            }
            else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY)
            {
                // 空中髋、膝只跟踪安全落地构型，机身姿态交给反作用轮。
                // 提高阻尼而不是只堆刚度，抑制上一轮中约 10 Hz 的关节摆动。
                const double gain_blend = bbot_jump::clamp_value(
                    (now_sec - protective_deploy_start_time_) / 0.10, 0.0, 1.0);
                kp_hip_fl = bbot_jump::lerp(18.0, 32.0, gain_blend);
                kd_hip_fl = bbot_jump::lerp(4.5, 7.0, gain_blend);
                kp_knee_fl = bbot_jump::lerp(22.0, 40.0, gain_blend);
                kd_knee_fl = bbot_jump::lerp(5.0, 8.0, gain_blend);
            }
            else if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND)
            {
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

            bbot_jump::JointVector arrest_dynamics_ff = bbot_jump::JointVector::Zero();
            const bool arrest_phase =
                flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST;
            const bbot_jump::JointVector measured_q(
                hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_);
            const bbot_jump::JointVector measured_v(
                hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_);
            const bbot_jump::JointVector desired_a(
                qddot_des[0], qddot_des[1], qddot_des[2], qddot_des[3]);
            const auto ff_guard = bbot_jump::arrest_dynamics_guard(
                arrest_dynamics_feedforward_enable_, arrest_phase,
                effort_mode_active_, leg_mode_switch_pending_, now_sec,
                joint_sample_time_, torso_imu_.stamp(), measured_q, measured_v,
                desired_a, pitch_rate_raw_);
            arrest_dynamics_feedforward_guard_diag_ = static_cast<int>(ff_guard);
            if (ff_guard == bbot_jump::ArrestDynamicsGuard::Active)
            {
                bbot_jump::JointVector inertial = bbot_jump::JointVector::Zero();
                bbot_jump::JointVector bias = bbot_jump::JointVector::Zero();
                const auto raw = bbot_jump::flight_joint_dynamics_feedforward(
                    measured_q, measured_v, desired_a, -pitch_rate_raw_, body_mass_,
                    &inertial, &bias);
                const double arrest_elapsed = std::max(0.0, now_sec - attitude_arrest_start_time_);
                const double blend = bbot_jump::arrest_dynamics_feedforward_blend(arrest_elapsed);
                const auto limits = current_effort_limits();
                const bbot_jump::JointVector effort_limits(
                    limits.hip, limits.knee, limits.hip, limits.knee);
                arrest_dynamics_ff = bbot_jump::bound_arrest_dynamics_feedforward(
                    raw, effort_limits, blend);
                arrest_dynamics_feedforward_scope_diag_ = true;
                arrest_dynamics_feedforward_blend_diag_ = blend;
                for (int i = 0; i < 4; ++i)
                {
                    arrest_dynamics_qdd_diag_[i] = desired_a[i];
                    arrest_dynamics_inertial_diag_[i] = inertial[i];
                    arrest_dynamics_bias_diag_[i] = bias[i];
                    arrest_dynamics_raw_diag_[i] = raw[i];
                    arrest_dynamics_bounded_diag_[i] = arrest_dynamics_ff[i];
                }
            }

            publish_effort_leg_control_lr(
                q_des[0], q_des[1], q_des[2], q_des[3],
                qdot_des[0], qdot_des[1], qdot_des[2], qdot_des[3],
                tau_hip_air, 0.0,
                tau_hip_air, 0.0,
                kp_hip_fl, kd_hip_fl, kp_knee_fl, kd_knee_fl,
                tau_hip_air, tau_hip_air, arrest_dynamics_ff);
        }
        else
        {
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
        double landing_geom_left = 0.0, landing_geom_right = 0.0;
        const bool landing_geometry_valid = aligned_takeoff_geometry(
            now_sec, landing_geom_left, landing_geom_right);
        if (landing_geometry_valid)
            wheel_clearance_ = gazebo_world_z_ - ground_height_offset_ -
                               std::max(landing_geom_left, landing_geom_right);
        // 0.035 是试验/调参值，且它已经不再是主通道：v6.28 之后 IMU 判据在 FLIGHT
        // 起跳后 0.27~0.37 s 就先切换状态，全高/载荷 42 条日志里这条几何窗口一次都没开
        // （FLIGHT 退出帧净空 +0.042~+0.052 m）；低高度 10 条里都开了，但退出帧净空
        // 只有 +0.032~+0.038 m，正好压在门限两侧 ⇒ 往下调会让低高度也失效，保持不动。
        contact_window_ = landing_geometry_valid && world_descending &&
                          wheel_clearance_ <= 0.035;

        // 触地判定的独立通道：躯干 IMU 比力（fz 是 specific force，不是世界加速度）。
        // 上面那份净空用的是离地瞬间锁存的关节构型（aligned_takeoff_geometry），
        // 腿在空中收展之后它就不再是轮底到地面的距离：21/21 条 v6.27 及更早的日志里
        // contact_window_ 确实开过，但要到 FLIGHT 已经跑了 0.9~1.5 s 之后才开
        // （退出帧净空 −0.013~+0.035 m），机身明明已被托住（fz≈9.5 m/s²）时它仍读
        // +0.04~0.05 m ⇒ FLIGHT 比真实落地晚 0.6~1.3 s 才退出，其间空中轮律把
        // +0.7 m/s 的前进速度拖成 −1.0~−1.75 m/s 的后退速度。实测自由飞行 fz 在
        // −18~+5.3，支撑时中位 9.79 ⇒ 门限取 7.0（试验/调参值），并要求 50 ms 持续。
        // 新鲜度必须直接问观察者：torso_imu_fresh_ 只在支撑腿 PD 分支里赋值，
        // 其它状态一律置 false，在 FLIGHT 里恒为 false（第一轮 v6.28 实跑因此空转）。
        constexpr double k_support_fz = 7.0;
        constexpr double k_support_hold = 0.050;
        const bool torso_supported = torso_imu_.fresh(now_sec) &&
                                     torso_imu_.fz() >= k_support_fz &&
                                     gazebo_world_z_dot_ < 0.30;
        if (torso_supported)
        {
            if (landing_support_start_time_ < 0.0)
                landing_support_start_time_ = now_sec;
        }
        else
        {
            landing_support_start_time_ = -1.0;
        }
        const bool sustained_support = landing_support_start_time_ >= 0.0 &&
                                       now_sec - landing_support_start_time_ >= k_support_hold;

        // 3. 空中飞轮效应姿态控制 (动量轮反作用扭矩)
        // air_pitch_ref / air_pitch_rate_ref 已在本状态函数顶部统一生成；
        // 髋部小辅助力矩和反作用轮共同跟踪它，避免 reference 打架。
        // 降低旧版过强的 D 制动；现在 D 项针对参考角速度误差，而不是强迫 gyro=0。
        constexpr double k_air_p = 0.50;
        const double k_air_d = (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_EXTEND)
                                   ? air_wheel_extend_kd_
                                   : (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_TUCK)
                                         ? (pre_jump_effort_capture_ ? 2.50 : air_wheel_tuck_kd_)
                                         : 0.45;

        double arrest_wheel_ff = 0.0;
        // Reaction-wheel damping needs the current direction of rotation;
        // the legacy alpha=.15 gyro remains unchanged for leg/phase gating.
        const double air_wheel_rate_error =
            (torso_imu_.fresh(now_sec) ? torso_imu_.rate() : pitch_rate_) - air_pitch_rate_ref;
        double cmd_target = bbot_jump::flight_wheel_target(
            air_wheel_baseline_,
            air_pitch_err,
            air_wheel_rate_error,
            0.5 * (std::abs(knee_vel_left_) + std::abs(knee_vel_right_)),
            elapsed,
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST,
            air_wheel_sign_,
            k_air_p,
            k_air_d,
            air_wheel_linear_speed_limit_,
            &arrest_wheel_ff,
            &air_wheel_cmd_raw_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 80,
            "[FLIGHT_ATT] t=%.3f sub=%s pitch=%.3f/%.3f gyro=%.3f/%.3f "
            "perr=%.3f rerr=%.3f aff=%.3f wheel_raw=%.3f",
            elapsed, bbot_jump::flight_subphase_to_string(flight_subphase_),
            pitch_, air_pitch_ref, air_wheel_rate_error + air_pitch_rate_ref, air_pitch_rate_ref,
            air_pitch_err, air_wheel_rate_error, arrest_wheel_ff, air_wheel_cmd_raw_);

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

        // Opt-in effort-servo diagnostic; keep wheel target and leg trajectory,
        // but mark a short interval for a true zero-commanded-effort test.
        flight_arrest_freewheel_active_ = flight_arrest_freewheel_duration_ > 0.0 &&
            elapsed >= 0.0 && elapsed < flight_arrest_freewheel_duration_ &&
            flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 70,
                             "[LANDING_WHEEL_HANDOFF] clr=%.3f aligned=%d descending=%d blend=%.2f air=%.3f ground=%.3f target=%.3f xdot=%.2f",
                             wheel_clearance_, landing_geometry_valid ? 1 : 0, world_descending ? 1 : 0,
                             landing_wheel_ground_blend_, air_wheel_cmd_raw_, landing_ground_cmd, cmd_target,
                             x_dot_);

        // 速度型轮控需要尽快建立轮加速度才能产生反作用力矩；0.12/周期
        // 对当前约 1.3 rad/s 的离地角速度制动偏慢。
        const double max_air_wheel_step =
            (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST) ? 0.25 : 0.18;
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_air_wheel_step, max_air_wheel_step);
        publish_wheel_cmd(cmd_x, 0.0, flight_arrest_freewheel_active_);

        const bool leg_compressed = legs_deployed && contact_window_ &&
                                    current_z_ < L_target - 0.010 && current_z_dot_ < -0.10;
        if (contact_window_ &&
            (std::abs(knee_effort_left_) > 15.0 || std::abs(knee_effort_right_) > 15.0))
        {
            touchdown_knee_effort_count_++;
        }
        else
        {
            touchdown_knee_effort_count_ = 0;
        }
        // 膝力矩可以由控制器自身产生，必须同时观测到压缩或 IMU 冲击。
        // 冲击后世界速度可能立即回零，因此保留短暂的下降历史。
        const bool imu_impact = contact_window_ && acc_z_filt_ > 15.0;
        const bool torque_spike = touchdown_knee_effort_count_ >= 2 &&
                                  current_z_dot_ < -0.10 && wheel_clearance_ <= 0.010;

        log_data(cmd_x, 0.0, 0.0, 0.0);

        const bool persistent_contact = touchdown_confirmation_.update(
            takeoff_odom_stamp_, now_sec, landing_geometry_valid, wheel_clearance_,
            world_descending, acc_z_filt_);
        if (persistent_contact || leg_compressed || torque_spike || imu_impact || sustained_support)
        {
            const char *trig = sustained_support ? "持续支撑" : (persistent_contact ? "持续接触" : (leg_compressed ? "腿压缩" : (torque_spike ? "膝力矩" : "IMU冲击")));
            RCLCPP_INFO(this->get_logger(),
                        ">>> 触地检测触发 [%s] (t=%.3fs, z=%.3f, air=%.2fs, vx=%.2f, fz=%.1f, "
                        "wvx=%.2f, clr=%.3f)！进入缓冲阻抗控制",
                        trig, elapsed, current_z_, now_sec - state_start_time_, x_dot_,
                        torso_imu_.fz(), predicted_landing_forward_velocity(now_sec),
                        wheel_clearance_);
            current_state_ = bbot_jump::STATE_TOUCHDOWN_BUFFER;
            state_start_time_ = now_sec;
            touchdown_buffer_initialized_ = false;
            touchdown_torso_convergence_latched_ = false;
            touchdown_torso_convergence_start_time_ = -1.0;
            touchdown_torso_convergence_blend_ = 0.0;
            touchdown_stable_count_ = 0;
            protective_landing_ = false;
            if (!touchdown_x_latched_)
            {
                touchdown_x_ref_ = x_;
                touchdown_x_latched_ = true;
            }
            landing_pitch_err_ = pitch_ - balance_offset_;
            target_x_ = touchdown_x_ref_;
            was_moving_ = false;
            air_wheel_cmd_raw_ = 0.0;
            // 保留 FLIGHT 最后一帧轮速指令，避免触地瞬间人为把姿态控制截断。
            touchdown_catch_active_ = true;
            touchdown_reverse_brake_active_ = false;
            touchdown_reverse_brake_consumed_ = false;
            landing_fall_start_time_ = -1.0;
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
        }
        else if (elapsed >= T_FLIGHT_TIMEOUT_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[腾空超时] 尚未检测到触地，保持展腿等待真实接触 (z=%.3f, vz=%.2f)",
                current_z_, current_z_dot_);
        }
    }

    double landing_capture_target(double fallback)
    {
        if (!capture_world_valid_)
            return fallback;
        // Use the existing diff_drive YAML speed ceiling (5 m/s). The old
        // 0.9/1.5 m/s catch ceiling could be below the moving COM speed.
        // Capture, braking and recovery share this target and 8 m/s² slew limit.
        const double shank_rate = 0.5 * (hip_vel_left_ + knee_vel_left_ +
                                         hip_vel_right_ + knee_vel_right_) -
                                  torso_imu_.rate();
        double velocity_reference = 0.0;
        // 该律的速度不动点是 v = v_ref - 4*(omega*r + R*r_dot)。落地构型下 r 带着
        // 结构性偏置（实测 -0.006~-0.012 m），于是整机以 0.2255 m/s（空载）/
        // 0.3330 m/s（1 kg）永久前滚，顶穿 CATCH 释放门限 |v|<0.15（平地 8/8 不交权）。
        // omega*r 这一路**不能整块抵掉**：各缓冲阶段自己的 catch_wheel_target P/D 项
        // 在 capture_world_valid_ 为真时会被整个丢弃，ωr 就是缓冲段仅剩的姿态反馈。
        // 直接把 v_ref 设成 4*(ωr+Rṙ)（=抵掉它）实跑 v631flatB_t1,t2,t3,t5 的后果是机身
        // 一路倒到门限边缘 com_lean=-0.100、门每 0.1~0.3 s 抖一次（81 OFF / 82 ON），
        // 速度只降到 0.042~0.062 m/s、p_fin 由 +0.058 退到 +0.071~+0.079，仍然不交权。所以这里只慢慢学掉它的**偏置**：
        // 静稳期内 v_ref' = -k_i*v，收敛点恰是 v_ref = 4*(ωr+Rṙ) ⇒ v→0，
        // 而快时间尺度上的 ωr 反馈原样保留。k_i=1.0 /s、限幅 ±0.60 m/s、
        // 退出静稳区后以 1.0 m/s/s 归零，均为试验/调参值。
        const bool moving_effort_jump = pre_jump_effort_capture_ && pre_jump_capture_armed_ &&
            (current_state_ == bbot_jump::STATE_PRE_JUMP || current_state_ == bbot_jump::STATE_SQUAT);
        if (moving_effort_jump)
            velocity_reference = capture_v_ref_ + target_speed_smoothed_;
        else if (capture_anchor_active_)
            velocity_reference = capture_v_ref_;
        capture_world_target_ = bbot_jump::centroidal_catch_target(
            capture_com_velocity_, centroidal_balance_.forward, centroidal_balance_.height,
            shank_rate, kinematics_.get_params().wheel_radius, 5.0, velocity_reference);
        capture_world_active_ = true;
        return capture_world_target_;
    }

    // 门限取 CATCH 释放条件里的姿态子集（0.10 rad / 0.35 rad/s），故意不含速度项：
    // 本门只负责"姿态已经安静"，绝不能因为比释放更严而挡住交权。
    // FLIGHT/TUCK 不启用——空中轮律没有滚动约束，那条路径是 17/17 验收的原样。
    bool capture_creep_anchor_ready() const
    {
        if (!centroidal_balance_.valid || !centroidal_lean_rate_.valid(this->now().seconds()))
            return false;
        if (current_state_ != bbot_jump::STATE_TOUCHDOWN_BUFFER &&
            current_state_ != bbot_jump::STATE_RECOVERY &&
            !(current_state_ == bbot_jump::STATE_BALANCE && post_landing_effort_support_) &&
            !(current_state_ == bbot_jump::STATE_PRE_JUMP && pre_jump_effort_capture_ &&
              !pre_jump_capture_armed_))
            return false;
        return std::abs(ground_balance_angle()) < 0.10 &&
               std::abs(ground_balance_rate()) < 0.35;
    }

    // 每个控制周期在状态分支之前调用一次：先判门，再更新积分参考速度。
    void update_capture_velocity_reference(double dt)
    {
        const bool moving_effort_jump = pre_jump_effort_capture_ && pre_jump_capture_armed_ &&
            (current_state_ == bbot_jump::STATE_PRE_JUMP || current_state_ == bbot_jump::STATE_SQUAT);
        if (moving_effort_jump)
        {
            // Keep the learned stationary bias fixed and move its equilibrium
            // with the smoothed approach velocity; do not relearn v_ref toward
            // zero while the robot is intentionally rolling forward.
            capture_anchor_active_ = true;
            return;
        }
        // A missed/misaligned sensor sample invalidates stability evidence,
        // but it does not mean the robot physically left its captured
        // equilibrium. Hold the learned bias until fresh measurements can
        // confirm real motion; otherwise one stale frame erases the balance
        // point and causes a late forward creep on the next hop.
        const double now_sec = this->now().seconds();
        if (!centroidal_balance_.valid || !centroidal_lean_rate_.valid(now_sec) ||
            !capture_world_valid_)
            return;
        const bool anchor_on = capture_creep_anchor_ready() && capture_world_valid_;
        if (anchor_on != capture_creep_anchor_latched_)
        {
            capture_creep_anchor_latched_ = anchor_on;
            RCLCPP_INFO(this->get_logger(),
                        "[CAPTURE_ANCHOR-B v6.31] %s state=%d com_lean=%.3f com_rate=%.3f r=%.4f "
                        "v_com=%.3f v_ref=%.3f",
                        anchor_on ? "ON" : "OFF", static_cast<int>(current_state_),
                        ground_balance_angle(), ground_balance_rate(),
                        centroidal_balance_.forward, capture_com_velocity_, capture_v_ref_);
        }
        capture_anchor_active_ = anchor_on;
        double step_dt = dt;
        if (!std::isfinite(step_dt) || step_dt <= 0.0 || step_dt > 0.05)
            step_dt = 0.010;
        if (anchor_on)
        {
            capture_v_ref_ = bbot_jump::clamp_value(
                capture_v_ref_ - 1.0 * capture_com_velocity_ * step_dt, -0.60, 0.60);
        }
        else
        {
            const double release = 1.0 * step_dt;
            capture_v_ref_ += bbot_jump::clamp_value(-capture_v_ref_, -release, release);
        }
    }

    double ground_balance_angle() const
    {
        return centroidal_balance_.valid ? centroidal_balance_.angle : pitch_ - balance_offset_;
    }
    double ground_balance_rate() const
    {
        const double now_sec = this->now().seconds();
        if (centroidal_lean_rate_.valid(now_sec)) return centroidal_lean_rate_.rate();
        return centroidal_balance_.valid ? centroidal_balance_.rate : pitch_rate_;
    }
    double ground_balance_height() const
    {
        return centroidal_balance_.valid ? bbot_jump::clamp_value(centroidal_balance_.height, 0.15, 0.55) : bbot_jump::clamp_value(current_z_, 0.30, 0.50);
    }

    bool jump_ready_for_command() const
    {
        const double now_sec = this->now().seconds();
        const bool sensors = odom_received_ && takeoff_odom_pose_valid_ &&
            now_sec - last_world_odom_time_ <= 0.10 && centroidal_velocity_valid_ &&
            capture_world_valid_ && centroidal_lean_rate_.valid(now_sec) &&
            centroidal_balance_.valid;
        const bool quiet = std::abs(pitch_ - balance_offset_) < 0.04 &&
            std::abs(pitch_rate_) < 0.15 && std::abs(x_dot_) < 0.08 &&
            std::abs(ground_balance_angle()) < 0.10 &&
            std::abs(ground_balance_rate()) < 0.35 &&
            std::abs(capture_com_velocity_) < 0.08;
        return current_state_ == bbot_jump::STATE_BALANCE && sensors && quiet &&
            (!post_landing_effort_support_ || post_jump_balance_stable_timer_ >= 0.50);
    }

    // POST_BRAKE_HOLD 与 RECOVERY 共用的低速轮毂控制律。
    // 共用同一实现是状态切换连续性的硬约束，避免后续调参只改一侧。
    double compute_post_brake_hold_wheel_target(
        double &p_term, double &d_term,
        double &capture_state, double &capture_guard)
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

        wheel_pitch_err_diag_ = pitch_err;
        wheel_balance_rate_diag_ = balance_rate;
        wheel_k_theta_diag_ = current_gain_.k_theta;
        wheel_k_theta_dot_diag_ = current_gain_.k_theta_dot;
        wheel_control_height_diag_ = current_height_;
        wheel_p_term_diag_ = p_term;
        wheel_d_term_diag_ = d_term;
        wheel_attitude_cmd_diag_ = attitude_cmd;
        wheel_vel_error_diag_ = velocity_error;
        wheel_vel_term_diag_ = 0.85 * velocity_error;
        wheel_raw_pos_err_diag_ = raw_position_error;
        wheel_pos_error_diag_ = position_error;
        wheel_pos_term_diag_ = 0.35 * position_error;
        wheel_cmd_raw_diag_ = cmd_target;

        // V17/V19：微捕获必须前后对称，但不能把“当前速度+余量”作为
        // 常驻轮速目标。那会把姿态传感器的小振荡整流成持续行走；capture
        // 只用于平滑放宽对应方向的饱和上限，真正失稳仍由 CATCH 处理。
        capture_guard = 0.0;
        double max_backward_cmd = 0.75;
        double min_forward_cmd = -0.75;
        if (pitch_err < 0.0 && capture_state < -0.030)
        {
            capture_guard = bbot_jump::clamp_value(
                -capture_state - 0.030, 0.0, 0.20);
            max_backward_cmd = bbot_jump::clamp_value(
                0.75 + 1.75 * capture_guard, 0.75, 1.10);
        }
        else if (pitch_err > 0.0 && capture_state > 0.030)
        {
            capture_guard = bbot_jump::clamp_value(
                capture_state - 0.030, 0.0, 0.20);
            min_forward_cmd = -bbot_jump::clamp_value(
                0.75 + 1.75 * capture_guard, 0.75, 1.10);
        }

        // 超过回中范围后，只有明确进入倒下相平面才允许继续远离目标点。
        if (raw_position_error > 0.75 && capture_state < 0.12)
        {
            cmd_target = std::max(cmd_target, 0.0);
        }
        else if (raw_position_error < -0.75 && capture_state > -0.12)
        {
            cmd_target = std::min(cmd_target, 0.0);
        }
        wheel_cmd_pre_clamp_diag_ = cmd_target;
        wheel_min_fwd_cmd_diag_ = min_forward_cmd;
        wheel_max_bwd_cmd_diag_ = max_backward_cmd;
        const double final_cmd_target = bbot_jump::clamp_value(
            cmd_target, min_forward_cmd, max_backward_cmd);
        wheel_cmd_target_diag_ = final_cmd_target;
        // Keep the measured-COM capture/stop law after CATCH releases. The
        // velocity-servo hold law above has an unstable gravity mode even at
        // larger attitude gains; braking axle velocity is not COM capture.
        // Reuse the same estimator, shank compensation and bias reference in
        // HOLD, RECOVERY and post-landing BALANCE to avoid another handoff.
        if (capture_world_valid_)
        {
            const double capture_target = landing_capture_target(final_cmd_target);
            const double shank_rate = 0.5 * (hip_vel_left_ + knee_vel_left_ +
                                             hip_vel_right_ + knee_vel_right_) -
                                      torso_imu_.rate();
            // The LQR terms above are fallback-only when capture is active.
            // Log the applied COM and shank terms, rather than presenting the
            // unused braking terms as contributions to this wheel command.
            p_term = -capture_omega * centroidal_balance_.forward;
            d_term = -kinematics_.get_params().wheel_radius * shank_rate;
            wheel_k_theta_diag_ = 0.0;
            wheel_k_theta_dot_diag_ = 0.0;
            wheel_p_term_diag_ = p_term;
            wheel_d_term_diag_ = d_term;
            wheel_attitude_cmd_diag_ = p_term + d_term;
            wheel_vel_error_diag_ = capture_com_velocity_;
            wheel_vel_term_diag_ = -1.25 * capture_com_velocity_;
            wheel_pos_error_diag_ = 0.0;
            wheel_pos_term_diag_ = 0.0;
            wheel_cmd_raw_diag_ = capture_target;
            wheel_cmd_pre_clamp_diag_ = capture_target;
            wheel_min_fwd_cmd_diag_ = -5.0;
            wheel_max_bwd_cmd_diag_ = 5.0;
            wheel_cmd_target_diag_ = capture_target;
            return capture_target;
        }
        return final_cmd_target;
    }

    // ── 阶段 4：触地缓冲阻抗控制 (TOUCHDOWN_BUFFER) ──
    void run_state_touchdown_buffer(double now_sec, double dt)
    {
        if (!effort_mode_active_)
        {
            request_effort_controller();
            bbot_kinematics::IKSolution ik_hold = kinematics_.inverse_kinematics(L_TOUCH_, 0.0);
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);
            return;
        }
        if (!touchdown_buffer_initialized_)
        {
            state_start_time_ = now_sec;
            touchdown_buffer_initialized_ = true;
            touchdown_stable_count_ = 0;
            // 触地第一帧直接按下落速度预充阻尼支撑力；从静态重力起步会
            // 让高速下落的腿先压缩一段行程才开始承重。
            const double mass_per_leg = TOTAL_MASS_ * 0.5;
            // 缓冲高度参考的起点。折叠式落地构型下机身触地时只有约 0.26 m，
            // 直接以展腿参考 L_TOUCH_=0.50 起步会让第一帧请求 197 N/腿
            // （体重仅 86 N/腿），实测把机身以 +1.0~1.2 m/s 重新弹离地面，
            // 二次落地必然后倒。锚点取"触地实测高度"并夹在沉降带内。
            touchdown_height_ = current_z_;
            const auto buffer_profile = bbot_jump::landing_buffer_profile(
                touchdown_height_, L_BUFFER_SETTLE_, L_STAND_);
            touchdown_buffer_target_height_ = buffer_profile.target_height;
            touchdown_buffer_settle_duration_ = buffer_profile.duration;
            const double buffer_anchor_height = bbot_jump::clamp_value(
                touchdown_height_, touchdown_buffer_target_height_, L_TOUCH_);
            const double preload_force = mass_per_leg * 9.81 +
                                         K_Z_BUFFER_ * (buffer_anchor_height - current_z_) -
                                         D_Z_BUFFER_ * current_z_dot_;
            const double min_force = 0.25 * mass_per_leg * 9.81;
            buffer_force_per_leg_ = bbot_jump::clamp_value(
                preload_force, min_force, F_Z_BUFFER_MAX_);
            touchdown_catch_active_ = true;
            touchdown_reverse_brake_active_ = false;
            touchdown_reverse_brake_consumed_ = false;
            touchdown_catch_stable_count_ = 0;
            touchdown_catch_stable_time_ = 0.0;
            touchdown_settle_start_time_ = -1.0;
            touchdown_brake_active_ = false;
            touchdown_brake_ready_count_ = 0;
            touchdown_brake_start_time_ = -1.0;
            touchdown_brake_cmd_ref_ = 0.0;
            if (touchdown_joint_handoff_start_time_ < 0.0)
            {
                touchdown_joint_handoff_start_time_ = now_sec;
                touchdown_hip_left_start_ = hip_pos_left_;
                touchdown_knee_left_start_ = knee_pos_left_;
                touchdown_hip_right_start_ = hip_pos_right_;
                touchdown_knee_right_start_ = knee_pos_right_;
            }
            // 不清零 last_wheel_cmd_x_：从空中姿态控制连续接管到触地捕获。
        }
        double elapsed = now_sec - state_start_time_;

        // +fall-guard-v6.27：缓冲段一旦俯仰越出可恢复范围就切断轮驱并退出状态机。
        // 实测 B 型后倒是单调过程：站住样本触地后 1.5s 内 max|pitch| 只有
        // 0.21~0.42 rad，摔倒样本全部越过 0.60（越过时刻 0.89~1.47s），此后姿态
        // 不再回来，而捕获律继续以 12~23 rad/s 驱动轮子把整机拖行 15~18m
        // （v626_roll_t2：x 从 +3.39 退到 -14.65，并在 TOUCHDOWN_BUFFER 停驻 14.7s）。
        // 此时的追速需求已无解，轮驱只剩"把机器人往回拽"这一个作用。
        // 0.60 rad / 0.15 s 为试验/调参值：夹在上面两组实测值之间。
        constexpr double k_landing_fall_pitch = 0.60;
        constexpr double k_landing_fall_hold = 0.15;
        if (bbot_jump::landing_pitch_unrecoverable(
                pitch_, pitch_rate_, k_landing_fall_pitch, 0.90))
        {
            if (landing_fall_start_time_ < 0.0)
                landing_fall_start_time_ = now_sec;
        }
        else
        {
            landing_fall_start_time_ = -1.0;
        }
        if (landing_fall_start_time_ >= 0.0 &&
            now_sec - landing_fall_start_time_ >= k_landing_fall_hold)
        {
            RCLCPP_WARN(this->get_logger(),
                        "[落地终止] 触地后 %.2fs 俯仰 %.3f rad 越过 %.2f 且继续外倒 %.2fs，"
                        "判定不可恢复：轮速清零并退出跳跃状态机 (x=%.2f, vx=%.2f, wl=%.1f)",
                        elapsed, pitch_, k_landing_fall_pitch,
                        now_sec - landing_fall_start_time_, x_, x_dot_, left_wheel_vel_);
            jump_failure_reason_ = "landing_unrecoverable_pitch";
            write_jump_summary(false, jump_failure_reason_);
            publish_wheel_cmd(0.0, 0.0);
            current_state_ = bbot_jump::STATE_EMERGENCY;
            state_start_time_ = now_sec;
            return;
        }

        // 1. 任务空间阻抗计算
        // 初触地保留展腿高度来吸收冲击。长腿 wheel-first 构型只执行
        // 有限压缩，且按行程限制目标高度的变化率；旧 0.18 s 直接压到
        // 0.34 m 会让髋、膝目标各翻转超过 1 rad，并把 COM 甩到轮轴后方。
        const double settle_ratio = bbot_jump::clamp_value(
            elapsed / touchdown_buffer_settle_duration_, 0.0, 1.0);
        const double smooth_settle = settle_ratio * settle_ratio * (3.0 - 2.0 * settle_ratio);
        const double buffer_height_target = bbot_jump::lerp(
            bbot_jump::clamp_value(
                touchdown_height_, touchdown_buffer_target_height_, L_TOUCH_),
            touchdown_buffer_target_height_, smooth_settle);
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
        const double ik_z_buf = bbot_jump::clamp_value(
            buffer_height_target, touchdown_buffer_target_height_, L_TOUCH_);
        // 触地后的最初缓冲阶段继续保持腿在世界系近似竖直，防止一旦
        // 存在俯仰误差，IK 又把膝盖向地面方向折回去。
        const double buffer_pitch_comp = bbot_jump::clamp_value(
            pitch_err, -0.15, 0.15);
        bbot_kinematics::IKSolution ik_buf =
            kinematics_.inverse_kinematics(ik_z_buf, buffer_pitch_comp);
        // 触地瞬间保持空中末帧构型，随后再把 IK 参考平滑交接给缓冲控制。
        // 位置目标和速度目标都连续，避免髋关节为追赶新的 IK 反向猛甩。
        const double handoff_elapsed = std::max(
            0.0, now_sec - touchdown_joint_handoff_start_time_);
        const double active_handoff_duration = std::max(
            landing_joint_handoff_duration_, touchdown_buffer_settle_duration_);
        const double handoff_ratio = bbot_jump::clamp_value(
            handoff_elapsed / active_handoff_duration, 0.0, 1.0);
        const double handoff_smooth = handoff_ratio * handoff_ratio *
                                      (3.0 - 2.0 * handoff_ratio);
        const double handoff_smooth_dot =
            (handoff_ratio > 0.0 && handoff_ratio < 1.0) ? 6.0 * handoff_ratio * (1.0 - handoff_ratio) /
                                                               active_handoff_duration
                                                         : 0.0;
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
        double tau_body_per_hip = bbot_jump::touchdown_torso_pitch_torque(
            pitch_err, pitch_rate_, K_BODY_P_BUFFER_, K_BODY_D_BUFFER_, TAU_HIP_BODY_MAX_);
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
                                            balance_angle, balance_rate, pitch_capture_state))
        {
            touchdown_catch_active_ = true;
            touchdown_reverse_brake_active_ = false;
            touchdown_catch_stable_count_ = 0;
            touchdown_catch_stable_time_ = 0.0;
            touchdown_brake_active_ = false;
            touchdown_brake_ready_count_ = 0;
            touchdown_brake_start_time_ = -1.0;
            touchdown_brake_cmd_ref_ = 0.0;
            touchdown_settle_start_time_ = -1.0;
            RCLCPP_WARN(this->get_logger(),
                        "[RECAPTURE] 重心再次发散，返回CATCH com_lean=%.3f com_rate=%.3f capture=%.3f vx=%.3f",
                        balance_angle, balance_rate, pitch_capture_state, x_dot_);
        }
        const bool aligned_capture_fresh = centroidal_balance_.valid &&
                                           centroidal_lean_rate_.valid(now_sec);
        const bool catch_release_candidate = aligned_capture_fresh && bbot_jump::touchdown_release_ready(
                                                 balance_angle, balance_rate, pitch_capture_state, x_dot_, last_wheel_cmd_x_) &&
                                             std::abs(pitch_err) < 0.10 && std::abs(pitch_rate_) < 0.35;

        if (touchdown_catch_active_)
        {
            touchdown_catch_stable_count_ = catch_release_candidate ? std::min(touchdown_catch_stable_count_ + 1, 1000) : 0;
            touchdown_catch_stable_time_ = catch_release_candidate ? touchdown_catch_stable_time_ + dt : 0.0;
            if (touchdown_catch_stable_time_ >= 0.12)
            {
                touchdown_catch_active_ = false;
                touchdown_reverse_brake_active_ = false;
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
        }
        else if (!touchdown_brake_active_)
        {
            const double prepare_elapsed =
                (touchdown_settle_start_time_ >= 0.0) ? (now_sec - touchdown_settle_start_time_) : 0.0;

            const bool brake_ready_nominal =
                aligned_capture_fresh &&
                prepare_elapsed >= 0.08 &&
                pitch_capture_state > 0.08 &&
                balance_angle > 0.015 &&
                balance_rate > 0.05;

            const bool brake_ready_urgent =
                aligned_capture_fresh &&
                prepare_elapsed >= 0.06 &&
                pitch_capture_state > 0.12 &&
                balance_angle > 0.08 &&
                balance_rate > 0.30;

            const bool brake_ready_low_speed =
                aligned_capture_fresh &&
                prepare_elapsed > 0.25 &&
                std::abs(x_dot_) < 0.35 &&
                std::abs(balance_angle) < 0.06 &&
                std::abs(balance_rate) < 0.45;

            const bool brake_ready_counted =
                brake_ready_nominal || brake_ready_low_speed;
            touchdown_brake_ready_count_ = brake_ready_counted ? std::min(touchdown_brake_ready_count_ + 1, 1000) : 0;
            if (brake_ready_urgent || touchdown_brake_ready_count_ >= 2)
            {
                touchdown_brake_active_ = true;
                touchdown_brake_ready_count_ = 0;
                touchdown_brake_start_time_ = now_sec;
                touchdown_brake_cmd_ref_ = bbot_jump::clamp_value(
                    std::max(0.0, -x_dot_), 0.0, 1.2);
                RCLCPP_INFO(this->get_logger(),
                            "[落地捕获] PREPARE_BRAKE -> BRAKE (com_lean=%.3f, com_rate=%.3f, capture=%.3f, wheel_v=%.3f, brake_ref=%.3f)",
                            balance_angle, balance_rate, pitch_capture_state, x_dot_, touchdown_brake_cmd_ref_);
            }
        }
        else
        {
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
        const char *wheel_phase_name = "CATCH";

        if (touchdown_catch_active_)
        {
            wheel_phase_name = "CATCH";
            // 保留既有增益和命令变化率，仅将反馈改为重心倾角及其变化率。
            const double catch_p_term = 0.55 * current_gain_.k_theta * touchdown_catch_pitch_err;
            double catch_d_term = 0.35 * current_gain_.k_theta_dot * balance_rate;
            if (std::abs(touchdown_catch_pitch_err) > 0.08 && catch_p_term * catch_d_term < 0.0)
            {
                const double max_opposing_d = 0.45 * std::abs(catch_p_term);
                catch_d_term = bbot_jump::clamp_value(catch_d_term, -max_opposing_d, max_opposing_d);
            }
            wheel_pitch_err_diag = touchdown_catch_pitch_err;
            wheel_p_term_diag = catch_p_term;
            wheel_d_term_diag = catch_d_term;
            // 统一轮速控制律：包含速度死区
            // 与临近触地时完全相同的捕获律；刚接触时轮速尚未代表机身平移。
            const double contact_speed_blend = bbot_jump::clamp_value(elapsed / 0.08, 0.0, 1.0);
            const double catch_forward_velocity = bbot_jump::lerp(
                predicted_landing_forward_velocity(now_sec), x_dot_, contact_speed_blend);
            cmd_target = bbot_jump::catch_wheel_target(
                touchdown_catch_pitch_err, balance_rate, catch_forward_velocity,
                current_gain_.k_theta, current_gain_.k_theta_dot, cmd_scale_,
                bbot_jump::touchdown_catch_limit(touchdown_catch_pitch_err, balance_rate, capture_height));
            cmd_target = landing_capture_target(cmd_target);
            cmd_accel_limit = 8.0;

            if (!touchdown_reverse_brake_active_ &&
                !touchdown_reverse_brake_consumed_ && capture_world_valid_ &&
                bbot_jump::touchdown_reverse_brake_ready(
                    capture_com_velocity_, x_dot_, pitch_capture_state, balance_angle, balance_rate))
            {
                touchdown_reverse_brake_active_ = true;
                touchdown_reverse_brake_consumed_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "[TOUCHDOWN_REVERSE_BRAKE] support crossed zero while COM recovered "
                            "(v_com=%.3f xdot=%.3f capture=%.3f angle=%.3f rate=%.3f)",
                            capture_com_velocity_, x_dot_, pitch_capture_state, balance_angle, balance_rate);
            }
            // If the torso clearly resumes falling backward, hand authority
            // straight back to the full centroidal capture law.
            const double com_obs_age = now_sec - centroidal_world_.stamp();
            const bool capture_diverged = bbot_jump::reverse_brake_capture_diverged(
                capture_world_valid_, com_obs_age, pitch_capture_state, -0.16, 0.080);
            if (touchdown_reverse_brake_active_ &&
                (bbot_jump::landing_pitch_unrecoverable(pitch_, pitch_rate_, 0.22, 0.65) ||
                 capture_diverged))
            {
                touchdown_reverse_brake_active_ = false;
                RCLCPP_WARN(this->get_logger(),
                            "[TOUCHDOWN_RECAPTURE] reverse brake released (%s) "
                            "(pitch=%.3f rate=%.3f capture=%.3f age=%.3fs)",
                            capture_diverged ? "capture_diverged" : "diverging_torso",
                            pitch_, pitch_rate_, pitch_capture_state, com_obs_age);
            }
            if (touchdown_reverse_brake_active_)
            {
                wheel_phase_name = "REVERSE_BRAKE";
                // Positive command produces negative support velocity, so a
                // command with the same sign as xdot is viscous braking here.
                cmd_target = bbot_jump::clamp_value(
                    0.90 * bbot_jump::deadband(x_dot_, 0.03), -0.80, 0.80);
                cmd_accel_limit = 12.0;
            }
        }
        else if (!touchdown_brake_active_)
        {
            wheel_phase_name = "PREPARE";
            phase_elapsed_diag =
                (touchdown_settle_start_time_ >= 0.0) ? (now_sec - touchdown_settle_start_time_) : 0.0;

            const double prepare_pitch_ref = 0.035;
            const double prepare_pitch_err = balance_angle - prepare_pitch_ref;

            double prepare_p_term = 0.45 * current_gain_.k_theta * prepare_pitch_err;
            double prepare_d_term = 0.20 * current_gain_.k_theta_dot * balance_rate;
            if (prepare_p_term * prepare_d_term < 0.0)
            {
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
            cmd_target = landing_capture_target(cmd_target);
            cmd_accel_limit = 8.0;
        }
        else
        {
            wheel_phase_name = "BRAKE";
            phase_elapsed_diag =
                (touchdown_brake_start_time_ >= 0.0) ? (now_sec - touchdown_brake_start_time_) : 0.0;

            const double brake_ref_ramp =
                (phase_elapsed_diag < 0.25) ? 1.00 : 1.40;
            touchdown_brake_cmd_ref_ = std::max(
                0.0,
                touchdown_brake_cmd_ref_ -
                    brake_ref_ramp * std::max(dt, 0.001));

            const bool post_brake_hold =
                touchdown_brake_cmd_ref_ < 0.06 &&
                balance_rate > -0.20 &&
                (pitch_capture_state > -0.06 || std::abs(balance_angle) < 0.05);

            if (post_brake_hold)
            {
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
                                   std::abs(hold_capture_state) < 0.12)
                                      ? 4.0
                                      : 7.5;

                wheel_pitch_err_diag = hold_pitch_err;
                wheel_p_term_diag = hold_p_term;
                wheel_d_term_diag = hold_d_term;
                (void)hold_capture_state;
                phase_aux_diag = touchdown_brake_cmd_ref_;
                phase_guard_diag = hold_capture_guard;
            }
            else
            {
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
                cmd_target = landing_capture_target(cmd_target);
                cmd_accel_limit = 12.0;
            }
        }

        if (capture_world_active_ && !touchdown_reverse_brake_active_)
            cmd_accel_limit = 8.0;
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
        // V6.32：V15 的 |capture|<0.030 是地形参数而非稳定判据。capture 的零点是
        // 重心—轮轴连线相对重力线的夹角，稳态值随支撑面整体平移（实测 5° 坡 -0.021、
        // 低高度坡 -0.028、平地 -0.0381），平地因此永远交不了权（v631bflatC/D 6/6
        // 卡满 19.7 s，而其余六项门限都已 ≥93% 通过）。这里改用缓冲段自己宣布捕获完成
        // 的同一判据（0.12/0.10/0.35，与 CATCH 释放、capture_creep_anchor_ready 一致）。
        // 129 条归档日志离线复算：与旧的绝对门限相比，坡道验收批次交权时刻不变
        // （1 kg 7/7 完全相同），平地 6/6 变为 1.80~2.22 s 交权；所有归档翻倒/蠕行
        // 样本（v611_bufanchor、v625_antitow、v627_fall、v628_sup、v628pl1kg、
        // v630flatB 蠕行）在新门限下仍然一项都不通过——挡住它们的是轮速与轮命令，
        // 从来不是这个姿态幅值。
        const bool capture_settled = bbot_jump::touchdown_capture_settled(
            balance_angle, balance_rate, pitch_capture_state);
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

        if (touchdown_stable_count_ >= 20 || safe_timeout_exit)
        {
            RCLCPP_INFO(
                this->get_logger(),
                ">>> 落地缓冲完成 (elapsed=%.3fs, safe_timeout=%d)，进入阶段 5：恢复自平衡 (RECOVERY)... <<<",
                elapsed, safe_timeout_exit ? 1 : 0);

            current_state_ = bbot_jump::STATE_RECOVERY;
            state_start_time_ = now_sec;
            recovery_x_ref_ = touchdown_x_ref_;
            target_x_ = touchdown_x_ref_;
            was_moving_ = false;
            recovery_stable_count_ = 0;
            post_landing_pitch_ref_ = balance_offset_;
            height_force_per_leg_ = buffer_force_per_leg_;
            height_force_initialized_ = true;
            target_height_ = L_STAND_;
            recovery_hold_height_ = touchdown_buffer_target_height_;
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
        }
        else if (elapsed >= 0.80)
        {
            const char *touchdown_phase = touchdown_catch_active_ ? "CATCH" : (!touchdown_brake_active_ ? "PREPARE" : (touchdown_brake_cmd_ref_ < 0.06 ? "HOLD" : "BRAKE"));
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
    void trigger_handoff_fallback(const std::string &reason)
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

        // 统一轮速控制律
        double recovery_p_term = 0.0;
        double recovery_d_term = 0.0;
        double recovery_capture_state = 0.0;
        double recovery_capture_guard = 0.0;
        const double cmd_target = compute_post_brake_hold_wheel_target(
            recovery_p_term, recovery_d_term,
            recovery_capture_state, recovery_capture_guard);
        const double max_cmd_accel = capture_world_active_ ? 8.0 : (std::abs(x_dot_) < 0.20 && std::abs(recovery_capture_state) < 0.12) ? 4.0
                                                                                                                                        : 7.5;
        const double max_cmd_step = max_cmd_accel * std::max(dt, 0.001);
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
                                               cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

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
            if (!effort_mode_active_)
            {
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
            if (crouch_reached)
            {
                recovery_subphase_ = bbot_jump::RECOVERY_FAIL_STABILIZE;
                fail_recovery_stable_timer_ = 0.0;
                recovery_hold_height_ = failed_thrust_crouch_height_;
                current_height_ = failed_thrust_crouch_height_;
                recovery_hip_reference_ = kinematics_.inverse_kinematics(
                                                         failed_thrust_crouch_height_, 0.0)
                                              .theta_hip;
                RCLCPP_INFO(this->get_logger(),
                            ">>> [FAIL_RECOVERY] 已缩腿到 %.3fm，先在低位稳定姿态再重新站起 <<<",
                            failed_thrust_crouch_height_);
            }
            break;
        }

        case bbot_jump::RECOVERY_FAIL_STABILIZE:
        {
            controller_mode_str_ = "EFFORT_FAIL_STABILIZE";
            if (!effort_mode_active_)
            {
                request_effort_controller();
            }
            current_height_ = failed_thrust_crouch_height_;
            recovery_follow_height_ik_ = true;
            recovery_hip_reference_ = kinematics_.inverse_kinematics(
                                                     failed_thrust_crouch_height_, 0.0)
                                          .theta_hip;
            support_force_total = publish_effort_height_control(
                failed_thrust_crouch_height_, 0.0,
                380.0, 90.0, 220.0,
                22.0, 4.5, 42.0, 7.0,
                true);

            // 这里只要求“可站起”，不要求达到最终 BALANCE 的极严稳态。
            const bool fail_quiet =
                centroidal_balance_.valid && centroidal_lean_rate_.valid(now_sec) &&
                std::abs(pitch_err) <= 0.18 &&
                std::abs(pitch_rate_) <= 0.80 &&
                std::abs(x_dot_) <= 0.25 &&
                std::abs(current_z_dot_) <= 0.25;
            if (fail_quiet)
            {
                fail_recovery_stable_timer_ += dt;
            }
            else
            {
                fail_recovery_stable_timer_ = 0.0;
            }

            if (fail_recovery_stable_timer_ >= failed_thrust_settle_duration_)
            {
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
            if (!effort_mode_active_)
            {
                request_effort_controller();
            }
            double des_z = L_STAND_, des_v = 0.0, des_acc = 0.0;
            quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);
            current_height_ = des_z;
            if (recovery_follow_height_ik_)
            {
                recovery_hip_reference_ = kinematics_.inverse_kinematics(
                                                         des_z, 0.0)
                                              .theta_hip;
            }

            support_force_total = publish_effort_height_control(
                des_z, des_v,
                K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                25.0, 3.5, 45.0, 6.0,
                true);

            if (quintic_traj_.is_finished(now_sec) && std::abs(current_z_ - L_STAND_) < 0.04)
            {
                RCLCPP_INFO(this->get_logger(),
                            ">>> [RECOVERY] EFFORT_RAISE 完成，进入 EFFORT_STABILIZE 稳态检测 <<<");
                recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_STABILIZE;
                recovery_hip_reference_ = kinematics_.inverse_kinematics(
                                                         L_STAND_, 0.0)
                                              .theta_hip;
                recovery_follow_height_ik_ = false;
                recovery_stable_timer_ = 0.0;
            }
            break;
        }

        case bbot_jump::RECOVERY_EFFORT_STABILIZE:
        {
            controller_mode_str_ = "EFFORT";
            if (!effort_mode_active_)
            {
                request_effort_controller();
            }
            current_height_ = L_STAND_;
            support_force_total = publish_effort_height_control(
                L_STAND_, 0.0,
                K_Z_BUFFER_, D_Z_BUFFER_, F_Z_BUFFER_MAX_,
                25.0, 3.5, 45.0, 6.0,
                true);

            const bool com_steady = centroidal_lean_rate_.valid(now_sec) &&
                bbot_jump::centroidal_hold_ready(
                capture_world_valid_, ground_balance_angle(), ground_balance_rate(),
                capture_com_velocity_);
            const bool steady = com_steady && std::abs(pitch_err) <= 0.04 &&
                                std::abs(pitch_rate_) <= 0.15 &&
                                std::abs(x_dot_) <= 0.08 &&
                                std::abs(current_z_dot_) <= 0.03;
            if (steady)
            {
                recovery_stable_timer_ += dt;
            }
            else
            {
                recovery_stable_timer_ = 0.0;
            }

            if (recovery_stable_timer_ >= 0.50)
            {
                if (auto_return_balance_ || !enable_position_handoff_ || position_handoff_suppressed_for_jump_)
                {
                    RCLCPP_INFO(this->get_logger(),
                                ">>> [RECOVERY] 连续静稳 0.50s，平滑切入 Effort BALANCE <<<");
                    current_state_ = bbot_jump::STATE_BALANCE;
                    balance_entry_time_ = now_sec;
                    target_x_ = x_; // 同步位置参考，消除跳后位置大偏差
                    target_speed_const_ = 0.0;
                    target_speed_smoothed_ = 0.0;
                    post_landing_balance_soft_start_ = true;
                    post_landing_effort_support_ = true;
                    post_jump_balance_stable_timer_ = 0.0;
                    return;
                }
                else
                {
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
            if (position_preload_timer_ >= 0.10)
            {
                if (request_position_controller(true))
                {
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
            if (effort_mode_active_)
            {
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
                now_sec - position_switch_request_time_ > 1.0)
            {
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

            if (max_joint_err > handoff_joint_error_limit_)
            {
                trigger_handoff_fallback("HOLD阶段关节跟踪误差过大: " + std::to_string(max_joint_err));
                return;
            }
            if (height_drop > handoff_height_drop_limit_ || std::abs(current_z_dot_) > handoff_z_dot_limit_)
            {
                trigger_handoff_fallback("HOLD阶段高度塌陷: drop=" + std::to_string(height_drop) + " vz=" + std::to_string(current_z_dot_));
                return;
            }
            if (pitch_abs_err > handoff_pitch_error_limit_)
            {
                trigger_handoff_fallback("HOLD阶段俯仰角超标: err=" + std::to_string(pitch_abs_err));
                return;
            }

            position_hold_timer_ += dt;
            if (position_hold_timer_ >= 0.25)
            {
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

            if (max_joint_err > handoff_joint_error_limit_)
            {
                trigger_handoff_fallback("RETURN阶段关节跟踪误差过大: " + std::to_string(max_joint_err));
                return;
            }
            if (height_drop > handoff_height_drop_limit_ || std::abs(current_z_dot_) > handoff_z_dot_limit_)
            {
                trigger_handoff_fallback("RETURN阶段高度塌陷: drop=" + std::to_string(height_drop) + " vz=" + std::to_string(current_z_dot_));
                return;
            }
            if (pitch_abs_err > handoff_pitch_error_limit_)
            {
                trigger_handoff_fallback("RETURN阶段俯仰角超标: err=" + std::to_string(pitch_abs_err));
                return;
            }

            position_return_timer_ += dt;
            if (position_return_timer_ >= 0.80 && traj_return_hip_l_.is_finished(now_sec))
            {
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
        if (effort_mode_active_)
        {
            publish_effort_leg_control(ik_stand.theta_hip, ik_stand.theta_knee, 0.0, 0.0,
                                       0.0, 0.0, 100.0, 5.0, 100.0, 5.0);
        }
        else
        {
            publish_position_leg_control(ik_stand.theta_hip, ik_stand.theta_knee);
        }

        double pitch_err = pitch_ - balance_offset_;
        if (std::abs(pitch_err) < 0.18 && std::abs(pitch_rate_) < 1.5)
        {
            current_state_ = bbot_jump::STATE_BALANCE;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[平衡控制器] 机身摆起成功，切入 LQR 自平衡！");
        }
        else
        {
            double standup_vel = (pitch_err < -0.15) ? 2.5 : ((pitch_err > 0.15) ? -2.5 : 0.0);
            publish_wheel_cmd(standup_vel, 0.0);
        }
    }

    // ── 辅助与发布函数 ──
    void abort_jump_to_recovery(double now_sec, const char *reason)
    {
        const bool grounded_thrust_abort =
            current_state_ == bbot_jump::STATE_THRUST && !last_wheels_airborne_;
        jump_failure_reason_ = reason ? reason : "thrust_abort";
        RCLCPP_ERROR(this->get_logger(),
                     "[跳跃保护] %s (z=%.3f m, grounded_thrust=%d)，中止推地并进入恢复",
                     reason, current_z_, grounded_thrust_abort ? 1 : 0);
        current_state_ = bbot_jump::STATE_RECOVERY;
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

        if (grounded_thrust_abort)
        {
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
                                                     safe_start_height, 0.0)
                                          .theta_hip;
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
        }
        else
        {
            // 非接地 THRUST 失败保留原恢复入口，但修正位置参考并初始化 hip target。
            const double safe_start_height = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, L_STAND_);
            recovery_hold_height_ = safe_start_height;
            current_height_ = safe_start_height;
            recovery_follow_height_ik_ = true;
            recovery_hip_reference_ = kinematics_.inverse_kinematics(
                                                     safe_start_height, 0.0)
                                          .theta_hip;
            recovery_subphase_ = bbot_jump::RECOVERY_EFFORT_RAISE;
            quintic_traj_.init(now_sec, 0.45, safe_start_height, 0.0, 0.0,
                               L_STAND_, 0.0, 0.0);
            target_height_ = L_STAND_;
        }

        write_jump_summary(false, jump_failure_reason_);
        // THRUST 已处于 effort 模式；恢复环继续用 effort，避免异步控制器往返切换。
    }

    void transition_to_protective_landing(double now_sec, const char *reason)
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
        flight_tuck_plan_check_ = -1;
        flight_extend_plan_check_ = -1;
        flight_round_trip_plan_ = {};
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
        double tau_body_per_hip = bbot_jump::touchdown_torso_pitch_torque(
            pitch_err, pitch_rate_, K_BODY_P_BUFFER_, K_BODY_D_BUFFER_, TAU_HIP_BODY_MAX_);
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
                                       double &jz_hip, double &jz_knee) const
    {
        const auto &params = kinematics_.get_params();
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
        bool stabilize_body,
        double q_dot_hip_des = 0.0, double q_dot_knee_des = 0.0)
    {
        const double mass_per_leg = TOTAL_MASS_ * 0.5;
        double force_target = mass_per_leg * 9.81 +
                              k_z * (z_des - current_z_) +
                              d_z * (z_dot_des - current_z_dot_);
        force_target = bbot_jump::clamp_value(force_target, 0.0, force_per_leg_max);

        if (!height_force_initialized_)
        {
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

        if (stabilize_body)
        {
            const bool use_landing_pitch_ref =
                (current_state_ == bbot_jump::STATE_RECOVERY) || post_landing_gyro_reduced_;
            const double pitch_ref = use_landing_pitch_ref ? post_landing_pitch_ref_ : balance_offset_;
            const double pitch_err = pitch_ - pitch_ref;
            // V11：TOUCHDOWN 阶段一直使用髋、膝共同承担 Jz^T F，V10 一进入
            // RECOVERY 却把髋部垂直支撑力矩瞬间清零，造成明显的负载重分配阶跃。
            // RECOVERY 保留髋部垂直支撑，只在其上叠加机身俯仰阻尼；其它调用场景
            // 仍保留原先的“膝主承重”策略，避免扩大改动范围。
            // SQUAT 需要髋、膝共同提供 J^T Fz 支撑。此前把髋部支撑清零，
            // 使下蹲后的腿只能靠膝关节和小 PD 承重，导致直接后倒。
            if (current_state_ != bbot_jump::STATE_RECOVERY &&
                current_state_ != bbot_jump::STATE_SQUAT)
            {
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
            (current_state_ == bbot_jump::STATE_RECOVERY && !recovery_follow_height_ik_) ? recovery_hip_reference_ : ik.theta_hip;
        publish_effort_leg_control_lr(hip_position_target, ik.theta_knee,
                                      q_dot_hip_des, q_dot_knee_des,
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

        if (!height_force_initialized_)
        {
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
                double &tauh, double &tauk)
        {
            const double norm = std::hypot(jh, jk);
            if (norm < 1e-5)
                return;
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

        if (num_ % 25 == 0)
        {
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
        if (current_height_ > target_height_)
        {
            current_height_ -= step;
            if (current_height_ < target_height_)
                current_height_ = target_height_;
        }
        else if (current_height_ < target_height_)
        {
            current_height_ += step;
            if (current_height_ > target_height_)
                current_height_ = target_height_;
        }
        current_height_ = bbot_jump::clamp_value(current_height_, L_MIN_, L_MAX_);
    }

    void record_command_publication(const char*kind,const std::vector<double>&leg,
                                    double linear,double angular,bool zero,
                                    int64_t publish_ns,int64_t publish_end_ns,
                                    int64_t wall_publish_ns,int64_t wall_publish_end_ns)
    {
        if(!command_publication_log_.is_open())return;
        command_publication_log_<<std::setprecision(17)<<++command_publication_id_<<','
            <<publish_ns<<','<<capture_control_ns_<<','<<current_state_<<','<<kind;
        for(int i=0;i<4;++i)command_publication_log_<<','<<(i<static_cast<int>(leg.size())?leg[i]:std::numeric_limits<double>::quiet_NaN());
        command_publication_log_<<','<<linear<<','<<angular<<','<<zero<<','<<publish_end_ns<<','<<wall_publish_ns
            <<','<<wall_publish_end_ns<<'\n';
        command_publication_log_.flush();
    }

    template<typename PublisherT, typename MessageT>
    void publish_recorded_command(const PublisherT &publisher, const MessageT &message,
                                  const char *kind, const std::vector<double> &leg,
                                  double linear, double angular, bool zero=false)
    {
        const auto wall_ns = []() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        };
        const int64_t publish_ns = this->now().nanoseconds();
        const int64_t wall_publish_ns = wall_ns();
        publisher->publish(message);
        const int64_t wall_publish_end_ns = wall_ns();
        const int64_t publish_end_ns = this->now().nanoseconds();
        last_command_publish_ns_ = publish_ns;
        last_command_publish_end_ns_ = publish_end_ns;
        last_command_wall_publish_ns_ = wall_publish_ns;
        last_command_wall_publish_end_ns_ = wall_publish_end_ns;
        record_command_publication(kind, leg, linear, angular, zero,
            publish_ns, publish_end_ns, wall_publish_ns, wall_publish_end_ns);
    }

    void publish_wheel_cmd(double linear_x, double angular_z, bool zero_wheel_effort = false)
    {
        last_wheel_cmd_x_ = linear_x;
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        // Atomic with this target; only the opt-in effort servo consumes the marker.
        // Default velocity-mode commands retain the normal base_link frame.
        cmd.header.frame_id = zero_wheel_effort ? "base_link__zero_wheel_effort" : "base_link";
        cmd.twist.linear.x = linear_x;
        cmd.twist.angular.z = angular_z;
        publish_recorded_command(cmd_pub_, cmd, "wheel", {}, linear_x, angular_z, zero_wheel_effort);
    }

    // ── 位置阶段：只向位置控制器发布构型命令 ──
    void publish_position_leg_control(double q_hip_des, double q_knee_des)
    {
        publish_position_leg_control_lr(q_hip_des, q_knee_des, q_hip_des, q_knee_des);
    }

    void publish_position_leg_control_lr(double q_hip_left, double q_knee_left,
                                         double q_hip_right, double q_knee_right)
    {
        if (!pos_cmd_init_)
        {
            // 首次发布时从实际测量关节位置平滑起步，杜绝开机阶跃“踹地”
            last_q_hip_cmd_left_ = hip_pos_left_;
            last_q_knee_cmd_left_ = knee_pos_left_;
            last_q_hip_cmd_right_ = hip_pos_right_;
            last_q_knee_cmd_right_ = knee_pos_right_;
            pos_cmd_init_ = true;
        }

        constexpr double kMaxJointSlewRate = 2.0; // rad/s 最大斜坡变化率
        constexpr double dt = 0.005;
        const double max_step = kMaxJointSlewRate * dt;

        hip_pos_cmd_left_ = last_q_hip_cmd_left_ + bbot_jump::clamp_value(
                                                       q_hip_left - last_q_hip_cmd_left_, -max_step, max_step);
        knee_pos_cmd_left_ = last_q_knee_cmd_left_ + bbot_jump::clamp_value(
                                                         q_knee_left - last_q_knee_cmd_left_, -max_step, max_step);
        hip_pos_cmd_right_ = last_q_hip_cmd_right_ + bbot_jump::clamp_value(
                                                         q_hip_right - last_q_hip_cmd_right_, -max_step, max_step);
        knee_pos_cmd_right_ = last_q_knee_cmd_right_ + bbot_jump::clamp_value(
                                                           q_knee_right - last_q_knee_cmd_right_, -max_step, max_step);

        last_q_hip_cmd_left_ = hip_pos_cmd_left_;
        last_q_knee_cmd_left_ = knee_pos_cmd_left_;
        last_q_hip_cmd_right_ = hip_pos_cmd_right_;
        last_q_knee_cmd_right_ = knee_pos_cmd_right_;

        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip_pos_cmd_left_, knee_pos_cmd_left_,
                        hip_pos_cmd_right_, knee_pos_cmd_right_};
        leg_pos_pub_->publish(leg_cmd);
    }

    bbot_jump::JointEffortLimits current_effort_limits() const
    {
        return bbot_jump::jump_effort_limits(current_state_ == bbot_jump::STATE_THRUST,
                                             this->get_parameter("use_sim_time").as_bool(), sim_relax_thrust_limits_);
    }

    bool publish_allocated_effort(const std::array<double,4> &requested, double now_sec)
    {
        if ((current_state_ != bbot_jump::STATE_THRUST &&
             current_state_ != bbot_jump::STATE_FLIGHT) || !effort_mode_active_ ||
            leg_mode_switch_pending_) return false;
        std_msgs::msg::Float64MultiArray message;
        message.data.resize(4);
        for (int i=0;i<4;++i) {
            if (!std::isfinite(requested[i])) return false;
            const auto limits=current_effort_limits();
            const double cap=i%2?limits.knee:limits.hip;
            message.data[i] = std::clamp(requested[i], -cap, cap);
        }
        hip_cmd_left_=message.data[0]; knee_cmd_left_=message.data[1];
        hip_cmd_right_=message.data[2]; knee_cmd_right_=message.data[3];
        actual_tau_hip_left_=hip_cmd_left_; actual_tau_knee_left_=knee_cmd_left_;
        actual_tau_hip_right_=hip_cmd_right_; actual_tau_knee_right_=knee_cmd_right_;
        last_effort_tau_hip_left_=hip_cmd_left_; last_effort_tau_knee_left_=knee_cmd_left_;
        last_effort_tau_hip_right_=hip_cmd_right_; last_effort_tau_knee_right_=knee_cmd_right_;
        last_effort_time_=now_sec; effort_slew_initialized_=true;
        publish_recorded_command(leg_effort_pub_, message, "leg", message.data, NAN, NAN);
        return true;
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
        double tau_att_hip_l = 0.0, double tau_att_hip_r = 0.0,
        const bbot_jump::JointVector & arrest_dynamics_ff = bbot_jump::JointVector::Zero())
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

        auto compute_hip = [&](double tau_ff, double tau_att, double q_des, double q_dot_des, double q_act, double q_vel)
        {
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

        bbot_jump::JointVector flight_pd_before_arrest_ff = bbot_jump::JointVector::Zero();
        bool arrest_ff_entered_pd = false;
        if (current_state_ == bbot_jump::STATE_FLIGHT)
        {
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
            for (int i = 0; i < 4; ++i)
            {
                air_pd_raw_[i] = raw[i];
                air_pd_discrete_[i] = tau[i];
            }
            tau_hip_left = tau[0];
            tau_knee_left = tau[1];
            tau_hip_right = tau[2];
            tau_knee_right = tau[3];
            flight_pd_before_arrest_ff = tau;
            if (flight_subphase_ == bbot_jump::FLIGHT_SUBPHASE_ATTITUDE_ARREST &&
                arrest_dynamics_feedforward_enable_ && effort_mode_active_ &&
                !leg_mode_switch_pending_ &&
                arrest_dynamics_feedforward_guard_diag_ ==
                    static_cast<int>(bbot_jump::ArrestDynamicsGuard::Active) &&
                arrest_dynamics_ff.allFinite())
            {
                tau_hip_left += arrest_dynamics_ff[0];
                tau_knee_left += arrest_dynamics_ff[1];
                tau_hip_right += arrest_dynamics_ff[2];
                tau_knee_right += arrest_dynamics_ff[3];
                arrest_ff_entered_pd = true;
            }
        }

        // v6.9：箱体姿态与腿部任务分配不同自由度。承重前馈只限速，
        // 关节阻尼保持离散求解；髋共同力矩不再进入自由腿惯量求解后被抵消。
        const bool ground_feedback =
            current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER ||
            current_state_ == bbot_jump::STATE_RECOVERY ||
            (current_state_ == bbot_jump::STATE_BALANCE && post_landing_effort_support_) ||
            ((current_state_ == bbot_jump::STATE_PRE_JUMP ||
              current_state_ == bbot_jump::STATE_SQUAT) && pre_jump_effort_capture_);
        if (ground_feedback)
        {
            const bbot_jump::JointVector q(hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_);
            const bbot_jump::JointVector v(hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_);
            const bbot_jump::JointVector qd(q_hip_des_l, q_knee_des_l, q_hip_des_r, q_knee_des_r);
            const bbot_jump::JointVector vd(q_dot_hip_des_l, q_dot_knee_des_l, q_dot_hip_des_r, q_dot_knee_des_r);
            const bbot_jump::JointVector kp(kp_hip, kp_knee, kp_hip, kp_knee);
            const bbot_jump::JointVector kd(kd_hip, kd_knee, kd_hip, kd_knee);
            const auto gravity = bbot_jump::leg_gravity_torques(
                {hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_}, pitch_, body_mass_);
            // 支撑前馈必须包含腿/轮自身重力。只用 J^T F 会漏掉约数Nm的
            // 髋轴力矩，让已经离散衰减的箱体纠姿长期承担这一静态偏差。
            const bbot_jump::JointVector support = gravity + bbot_jump::JointVector(
                                                                 tau_ff_hip_l - tau_att_hip_l, tau_ff_knee_l,
                                                                 tau_ff_hip_r - tau_att_hip_r, tau_ff_knee_r);
            for (int i = 0; i < 4; ++i)
                ground_gravity_torque_[i] = gravity[i];
            const double sample_age = std::max(0.0, this->now().seconds() - joint_sample_time_);
            ground_pd_horizon_ = bbot_jump::clamp_value(
                2.0 * joint_sample_period_ + sample_age, 0.025, 0.060);
            const auto mass = bbot_jump::flight_joint_inertia(q, body_mass_);
            // The legacy ground inverse preserves only hip difference mode;
            // this opt-in probe needs the full four-joint implicit response
            // so symmetric hip references create common-mode hip torque too.
            // Static gravity/Jz support is added separately below exactly once.
            const auto feedback = ground_motion_law_active() ?
                bbot_jump::discrete_flight_pd(
                    mass, q, v, qd, vd, kp, kd,
                    bbot_jump::JointVector::Zero(), ground_pd_horizon_) :
                bbot_jump::discrete_ground_leg_feedback(
                    mass, q, v, qd, vd, kp, kd, ground_pd_horizon_);
            if (!ground_support_initialized_)
            {
                // 仅从上一前馈接入承重；不能把上一帧关节反馈当成前馈继续保持。
                ground_support_previous_ = last_support_feedforward_;
                ground_support_initialized_ = true;
            }
            const double effort_dt = bbot_jump::clamp_value(
                this->now().seconds() - last_effort_time_, 0.0, 0.020);
            const double step = (current_state_ == bbot_jump::STATE_RECOVERY ? 800.0 : 3000.0) * effort_dt;
            ground_support_previous_ += (support - ground_support_previous_).cwiseMax(-step).cwiseMin(step);
            last_support_feedforward_ = ground_support_previous_;
            const auto leg_torque = (ground_support_previous_ + feedback).eval();
            const double now_sec = this->now().seconds();
            torso_imu_fresh_ = torso_imu_.fresh(now_sec);
            torso_control_rate_ = torso_imu_fresh_ ? torso_imu_.rate() : pitch_rate_;
            const double imu_age = torso_imu_fresh_ ? std::max(0.0, now_sec - torso_imu_.stamp()) : sample_age;
            // 加上专用IMU滤波的10ms时间常数，不把200Hz定时器当传感器频率。
            torso_control_horizon_ = bbot_jump::clamp_value(
                2.0 * std::max(joint_sample_period_, torso_imu_.period()) +
                    std::max(sample_age, imu_age) + 0.010,
                0.030, 0.080);
            if (current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER ||
                current_state_ == bbot_jump::STATE_RECOVERY)
            {
                if (current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER &&
                    !touchdown_torso_convergence_latched_)
                {
                    const bool fresh_world_com = capture_world_valid_ &&
                        centroidal_world_.valid(now_sec) && centroidal_balance_.valid &&
                        centroidal_lean_rate_.valid(now_sec);
                    const double touchdown_elapsed = std::max(0.0, now_sec - state_start_time_);
                    if (bbot_jump::touchdown_torso_convergence_ready(
                            bbot_jump::touchdown_effort_support_active(
                                effort_mode_active_, leg_mode_switch_pending_),
                            touchdown_elapsed, fresh_world_com,
                            centroidal_balance_.angle, capture_com_velocity_,
                            pitch_ - balance_offset_))
                    {
                        touchdown_torso_convergence_latched_ = true;
                        touchdown_torso_convergence_start_time_ = now_sec;
                    }
                }
                const double pitch_err = pitch_ - balance_offset_;
                const double impact_torque = bbot_jump::touchdown_torso_pitch_torque(
                    pitch_err, torso_control_rate_, K_BODY_P_BUFFER_, K_BODY_D_BUFFER_, TAU_HIP_BODY_MAX_);
                const bbot_jump::TorsoPitchTorque impact{0.0, impact_torque, impact_torque};
                if (touchdown_torso_convergence_latched_)
                {
                    const double fy = torso_imu_fresh_ ? torso_imu_.fy() : -9.81 * std::sin(pitch_);
                    const double fz = torso_imu_fresh_ ? torso_imu_.fz() : 9.81 * std::cos(pitch_);
                    const auto balance = bbot_jump::torso_pitch_torque(
                        pitch_, balance_offset_, torso_control_rate_, fy, fz, body_mass_,
                        K_BODY_P_BUFFER_, K_BODY_D_BUFFER_, torso_control_horizon_, TAU_HIP_BODY_MAX_);
                    touchdown_torso_convergence_blend_ = bbot_jump::touchdown_torso_convergence_blend(
                        now_sec - touchdown_torso_convergence_start_time_);
                    torso_torque_ = bbot_jump::blend_torso_pitch_torque(
                        impact, balance, touchdown_torso_convergence_blend_);
                }
                else
                {
                    touchdown_torso_convergence_blend_ = 0.0;
                    torso_torque_ = impact;
                }
            }
            else
            {
                touchdown_torso_convergence_blend_ = 0.0;
                // 过期或无效加速度只退回静态重力补偿，不能继续用旧接触冲击。
                const bool groundMotion = ground_motion_law_active();
                const double fy = groundMotion ? 0.0 :
                    (torso_imu_fresh_ ? torso_imu_.fy() : -9.81 * std::sin(pitch_));
                const double fz = groundMotion ? 0.0 :
                    (torso_imu_fresh_ ? torso_imu_.fz() : 9.81 * std::cos(pitch_));
                const double torso_reference = pre_jump_effort_capture_ &&
                    (current_state_ == bbot_jump::STATE_PRE_JUMP || current_state_ == bbot_jump::STATE_SQUAT)
                    ? active_jump_pitch_ref_ :
                      groundMotion ? ground_motion_pitch_anchor_ : balance_offset_;
                const double torso_rate_reference = pre_jump_effort_capture_ &&
                    (current_state_ == bbot_jump::STATE_PRE_JUMP || current_state_ == bbot_jump::STATE_SQUAT)
                    ? active_jump_pitch_rate_ref_ : 0.0;
                torso_torque_ = bbot_jump::torso_pitch_torque(
                    pitch_, torso_reference, torso_control_rate_ - torso_rate_reference,
                    fy, fz, body_mass_,
                    K_BODY_P_BUFFER_, K_BODY_D_BUFFER_, torso_control_horizon_, TAU_HIP_BODY_MAX_);
            }
            double torso_request = ground_motion_law_active() ?
                torso_torque_.feedback : torso_torque_.command;
            last_tau_body_per_hip_ = torso_request;
            hip_common_before_allocation_ = 0.5 * (leg_torque[0] + leg_torque[2]);
            for (int i = 0; i < 4; ++i)
                ground_pd_feedback_[i] = feedback[i];
            if (ground_motion_law_active()) {
                bbot_jump::GroundMotionEffortInput direct_input;
                direct_input.support = {{leg_torque[0], leg_torque[1],
                                         leg_torque[2], leg_torque[3]}};
                direct_input.torso_per_hip = torso_request;
                direct_input.hip_limit = kHipTauLimit;
                direct_input.knee_limit = kKneeTauLimit;
                auto direct = bbot_jump::ground_motion_direct_full_support(direct_input);
                if (!direct.valid) {
                    fail_ground_motion("invalid_direct_support_effort");
                    direct.torque = {{actual_tau_hip_left_, actual_tau_knee_left_,
                                      actual_tau_hip_right_, actual_tau_knee_right_}};
                    for (double &tau : direct.torque)
                        if (!std::isfinite(tau)) tau = 0.0;
                }
                if (ground_motion_stage_ == "blending" ||
                    ground_motion_stage_ == "blend_settle") {
                    for (std::size_t i = 0; i < direct.torque.size(); ++i)
                        direct.torque[i] = (1.0 - ground_motion_blend_alpha_) *
                            ground_motion_baseline_effort_[i] +
                            ground_motion_blend_alpha_ * direct.torque[i];
                }
                tau_hip_left = direct.torque[0];
                tau_knee_left = direct.torque[1];
                tau_hip_right = direct.torque[2];
                tau_knee_right = direct.torque[3];
                ground_motion_torso_request_ = torso_request;
                ground_motion_static_feedforward_ = {{
                    ground_support_previous_[0], ground_support_previous_[1],
                    ground_support_previous_[2], ground_support_previous_[3]}};
                ground_motion_joint_feedback_ = {{
                    feedback[0], feedback[1], feedback[2], feedback[3]}};
                ground_motion_torso_static_ff_ = 0.0;
                hip_differential_torque_ = 0.5 *
                    (direct.torque[0] - direct.torque[2]);
            } else {
                const auto tau = bbot_jump::allocate_torso_hips(
                    leg_torque, torso_request, kHipTauLimit);
                hip_differential_torque_ = 0.5 * (tau[0] - tau[2]);
                tau_hip_left = tau[0];
                tau_knee_left = tau[1];
                tau_hip_right = tau[2];
                tau_knee_right = tau[3];
            }

            if (current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER ||
                current_state_ == bbot_jump::STATE_RECOVERY)
            {
                tau_hip_left = bbot_jump::touchdown_hip_soft_limit_guard(
                    hip_pos_left_, hip_vel_left_, tau_hip_left, TAU_HIP_BODY_MAX_, 1.20, 1.45, 0.025);
                tau_hip_right = bbot_jump::touchdown_hip_soft_limit_guard(
                    hip_pos_right_, hip_vel_right_, tau_hip_right, TAU_HIP_BODY_MAX_, 1.20, 1.45, 0.025);
            }
        }
        else
        {
            last_support_feedforward_ = bbot_jump::JointVector(
                tau_ff_hip_l - tau_att_hip_l, tau_ff_knee_l, tau_ff_hip_r - tau_att_hip_r, tau_ff_knee_r);
            ground_support_initialized_ = false;
            ground_pd_horizon_ = 0.0;
            ground_pd_feedback_.fill(0.0);
            ground_gravity_torque_.fill(0.0);
            torso_torque_ = {};
            torso_imu_fresh_ = false;
            hip_common_before_allocation_ = 0.0;
            hip_differential_torque_ = 0.0;
        }

        tau_hip_left = bbot_jump::clamp_value(tau_hip_left, -kHipTauLimit, kHipTauLimit);
        tau_knee_left = bbot_jump::clamp_value(tau_knee_left, -kKneeTauLimit, kKneeTauLimit);
        tau_hip_right = bbot_jump::clamp_value(tau_hip_right, -kHipTauLimit, kHipTauLimit);
        tau_knee_right = bbot_jump::clamp_value(tau_knee_right, -kKneeTauLimit, kKneeTauLimit);

        if (arrest_ff_entered_pd)
        {
            const bbot_jump::JointVector clipped_pd = flight_pd_before_arrest_ff.cwiseMax(
                bbot_jump::JointVector(-kHipTauLimit, -kKneeTauLimit,
                                       -kHipTauLimit, -kKneeTauLimit)).cwiseMin(
                bbot_jump::JointVector(kHipTauLimit, kKneeTauLimit,
                                       kHipTauLimit, kKneeTauLimit));
            const bbot_jump::JointVector final_tau(
                tau_hip_left, tau_knee_left, tau_hip_right, tau_knee_right);
            const auto applied = final_tau - clipped_pd;
            for (int i = 0; i < 4; ++i)
                arrest_dynamics_applied_diag_[i] = applied[i];
        }

        if (current_state_ == bbot_jump::STATE_TOUCHDOWN_BUFFER)
        {
            tau_knee_left = bbot_jump::landing_knee_extension_guard(
                knee_pos_left_, knee_vel_left_, tau_knee_left, kKneeTauLimit);
            tau_knee_right = bbot_jump::landing_knee_extension_guard(
                knee_pos_right_, knee_vel_right_, tau_knee_right, kKneeTauLimit);
        }

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
        publish_recorded_command(leg_effort_pub_, effort_cmd, "leg", effort_cmd.data, NAN, NAN);
        write_ground_motion_after_leg_publish();
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
        if (effort_mode_active_ || leg_mode_switch_pending_)
            return;
        if (!switch_ctrl_client_->service_is_ready())
        {
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
        effort_switch_request_stamp_ = this->now().seconds();
        effort_switch_ack_stamp_ = -1.0;
        effort_switch_result_ = 0;
        switch_ctrl_client_->async_send_request(request,
                                                [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
                                                {
                                                    leg_mode_switch_pending_ = false;
                                                    effort_switch_ack_stamp_ = this->now().seconds();
                                                    try
                                                    {
                                                        auto result = future.get();
                                                        effort_switch_result_ = result->ok ? 1 : 2;
                                                        if (result->ok)
                                                        {
                                                            effort_mode_active_ = true;
                                                            RCLCPP_INFO(this->get_logger(), "[控制器切换] >>> Position → Effort 切换成功！全力爆发模式已激活 <<<");
                                                        }
                                                        else
                                                        {
                                                            RCLCPP_WARN(this->get_logger(), "[控制器切换] Position → Effort 切换失败！");
                                                        }
                                                    }
                                                    catch (const std::exception &error)
                                                    {
                                                        effort_switch_result_ = 3;
                                                        RCLCPP_WARN(this->get_logger(), "[控制器切换] Position → Effort 响应异常: %s", error.what());
                                                    }
                                                });
        RCLCPP_INFO(this->get_logger(), "[控制器切换] 请求 Position → Effort ...");
    }

    // 请求切换为位置控制器 (调试/兼容)
    bool request_position_controller(bool preload_current_pose = false)
    {
        if (!effort_mode_active_ || leg_mode_switch_pending_)
            return false;
        if (!switch_ctrl_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[控制器切换] switch_controller 服务尚未就绪");
            return false;
        }
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        if (preload_current_pose)
        {
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
        }
        else
        {
            bbot_kinematics::IKSolution ik_stand = kinematics_.inverse_kinematics(current_height_, 0.0);
            publish_position_leg_control(ik_stand.theta_hip, ik_stand.theta_knee);
        }

        request->activate_controllers = {"leg_position_controller"};
        request->deactivate_controllers = {"leg_effort_controller"};
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        request->activate_asap = true;
        leg_mode_switch_pending_ = true;
        switch_ctrl_client_->async_send_request(request,
                                                [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
                                                {
                                                    auto result = future.get();
                                                    leg_mode_switch_pending_ = false;
                                                    if (result->ok)
                                                    {
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
                                                            recovery_subphase_ == bbot_jump::RECOVERY_SWITCHING)
                                                        {
                                                            recovery_subphase_ = bbot_jump::RECOVERY_POSITION_HOLD;
                                                            position_hold_timer_ = 0.0;
                                                            position_switch_request_time_ = -1.0;
                                                        }
                                                        else
                                                        {
                                                            // 超时回退后才到达的成功回调：不能让 Position 留在激活态。
                                                            RCLCPP_WARN(this->get_logger(),
                                                                        "[控制器切换] 收到过期的 Position 成功回调，立即恢复 Effort");
                                                            request_effort_controller();
                                                        }
                                                    }
                                                    else
                                                    {
                                                        RCLCPP_WARN(this->get_logger(), "[控制器切换] Effort → Position 切换失败！");
                                                        if (current_state_ == bbot_jump::STATE_RECOVERY &&
                                                            recovery_subphase_ == bbot_jump::RECOVERY_SWITCHING)
                                                        {
                                                            trigger_handoff_fallback("controller_manager拒绝Effort→Position切换");
                                                        }
                                                    }
                                                });
        RCLCPP_INFO(this->get_logger(), "[控制器切换] 请求 Effort → Position ...");
        return true;
    }

    void open_log_files()
    {
        std::string log_file = jump_log_path_.empty() ? (data_path_ + "jump_velocity_control_log.csv") : jump_log_path_;
        std::string summary_file = jump_summary_path_.empty() ? (data_path_ + "jump_velocity_summary.csv") : jump_summary_path_;
        const std::string expected_summary_header =
            "jump_id,jump_height_target,target_takeoff_velocity,takeoff_velocity,apex_height_delta,apex_world_z_delta,max_abs_pitch,"
            "takeoff_pitch_rate,landing_pitch_err,touchdown_drift_x,max_abs_hip_torque,max_abs_knee_torque,mechanical_work,"
            "recovery_completed,failed,reason,takeoff_com_z,takeoff_com_vz,takeoff_com_vx,apex_com_z_delta,"
            "tuck_entered,extend_entered,protective_reason,touchdown_time,recovery_time,balance_return_time,exit_code";

        // Preserve a historical/mismatched summary and use a correctly headed
        // sibling. Compare the complete field names, not a remembered count.
        auto header_matches = [](const std::string &path, const std::string &expected) {
            if (!std::filesystem::exists(path) || std::filesystem::file_size(path) == 0) return true;
            std::ifstream existing(path);
            std::string first_line;
            std::getline(existing, first_line);
            if (!first_line.empty() && first_line.back() == '\r') first_line.pop_back();
            return first_line == expected;
        };
        if (!header_matches(summary_file, expected_summary_header))
        {
            const std::string schema_base = summary_file + ".schema27.csv";
            summary_file = schema_base;
            for (int suffix = 1; !header_matches(summary_file, expected_summary_header); ++suffix)
                summary_file = schema_base + "." + std::to_string(suffix);
        }
        RCLCPP_INFO(this->get_logger(), "跳跃摘要写入: %s", summary_file.c_str());

        try {
            if (!log_file.empty()) {
                auto p = std::filesystem::path(log_file).parent_path();
                if (!p.empty()) std::filesystem::create_directories(p);
            }
            if (!summary_file.empty()) {
                auto p = std::filesystem::path(summary_file).parent_path();
                if (!p.empty()) std::filesystem::create_directories(p);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "创建日志目录异常: %s", e.what());
        }

        jump_log_file_.open(log_file);
        bool summary_needs_header = true;
        if (std::filesystem::exists(summary_file) && std::filesystem::file_size(summary_file) > 0) {
            summary_needs_header = false;
        }
        jump_summary_file_.open(summary_file, std::ios::out | std::ios::app);
        if (jump_summary_file_.is_open() && summary_needs_header)
        {
            jump_summary_file_ << expected_summary_header << "\n";
            jump_summary_file_.flush();
        }
        if (jump_log_file_.is_open())
        {
            jump_log_file_ << "timestamp,state,state_name,z,z_dot,gazebo_world_z,gazebo_world_z_dot,pitch,pitch_rate,acc_z,"
                           << "cmd_x,x,x_dot,touchdown_x_ref,hip_pos_left,knee_pos_left,hip_pos_right,knee_pos_right,"
                           << "hip_effort_left,knee_effort_left,hip_cmd_left,knee_cmd_left,"
                           << "hip_cmd_right,knee_cmd_right,"
                           << "actual_tau_hip_left,actual_tau_knee_left,actual_tau_hip_right,actual_tau_knee_right,"
                           << "hip_pos_cmd_left,knee_pos_cmd_left,hip_pos_cmd_right,knee_pos_cmd_right,"
                           << "tau_ff_hip,tau_ff_knee,F_z,F_z_request,F_z_limit,velocity_reached,wheels_airborne,airborne_confidence,target_takeoff_velocity,tau_body_hip,thrust_reaction_ff,"
                           << "flight_subphase,recovery_subphase,controller_mode,thrust_motion_elapsed,thrust_attitude_blocked,air_wheel_cmd_raw,left_wheel_vel,right_wheel_vel,attitude_arrest_stable_count,"
                           << "touchdown_phase,brake_ref,capture_state,target_x,x_error,cmd_target,hip_vel_left,knee_vel_left,hip_vel_right,knee_vel_right,hip_vel_cmd_left,knee_vel_cmd_left,hip_vel_cmd_right,knee_vel_cmd_right,wheel_clearance,contact_window,thrust_extension_scale,air_pd_horizon,air_pd_raw_hl,air_pd_raw_kl,air_pd_raw_hr,air_pd_raw_kr,air_pd_discrete_hl,air_pd_discrete_kl,air_pd_discrete_hr,air_pd_discrete_kr,jump_forward_speed,jump_takeoff_forward_speed,jump_pitch_ref,active_jump_pitch_ref,jump_takeoff_pitch_rate,active_jump_pitch_rate_ref,pre_jump_stable_timer,takeoff_forward_speed,air_wheel_baseline,gazebo_world_x_dot,gazebo_world_y_dot,landing_capture_vx,landing_capture_raw_offset,landing_capture_offset,landing_target_x,landing_capture_comp,landing_capture_omega,landing_forward_axis_x,landing_forward_axis_y,landing_capture_planned,flight_air_pitch_ref,flight_air_pitch_rate_ref,protective_deploy_duration,protective_deploy_rate_limited,takeoff_aligned,takeoff_sample_stamp,takeoff_clearance_left,takeoff_clearance_right,landing_wheel_ground_blend,ground_pd_horizon,ground_pd_hl,ground_pd_kl,ground_pd_hr,ground_pd_kr,world_velocity_valid,odom_twist_z,thrust_release_active,thrust_release_blend,catch_stable_time,com_world_z,com_world_vz,com_velocity_valid,com_sample_stamp,com_forward_from_axle,com_height_above_axle,com_lean,com_lean_rate,com_balance_valid,thrust_feedback_vz,ground_gravity_hl,ground_gravity_kl,ground_gravity_hr,ground_gravity_kr,hip_effort_limit,knee_effort_limit,thrust_force_before_budget,thrust_knee_pd_left,torso_force_ff,torso_feedback,torso_hip_command,torso_rate,torso_horizon,torso_imu_fresh,torso_imu_fy,torso_imu_fz,hip_common_before_allocation,hip_differential_torque,capture_com_velocity,capture_world_valid,capture_world_active,capture_world_target,"
                           << "com_takeoff_latched_vz,com_takeoff_confirmed_vz,apex_com_z_delta,"
                           << "thrust_fwd_term,thrust_att_term,thrust_wheel_target,thrust_cmd_x,thrust_com_stamp,thrust_com_valid,"
                           << "wheel_pitch_err,wheel_balance_rate,wheel_k_theta,wheel_k_theta_dot,wheel_control_height,"
                           << "wheel_p_term,wheel_d_term,wheel_attitude_cmd,wheel_vel_error,wheel_vel_term,"
                           << "wheel_raw_pos_err,wheel_pos_error,wheel_pos_term,wheel_cmd_raw,wheel_cmd_pre_clamp,"
                           << "wheel_min_fwd_cmd,wheel_max_bwd_cmd,wheel_cmd_target,"
                           << "odom_base_world_x,odom_base_world_y,odom_pose_valid,odom_sample_stamp,imu_sample_stamp,joint_sample_stamp,jump_cmd_rx_stamp,jump_cmd_accept_stamp,control_sim_dt_ms,control_wall_dt_ms,timer_calls_since_control,pitch_rate_raw,flight_arrest_freewheel_active,effort_switch_request_stamp,effort_switch_ack_stamp,effort_switch_result,effort_mode_active,leg_mode_switch_pending,thrust_gate_pitch_err,thrust_gate_rate_err,thrust_gate_stable_elapsed,thrust_gate_open_stamp,thrust_hip_requested_left,thrust_hip_requested_right,thrust_hip_floor_applied,fast_pitch_rate,capture_anchor_active,capture_v_ref,jump_id,prepare_subphase,jump_ready,jump_reject_reason,centroidal_legacy_lean_rate,centroidal_aligned_lean_rate,centroidal_aligned_rate_valid,centroidal_aligned_rate_stamp,squat_effort_support_preserved,touchdown_torso_convergence_active,touchdown_torso_convergence_blend,thrust_forward_velocity_kp,thrust_forward_attitude_taper_enabled,thrust_att_term_raw,thrust_attitude_scale,centroidal_world_x,centroidal_world_y,centroidal_world_z,centroidal_world_sample_stamp,centroidal_world_velocity_valid,jump_forward_axis_x,jump_forward_axis_y,jump_forward_axis_valid,thrust_release_fast_rate_comp_enabled,thrust_release_fast_rate_comp_requested,thrust_release_fast_rate_comp_applied,initial_balance_wheel_slew_active,initial_balance_wheel_raw_target,initial_balance_wheel_applied_cmd,thrust_forward_speed_prediction_enabled,thrust_forward_speed_raw,thrust_forward_speed_base,thrust_forward_speed_predicted,thrust_forward_speed_acceleration,thrust_forward_speed_horizon,thrust_forward_speed_delta,thrust_forward_speed_prediction_active,thrust_wheel_kinematics_enabled,thrust_wheel_kinematics_stamp,thrust_wheel_kinematics_age,thrust_wheel_kinematics_dt,thrust_wheel_kinematics_r_dot,thrust_wheel_kinematics_shank_projection,thrust_wheel_kinematics_raw_correction,thrust_wheel_kinematics_applied_correction,thrust_wheel_kinematics_valid,thrust_wheel_kinematics_q0,thrust_wheel_kinematics_q1,thrust_wheel_kinematics_q2,thrust_wheel_kinematics_q3,thrust_wheel_kinematics_R00,thrust_wheel_kinematics_R01,thrust_wheel_kinematics_R02,thrust_wheel_kinematics_R10,thrust_wheel_kinematics_R11,thrust_wheel_kinematics_R12,thrust_wheel_kinematics_R20,thrust_wheel_kinematics_R21,thrust_wheel_kinematics_R22,thrust_wheel_kinematics_relative_x,thrust_wheel_kinematics_relative_y,thrust_wheel_kinematics_relative_z,thrust_fast_rate_correction_from_gate_enabled,thrust_fast_rate_correction_from_gate_scope,thrust_fast_rate_correction_from_gate_blend,complete_contact_takeoff_enabled,contact_source_valid,contact_source_stamp_ns,contact_source_seq,contact_zero_frames,contact_zero_span_ns,contact_takeoff_confirmed,contact_source_taken,contact_takeoff_frame_stamp_ns,contact_takeoff_com_stamp_ns,arrest_dynamics_ff_enabled,arrest_dynamics_ff_scope,arrest_dynamics_ff_guard,arrest_dynamics_ff_blend,arrest_qdd_hl,arrest_qdd_kl,arrest_qdd_hr,arrest_qdd_kr,arrest_inertial_hl,arrest_inertial_kl,arrest_inertial_hr,arrest_inertial_kr,arrest_bias_hl,arrest_bias_kl,arrest_bias_hr,arrest_bias_kr,arrest_ff_raw_hl,arrest_ff_raw_kl,arrest_ff_raw_hr,arrest_ff_raw_kr,arrest_ff_bounded_hl,arrest_ff_bounded_kl,arrest_ff_bounded_hr,arrest_ff_bounded_kr,arrest_ff_applied_hl,arrest_ff_applied_kl,arrest_ff_applied_hr,arrest_ff_applied_kr,reference_ground_torque_pulse_enabled,reference_ground_torque_pulse_guard,reference_ground_torque_pulse_gate_elapsed,reference_ground_torque_pulse_shape,reference_ground_torque_contact_valid,reference_ground_torque_contact_continuous,reference_ground_torque_contact_wheel_mask,reference_ground_torque_contact_seq,reference_ground_torque_contact_stamp_ns,reference_ground_torque_contact_age_ms,reference_ground_torque_offset_requested_left,reference_ground_torque_offset_requested_right,reference_ground_torque_offset_applied_left,reference_ground_torque_offset_applied_right,imu_relative_pitch,sim_imu_reference_roll,landing_model_h_valid,landing_model_h_stamp,landing_model_h_total,landing_model_h_body_rate_term,landing_model_h_joint_rate_term,landing_model_h_wheel_relative_spin_term,landing_model_h_pitch_inertia,thrust_momentum_reference_enabled,thrust_momentum_reference_active,thrust_momentum_reference_blend,thrust_momentum_reference_vz,thrust_momentum_reference_target_h,thrust_momentum_reference_hip_delta,thrust_momentum_reference_knee_delta,thrust_momentum_reference_hip_feedback_left,thrust_momentum_reference_hip_feedback_right,reference_handoff_enabled,reference_handoff_mode,reference_handoff_valid,reference_handoff_limited,reference_handoff_force_suppressed,reference_handoff_target_hl,reference_handoff_target_kl,reference_handoff_target_hr,reference_handoff_target_kr,reference_handoff_applied_hl,reference_handoff_applied_kl,reference_handoff_applied_hr,reference_handoff_applied_kr,reference_handoff_rate_hl,reference_handoff_rate_kl,reference_handoff_rate_hr,reference_handoff_rate_kr,thrust_support_coordination_enabled,thrust_support_active,thrust_support_guard,thrust_support_blend,thrust_support_contact_forward,thrust_support_contact_vertical,thrust_support_nominal_forward_velocity,thrust_support_desired_forward_velocity,thrust_support_resulting_forward_velocity,thrust_support_nominal_vertical_velocity,thrust_support_resulting_vertical_velocity,thrust_support_jacobian_determinant,thrust_support_imu_stamp,thrust_support_joint_stamp,thrust_support_com_stamp,thrust_support_contact_stamp,qdot_support_hl,qdot_support_kl,qdot_support_hr,qdot_support_kr,dqdot_support_hl,dqdot_support_kl,dqdot_support_hr,dqdot_support_kr,thrust_support_hip_servo_left_nm,thrust_support_hip_servo_right_nm,thrust_support_imu_joint_delta_ms,thrust_support_com_imu_delta_ms,thrust_support_com_pair_delta_ms,thrust_support_contact_imu_delta_ms,thrust_support_allocator_enabled,thrust_support_allocator_active,thrust_support_allocator_legacy_active,thrust_support_allocator_contact_hold,thrust_support_allocator_guard,thrust_support_allocator_valid,thrust_support_allocator_reason,wheel_servo_gain,allocator_target_H,allocator_task_az,allocator_task_Hdot,allocator_task_com_rel_acc,allocator_current_H,allocator_com_rel_forward,allocator_com_rel_velocity,allocator_solve_time_us,allocator_eq_residual,allocator_ineq_residual,allocator_vertical_residual,allocator_Hdot_residual,allocator_com_rel_residual,allocator_tau_hip_l,allocator_tau_knee_l,allocator_tau_hip_r,allocator_tau_knee_r,allocator_actual_wheel_command,allocator_actual_tau_hip_l,allocator_actual_tau_knee_l,allocator_actual_tau_hip_r,allocator_actual_tau_knee_r,allocator_body_pitch_enabled,allocator_pitch_target,allocator_pitch_acc_task,allocator_pitch_acc_achieved,allocator_pitch_acc_residual,allocator_selected_contact_valid,allocator_selected_contact_ns,allocator_selected_contact_mask,allocator_selected_contact_age_ns,control_sim_time_ns,ground_input_stage,ground_input_reason\n";

            jump_log_file_.flush();
        }
    }

    void close_log_files()
    {
        if (jump_log_file_.is_open())
        {
            jump_log_file_.close();
        }
        if (jump_summary_file_.is_open())
        {
            jump_summary_file_.close();
        }
        if (motion_trace_log_.is_open()) motion_trace_log_.close();
        if (phase_events_log_.is_open()) phase_events_log_.close();
    }

    void write_jump_summary(bool recovery_completed, const std::string &reason)
    {
        if (jump_summary_written_ || !jump_summary_file_.is_open())
            return;
        const bool world_height_valid = world_height_valid_for_jump_ && odom_received_;
        const double apex_world_z_delta = world_height_valid ? (max_world_z_during_jump_ - thrust_start_world_z_) : -1.0;
        std::string effective_reason = reason;
        if (!world_height_valid)
        {
            if (!effective_reason.empty())
                effective_reason += "; ";
            effective_reason += "Gazebo世界高度里程计不可用";
        }
        const bool effective_recovery_completed = recovery_completed && world_height_valid;
        const double apex_com_z_delta = (max_com_z_during_jump_ > 0.0 && takeoff_com_z_ > 0.0) ?
                                        (max_com_z_during_jump_ - takeoff_com_z_) : -1.0;
        std::string effective_exit_code = exit_code_;
        if (effective_exit_code == "NOT_STARTED" || effective_exit_code == "IN_PROGRESS")
        {
            if (effective_recovery_completed && effective_reason.empty() && tuck_entered_ && extend_entered_ && protective_reason_.empty())
                effective_exit_code = "SUCCESS";
            else if (!protective_reason_.empty())
                effective_exit_code = "FAIL_PROTECTIVE_DEPLOY";
            else if (!effective_reason.empty())
                effective_exit_code = "FAIL_" + effective_reason;
            else
                effective_exit_code = "FAIL_INCOMPLETE";
        }
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
                           << "\"" << effective_reason << "\","
                           << takeoff_com_z_ << ","
                           << takeoff_com_vz_ << ","
                           << takeoff_com_vx_ << ","
                           << apex_com_z_delta << ","
                           << (tuck_entered_ ? 1 : 0) << ","
                           << (extend_entered_ ? 1 : 0) << ","
                           << "\"" << protective_reason_ << "\","
                           << touchdown_time_ << ","
                           << recovery_time_ << ","
                           << balance_return_time_ << ","
                           << "\"" << effective_exit_code << "\"\n";
        jump_summary_file_.flush();
        jump_summary_written_ = true;
    }

    void log_data(double cmd_x, double tau_hip, double tau_knee, double f_z)
    {
        const double t = control_log_timestamp_sec_;
        if (current_state_ != bbot_jump::STATE_BALANCE)
        {
            max_z_during_jump_ = std::max(max_z_during_jump_, current_z_);
            if (centroidal_height_.height() > 0.0)
                max_com_z_during_jump_ = std::max(max_com_z_during_jump_, centroidal_height_.height());
            max_abs_pitch_during_jump_ = std::max(max_abs_pitch_during_jump_, std::abs(pitch_));
            const double commanded_hip = std::max({std::abs(hip_cmd_left_), std::abs(hip_cmd_right_),
                                                   std::abs(tau_hip)});
            const double commanded_knee = std::max({std::abs(knee_cmd_left_), std::abs(knee_cmd_right_),
                                                    std::abs(tau_knee)});
            max_abs_hip_torque_during_jump_ = std::max(max_abs_hip_torque_during_jump_, commanded_hip);
            max_abs_knee_torque_during_jump_ = std::max(max_abs_knee_torque_during_jump_, commanded_knee);
        }
        if (jump_log_file_.is_open())
        {
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
                           << capture_world_target_ << ","
                           << com_takeoff_latched_vz_ << ","
                           << com_takeoff_confirmed_vz_ << ","
                           << ((max_com_z_during_jump_ > 0.0 && takeoff_com_z_ > 0.0) ? (max_com_z_during_jump_ - takeoff_com_z_) : -1.0) << ","
                           << thrust_fwd_term_diag_ << ","
                           << thrust_att_term_diag_ << ","
                           << thrust_wheel_target_diag_ << ","
                           << thrust_cmd_x_diag_ << ","
                           << thrust_com_sample_stamp_diag_ << ","
                           << thrust_com_valid_diag_ << ","
                           << wheel_pitch_err_diag_ << ","
                           << wheel_balance_rate_diag_ << ","
                           << wheel_k_theta_diag_ << ","
                           << wheel_k_theta_dot_diag_ << ","
                           << wheel_control_height_diag_ << ","
                           << wheel_p_term_diag_ << ","
                           << wheel_d_term_diag_ << ","
                           << wheel_attitude_cmd_diag_ << ","
                           << wheel_vel_error_diag_ << ","
                           << wheel_vel_term_diag_ << ","
                           << wheel_raw_pos_err_diag_ << ","
                           << wheel_pos_error_diag_ << ","
                           << wheel_pos_term_diag_ << ","
                           << wheel_cmd_raw_diag_ << ","
                           << wheel_cmd_pre_clamp_diag_ << ","
                           << wheel_min_fwd_cmd_diag_ << ","
                           << wheel_max_bwd_cmd_diag_ << ","
                           << wheel_cmd_target_diag_ << ","
                           << odom_base_position_.x() << ","
                           << odom_base_position_.y() << ","
                           << (odom_received_ && takeoff_odom_pose_valid_) << ","
                           << takeoff_odom_stamp_ << ","
                           << torso_imu_.stamp() << ","
                           << joint_sample_time_ << ","
                           << jump_cmd_rx_stamp_ << ","
                           << jump_cmd_accept_stamp_ << ","
                           << control_sim_dt_ms_diag_ << ","
                           << control_wall_dt_ms_diag_ << ","
                           << timer_calls_for_update_diag_ << ","
                           << pitch_rate_raw_ << ","
                           << (current_state_ == bbot_jump::STATE_FLIGHT && flight_arrest_freewheel_active_) << ","
                           << effort_switch_request_stamp_ << ","
                           << effort_switch_ack_stamp_ << ","
                           << effort_switch_result_ << ","
                           << effort_mode_active_ << ","
                           << leg_mode_switch_pending_ << ","
                           << thrust_gate_pitch_err_diag_ << ","
                           << thrust_gate_rate_err_diag_ << ","
                           << thrust_gate_stable_elapsed_diag_ << ","
                           << thrust_gate_open_stamp_ << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_hip_requested_left_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_hip_requested_right_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST && thrust_hip_floor_applied_diag_) << ","
                           << torso_imu_.rate() << ","
                           << capture_anchor_active_ << ","
                           << capture_v_ref_ << ","
                           << jump_id_ << ","
                           << (current_state_ == bbot_jump::STATE_PRE_JUMP
                                   ? (!pre_jump_capture_armed_ ? "EFFORT_CAPTURE"
                                      : (pre_jump_effort_capture_ && pre_jump_roll_in_elapsed_ < 0.35
                                             ? "ROLL_IN" : "ROLLING")) : "") << ","
                           << jump_ready_for_command() << ","
                           << jump_reject_reason_diag_ << ","
                           << centroidal_legacy_lean_rate_ << ","
                           << centroidal_lean_rate_.rate() << ","
                           << centroidal_lean_rate_.valid(this->now().seconds()) << ","
                           << centroidal_lean_rate_.stamp() << ","
                           << effort_support_handoff_preserved_ << ","
                           << touchdown_torso_convergence_latched_ << ","
                           << touchdown_torso_convergence_blend_ << ","
                           << thrust_forward_velocity_kp_ << ","
                           << thrust_forward_attitude_taper_enable_ << ","
                           << thrust_att_term_raw_diag_ << ","
                           << thrust_attitude_scale_diag_ << ","
                           << centroidal_world_.position().x() << ","
                           << centroidal_world_.position().y() << ","
                           << centroidal_world_.position().z() << ","
                           << centroidal_world_.stamp() << ","
                           << centroidal_world_.valid(this->now().seconds()) << ","
                           << jump_forward_axis_world_.x() << ","
                           << jump_forward_axis_world_.y() << ","
                           << jump_forward_axis_valid_ << ","
                           << (current_state_ == bbot_jump::STATE_THRUST &&
                               thrust_release_fast_rate_correction_active_diag_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_release_fast_rate_correction_requested_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_release_fast_rate_correction_applied_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_BALANCE &&
                               initial_balance_wheel_slew_active_diag_) << ","
                           << (current_state_ == bbot_jump::STATE_BALANCE
                                   ? initial_balance_wheel_raw_target_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_BALANCE
                                   ? initial_balance_wheel_applied_cmd_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST &&
                               thrust_forward_speed_prediction_enable_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_raw_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_base_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_predicted_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_accel_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_horizon_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST
                                   ? thrust_forward_speed_delta_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST &&
                               thrust_forward_speed_prediction_active_diag_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST &&
                               thrust_wheel_kinematics_compensation_enable_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_stamp_diag_ : -1.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_age_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_dt_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_r_dot_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_shank_projection_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_raw_correction_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_applied_correction_diag_ : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST && thrust_kinematics_valid_diag_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_q_diag_[0] : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_q_diag_[1] : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_q_diag_[2] : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_q_diag_[3] : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(0,0) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(0,1) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(0,2) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(1,0) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(1,1) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(1,2) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(2,0) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(2,1) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_rotation_diag_(2,2) : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_relative_body_diag_.x() : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_relative_body_diag_.y() : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_kinematics_relative_body_diag_.z() : 0.0) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST && thrust_fast_rate_correction_from_gate_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST && thrust_fast_rate_correction_scope_diag_) << ","
                           << (current_state_ == bbot_jump::STATE_THRUST ? thrust_fast_rate_correction_blend_diag_ : 0.0) << ","
                           << complete_contact_takeoff_confirmation_enable_ << ","
                           << contact_takeoff_observer_.source_fresh(this->now().nanoseconds()) << ","
                           << contact_takeoff_observer_.latest_stamp_ns() << ","
                           << contact_takeoff_observer_.latest_sequence() << ","
                           << contact_takeoff_observer_.zero_frame_count() << ","
                           << contact_takeoff_observer_.zero_span_ns() << ","
                           << contact_takeoff_confirmed_diag_ << ","
                           << contact_takeoff_observer_.source_taken() << ","
                           << contact_takeoff_observer_.source_take_stamp_ns() << ","
                           << contact_takeoff_observer_.source_take_com_stamp_ns() << ","
                           << arrest_dynamics_feedforward_enable_ << ","
                           << arrest_dynamics_feedforward_scope_diag_ << ","
                           << arrest_dynamics_feedforward_guard_diag_ << ","
                           << arrest_dynamics_feedforward_blend_diag_;
            const std::array<const std::array<double, 4> *, 6> arrest_diag = {
                &arrest_dynamics_qdd_diag_, &arrest_dynamics_inertial_diag_,
                &arrest_dynamics_bias_diag_, &arrest_dynamics_raw_diag_,
                &arrest_dynamics_bounded_diag_, &arrest_dynamics_applied_diag_};
            for (const auto * values : arrest_diag)
                for (double value : *values)
                    jump_log_file_ << "," << value;
            jump_log_file_ << "," << reference_ground_torque_pulse_enable_
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_guard_diag_ :
                                             static_cast<int>(bbot_jump::ReferenceGroundPulseGuard::WrongPhase))
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_gate_elapsed_diag_ : 0.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_shape_diag_ : 0.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST &&
                                      reference_ground_pulse_contact_valid_)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST &&
                                      reference_ground_pulse_contact_continuous_)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? static_cast<int>(reference_ground_pulse_wheel_mask_) : 0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_sequence_ : 0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_contact_stamp_ns_ : 0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST &&
                                      reference_ground_pulse_contact_stamp_ns_ > 0
                                         ? (this->now().nanoseconds() -
                                            reference_ground_pulse_contact_stamp_ns_) / 1.0e6 : -1.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_requested_left_diag_ : 0.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_requested_right_diag_ : 0.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_applied_left_diag_ : 0.0)
                           << "," << (current_state_ == bbot_jump::STATE_THRUST
                                           ? reference_ground_pulse_applied_right_diag_ : 0.0);

            jump_log_file_ << "," << imu_relative_pitch_ << "," << sim_imu_reference_roll_;
            jump_log_file_ << "," << landing_momentum_diag_.valid
                << "," << landing_momentum_stamp_diag_
                << "," << landing_momentum_diag_.total
                << "," << landing_momentum_diag_.body_rate_term
                << "," << landing_momentum_diag_.joint_rate_term
                << "," << landing_momentum_diag_.wheel_relative_spin_term
                << "," << landing_momentum_diag_.pitch_inertia;
            jump_log_file_ << "," << thrust_momentum_reference_enable_
                << "," << thrust_momentum_reference_active_diag_
                << "," << thrust_momentum_reference_blend_diag_
                << "," << thrust_momentum_reference_vz_diag_
                << "," << thrust_momentum_reference_h_diag_
                << "," << thrust_momentum_reference_hip_delta_diag_
                << "," << thrust_momentum_reference_knee_delta_diag_
                << "," << thrust_momentum_reference_hip_feedback_l_diag_
                << "," << thrust_momentum_reference_hip_feedback_r_diag_;
            jump_log_file_ << "," << thrust_reference_handoff_enable_
                << "," << handoff_reference_mode_diag_ << "," << handoff_reference_valid_diag_
                << "," << handoff_reference_limited_diag_ << "," << handoff_force_suppressed_diag_;
            for (double x:handoff_reference_target_diag_) jump_log_file_ << "," << x;
            for (double x:handoff_reference_applied_diag_) jump_log_file_ << "," << x;
            for (double x:handoff_reference_rate_diag_) jump_log_file_ << "," << x;
            jump_log_file_ << "," << thrust_support_coordination_enable_
                << "," << thrust_support_reference_diag_.active
                << "," << static_cast<int>(thrust_support_reference_diag_.guard)
                << "," << thrust_support_reference_diag_.blend
                << "," << thrust_support_reference_diag_.contact_forward
                << "," << thrust_support_reference_diag_.contact_vertical
                << "," << thrust_support_reference_diag_.nominal_forward_velocity
                << "," << thrust_support_reference_diag_.desired_forward_velocity
                << "," << thrust_support_reference_diag_.resulting_forward_velocity
                << "," << thrust_support_reference_diag_.nominal_vertical_velocity
                << "," << thrust_support_reference_diag_.resulting_vertical_velocity
                << "," << thrust_support_reference_diag_.jacobian_determinant
                << "," << thrust_support_imu_stamp_diag_
                << "," << thrust_support_joint_stamp_diag_
                << "," << thrust_support_com_stamp_diag_
                << "," << reference_ground_pulse_contact_stamp_ns_*1.0e-9;
            for (double x : thrust_support_reference_diag_.qdot) jump_log_file_ << "," << x;
            for (double x : thrust_support_reference_diag_.delta_qdot) jump_log_file_ << "," << x;
            jump_log_file_ << ","
                << thrust_support_hip_servo_left_diag_
                << "," << thrust_support_hip_servo_right_diag_
                << ","
                << (thrust_support_imu_stamp_diag_ > 0.0 && thrust_support_joint_stamp_diag_ > 0.0
                        ? 1000.0*(thrust_support_joint_stamp_diag_-thrust_support_imu_stamp_diag_) : -1.0)
                << "," << (thrust_support_imu_stamp_diag_ > 0.0 && thrust_support_com_stamp_diag_ > 0.0
                        ? 1000.0*(thrust_support_com_stamp_diag_-thrust_support_imu_stamp_diag_) : -1.0)
                << "," << (thrust_support_joint_stamp_diag_ > 0.0 && thrust_support_com_stamp_diag_ > 0.0
                        ? 1000.0*(thrust_support_com_stamp_diag_-thrust_support_joint_stamp_diag_) : -1.0)
                << "," << (thrust_support_imu_stamp_diag_ > 0.0 && reference_ground_pulse_contact_stamp_ns_ > 0
                        ? 1000.0*(reference_ground_pulse_contact_stamp_ns_*1.0e-9-thrust_support_imu_stamp_diag_) : -1.0)
                << "," << thrust_support_allocator_enable_
                << "," << thrust_support_allocator_active_diag_
                << "," << thrust_support_allocator_legacy_active_diag_
                << "," << thrust_support_allocator_contact_hold_diag_
                << "," << thrust_support_allocator_guard_diag_
                << "," << thrust_support_allocation_diag_.valid
                << "," << thrust_support_allocation_diag_.reason
                << "," << wheel_servo_gain_
                << "," << thrust_support_target_h_
                << "," << thrust_support_tasks_diag_.desired_com_vertical_acceleration
                << "," << thrust_support_tasks_diag_.desired_centroidal_hdot
                << "," << thrust_support_tasks_diag_.desired_com_axle_relative_forward_acceleration
                << "," << landing_momentum_diag_.total
                << "," << thrust_support_com_rel_forward_diag_
                << "," << thrust_support_com_rel_velocity_diag_
                << "," << thrust_support_allocation_diag_.solve_time_us
                << "," << thrust_support_allocation_diag_.maximum_equality_residual
                << "," << thrust_support_allocation_diag_.residuals.max_inequality_violation
                << "," << thrust_support_allocation_diag_.residuals.vertical_acceleration
                << "," << thrust_support_allocation_diag_.residuals.centroidal_hdot
                << "," << thrust_support_allocation_diag_.residuals.com_axle_relative_forward_acceleration
                << "," << thrust_support_allocation_diag_.decision[9]
                << "," << thrust_support_allocation_diag_.decision[10]
                << "," << thrust_support_allocation_diag_.decision[11]
                << "," << thrust_support_allocation_diag_.decision[12]
                << "," << thrust_support_actual_wheel_command_diag_;
            for (double effort : thrust_support_actual_leg_command_diag_)
                jump_log_file_ << "," << effort;
            jump_log_file_ << "," << thrust_support_tasks_diag_.body_pitch_enabled
                << "," << jump_pitch_ref_
                << "," << thrust_support_tasks_diag_.desired_body_pitch_acceleration
                << "," << -thrust_support_allocation_diag_.decision[2]
                << "," << thrust_support_allocation_diag_.residuals.body_pitch_acceleration
                << "," << thrust_support_contact_diag_.valid
                << "," << thrust_support_contact_diag_.stamp_ns
                << "," << int(thrust_support_contact_diag_.wheel_mask)
                << "," << (thrust_support_contact_diag_.valid ?
                    static_cast<int64_t>(std::llround(t*1e9))-thrust_support_contact_diag_.stamp_ns : -1)
                << "," << capture_control_ns_
                << "," << ground_input_stage_
                << "," << ground_input_reason_
                << "\n";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BBotLandingRepairController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
