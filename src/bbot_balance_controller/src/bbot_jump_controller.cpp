#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "bbot_balance_controller/keyboard_reader.h"
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
    STATE_BALANCE,            // 0: 变高度 LQR 自平衡状态
    STATE_SQUAT,              // 1: 下蹲蓄力阶段 (L -> L_SQUAT_)
    STATE_THRUST,             // 2: 爆发推地阶段 (L_SQUAT_ -> H_TAKEOFF_)
    STATE_FLIGHT,             // 3: 腾空相阶段 (冲顶 -> 收腿 0.30m -> 预展腿 0.40m)
    STATE_TOUCHDOWN_BUFFER,   // 4: 触地缓冲阻抗控制
    STATE_RECOVERY,           // 5: 平稳沉降与消除反弹
    STATE_STANDUP,            // 6: 倒地起立自恢复
    STATE_EMERGENCY           // 7: 紧急停机
};

inline const char* state_to_string(JumpState s)
{
    switch (s) {
        case STATE_BALANCE: return "BALANCE";
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

class BBotJumpController : public rclcpp::Node
{
public:
    BBotJumpController()
        : Node("bbot_jump_controller")
    {
        gain_low_ = {-6.1624, -46.8436, -197.6985, -46.8109};
        gain_high_ = {-6.3650, -48.5719, -229.4004, -58.6391};
        current_gain_ = gain_high_;

        balance_offset_ = 0.034;
        cmd_scale_ = 0.030;
        wheel_radius_ = 0.07;
        max_cmd_x_ = 10.0;

        walk_speed_ = 0.50;
        turn_speed_ = 0.60;
        speed_ramp_time_ = 1.0;

        L_MIN_ = 0.30;
        L_MAX_ = 0.50;
        L_STAND_ = 0.40;
        
        target_height_ = L_STAND_;
        current_height_ = target_height_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0; // 0.05 m/s

        // ── 跳跃核心参数 ──
        L_SQUAT_ = 0.30;          // 下蹲蓄力高度 [m]
        T_SQUAT_ = 0.45;          // 下蹲过渡时间 [s]

        T_THRUST_ = 0.160;        // 推地规划时间 [s]
        H_TAKEOFF_ = 0.475;       // 离地目标高度 [m]
        V_TAKEOFF_ = 2.30;        // 离地初速度 [m/s]
        K_BODY_P_THRUST_ = 20.0;  // 推地姿态刚度 [Nm/rad]
        K_BODY_D_THRUST_ = 3.0;   // 推地姿态阻尼 [Nm*s/rad]
        TAU_HIP_BODY_MAX_ = 18.0; // 髋关节姿态补偿力矩限幅 [Nm]
        K_BODY_P_BUFFER_ = 12.0;  // 缓冲阶段姿态刚度 [Nm/rad]
        K_BODY_D_BUFFER_ = 1.0;   // 缓冲阶段姿态阻尼 [Nm*s/rad]

        L_RETRACT_ = L_MIN_;      // 腾空收腿目标高度 [m]
        L_TOUCH_ = 0.450;         // 腾空展腿着陆高度 [m]
        T_FLIGHT_TUCK_ = 0.12;    // 收腿时间 [s]
        T_FLIGHT_APEX_ = 0.25;    // 展腿起始时刻 [s]
        T_FLIGHT_EXTEND_ = 0.12;  // 展腿时间 [s]
        T_FLIGHT_TIMEOUT_ = 0.60; // 腾空超时保护阈值 [s]
        PITCH_FLIGHT_GUARD_ = 0.45; // 腾空姿态保护阈值 [rad]

        K_Z_BUFFER_ = 450.0;      // 触地阻抗刚度 [N/m]
        D_Z_BUFFER_ = 45.0;       // 触地阻抗阻尼 [N*s/m]
        F_Z_BUFFER_MAX_ = 200.0;  // 单腿缓冲支撑力上限 [N]
        TOTAL_MASS_ = kinematics_.get_params().M_total; // 机器人总质量 [kg]
        height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;

        // 日志路径初始化
        const char * home_dir = getenv("HOME");
        data_path_ = std::string(home_dir ? home_dir : "/home/admin") + "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/";
        open_log_files();

        // ── 订阅话题 ──
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BBotJumpController::imu_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&BBotJumpController::joint_state_callback, this, std::placeholders::_1));

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
        timer_ = this->create_wall_timer(5ms, std::bind(&BBotJumpController::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=====================================================");
        RCLCPP_INFO(this->get_logger(), "  BBot 跳跃与 LQR 复合控制器已启动 (200Hz)");
        RCLCPP_INFO(this->get_logger(), "  支持按键: J (跳跃), W/A/S/D (遥控), Q/E (变高度), R (起立), X (停机)");
        RCLCPP_INFO(this->get_logger(), "=====================================================");
    }

    ~BBotJumpController()
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
    bool touchdown_buffer_initialized_ = false;
    bool protective_landing_ = false;
    double recovery_x_ref_ = 0.0;        // 落地恢复阶段位置参考
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

    // ── 触地检测计数器 ──
    int touchdown_knee_effort_count_ = 0;
    int touchdown_stable_count_ = 0;

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
    double K_BODY_P_BUFFER_;
    double K_BODY_D_BUFFER_;
    double L_RETRACT_;
    double L_TOUCH_;
    double T_FLIGHT_TUCK_;
    double T_FLIGHT_APEX_;
    double T_FLIGHT_EXTEND_;
    double T_FLIGHT_TIMEOUT_;
    double PITCH_FLIGHT_GUARD_;
    double K_Z_BUFFER_;
    double D_Z_BUFFER_;
    double F_Z_BUFFER_MAX_;
    double TOTAL_MASS_;

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

    // ── 模式与跳跃触发 ──
    void trigger_jump()
    {
        if (current_state_ != bbot_jump::STATE_BALANCE) {
            RCLCPP_WARN(this->get_logger(), "[跳跃请求忽略] 当前不在 BALANCE 自平衡状态 (当前: %s)",
                        bbot_jump::state_to_string(current_state_));
            return;
        }

        RCLCPP_INFO(this->get_logger(), ">>> 收到跳跃指令！启动阶段 1：下蹲蓄力 (SQUAT)... <<<");
        double now_sec = this->now().seconds();
        current_state_ = bbot_jump::STATE_SQUAT;
        state_start_time_ = now_sec;
        protective_landing_ = false;
        post_landing_effort_support_ = false;
        post_landing_position_handoff_requested_ = false;
        post_landing_position_settle_count_ = 0;
        post_landing_translation_feedback_ = false;
        post_landing_gyro_reduced_ = false;
        post_landing_pitch_ref_ = balance_offset_;
        position_switch_time_ = -1.0;

        // 初始化下蹲五次多项式 (从当前高度平滑下蹲到 L_SQUAT_)
        quintic_traj_.init(now_sec, T_SQUAT_,
                           current_height_, 0.0, 0.0,
                           L_SQUAT_, 0.0, 0.0);
        
        target_speed_const_ = 0.0;
        target_yaw_rate_ = 0.0;
        target_x_ = x_;
        was_moving_ = false;
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
        bool has_left = false, has_right = false;
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
                if (i < msg->position.size()) hip_pos_left_ = msg->position[i];
                if (i < msg->velocity.size()) hip_vel_left_ = msg->velocity[i];
                if (i < msg->effort.size()) hip_effort_left_ = msg->effort[i];
            } else if (msg->name[i] == "link_003_joint") {
                if (i < msg->position.size()) knee_pos_left_ = msg->position[i];
                if (i < msg->velocity.size()) knee_vel_left_ = msg->velocity[i];
                if (i < msg->effort.size()) knee_effort_left_ = msg->effort[i];
            } else if (msg->name[i] == "link_005_joint") {
                if (i < msg->position.size()) hip_pos_right_ = msg->position[i];
                if (i < msg->velocity.size()) hip_vel_right_ = msg->velocity[i];
                if (i < msg->effort.size()) hip_effort_right_ = msg->effort[i];
            } else if (msg->name[i] == "link_006_joint") {
                if (i < msg->position.size()) knee_pos_right_ = msg->position[i];
                if (i < msg->velocity.size()) knee_vel_right_ = msg->velocity[i];
                if (i < msg->effort.size()) knee_effort_right_ = msg->effort[i];
            }
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
        rclcpp::Time now_t = this->now();
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
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0001 || dt > 0.05) dt = 0.005;

        // 执行当前状态机分支
        switch (current_state_)
        {
            case bbot_jump::STATE_BALANCE:
                run_state_balance(dt);
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
                run_state_recovery(now_sec);
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
    }

    // ── 阶段 1：下蹲蓄力 (SQUAT) ──
    void run_state_squat(double now_sec, double dt)
    {
        (void)dt;
        double elapsed = now_sec - state_start_time_;
        double des_z, des_v, des_acc;
        quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);

        current_height_ = des_z; 

        // 下蹲重力补偿力矩 + 高刚度轨迹跟踪 (Kp=350, Kd=14)
        bbot_kinematics::IKSolution ik_sq = kinematics_.inverse_kinematics(des_z, 0.0);
        bbot_kinematics::JointTorques g_torques = kinematics_.compute_gravity_torques(0.0, ik_sq.theta_hip, ik_sq.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;
        if (effort_mode_active_) {
            support_force_total = publish_effort_height_control(
                des_z, des_v,
                550.0, 90.0, 220.0,
                25.0, 4.0, 45.0, 6.0,
                true);
        } else {
            publish_position_leg_control(ik_sq.theta_hip, ik_sq.theta_knee);
        }

        // 下蹲期间车轮全状态 LQR 自平衡
        interpolate_lqr_gain();
        double pos_error = x_ - target_x_;
        double vel_error = x_dot_;
        double pitch_err = pitch_ - balance_offset_;
        double u_pitch = -(current_gain_.k_theta * pitch_err + current_gain_.k_theta_dot * pitch_rate_
                           + current_gain_.k_x * pos_error + current_gain_.k_x_dot * vel_error);
        double cmd_x = bbot_jump::clamp_value(-u_pitch * cmd_scale_, -max_cmd_x_, max_cmd_x_);
        publish_wheel_cmd(cmd_x, 0.0);

        log_data(cmd_x, g_torques.hip_torque * 0.5, g_torques.knee_torque * 0.5,
                 support_force_total);

        // 下蹲蓄力完成条件：轨迹结束且沉降稳定
        bool traj_done = (elapsed >= T_SQUAT_);
        bool settled = (std::abs(current_z_dot_) < 0.06 && std::abs(current_z_ - L_SQUAT_) < 0.025);
        bool squat_timeout = (elapsed >= T_SQUAT_ + 0.15);

        if (traj_done && (settled || squat_timeout)) {
            RCLCPP_INFO(this->get_logger(), ">>> 蓄力沉降稳定 (t=%.3fs, z=%.3f, v=%.2f)！启动阶段 2：全力爆发弹射推地 (THRUST)... <<<",
                        elapsed, current_z_, current_z_dot_);
            current_state_ = bbot_jump::STATE_THRUST;
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            thrust_trajectory_initialized_ = false;

            // 切换前预写重力支撑力矩
            publish_effort_leg_control(ik_sq.theta_hip, ik_sq.theta_knee, 0.0, 0.0,
                                       -g_torques.hip_torque * 0.5, -g_torques.knee_torque * 0.5,
                                       80.0, 8.0, 80.0, 8.0);
            request_effort_controller();

            prev_q_hip_des_ = ik_sq.theta_hip;
            prev_q_knee_des_ = ik_sq.theta_knee;
        }
    }

    // ── 阶段 2：爆发推地 (THRUST) ──
    void run_state_thrust(double now_sec, double dt)
    {
        if (!effort_mode_active_) {
            // 控制器异步切换期间保持下蹲姿态
            request_effort_controller();
            bbot_kinematics::IKSolution ik_hold = kinematics_.inverse_kinematics(L_SQUAT_, 0.0);
            publish_position_leg_control(ik_hold.theta_hip, ik_hold.theta_knee);
            return;
        }

        if (!thrust_trajectory_initialized_) {
            state_start_time_ = now_sec;
            state_start_z_ = current_z_;
            quintic_traj_.init(now_sec, T_THRUST_, state_start_z_, 0.0, 0.0,
                               H_TAKEOFF_, V_TAKEOFF_, 0.0);
            thrust_trajectory_initialized_ = true;
        }
        double elapsed = now_sec - state_start_time_;
        double pitch_err = pitch_ - balance_offset_;

        // 推地五次轨迹评估
        double des_z, des_v, des_acc;
        quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);
        current_height_ = des_z;

        // 1. 逆运动学求解当前规划角度
        bbot_kinematics::IKSolution ik = kinematics_.inverse_kinematics(des_z, 0.0);

        // 期望关节角速度
        double q_dot_hip_des = (dt > 1e-4) ? (ik.theta_hip - prev_q_hip_des_) / dt : 0.0;
        double q_dot_knee_des = (dt > 1e-4) ? (ik.theta_knee - prev_q_knee_des_) / dt : 0.0;
        prev_q_hip_des_ = ik.theta_hip;
        prev_q_knee_des_ = ik.theta_knee;

        // 2. 腿端几何雅可比力矩映射
        double Jz_hip_leg, Jz_knee_leg;
        compute_leg_vertical_jacobian(0.0, ik.theta_hip, ik.theta_knee,
                                      Jz_hip_leg, Jz_knee_leg);
        double m_single_leg = TOTAL_MASS_ * 0.5;
        double F_z_thrust = std::max(0.0, m_single_leg * (9.81 + des_acc));
        
        // 垂直推力前馈力矩
        double tau_ff_hip = F_z_thrust * Jz_hip_leg;
        double tau_ff_knee = F_z_thrust * Jz_knee_leg;

        // 3. 髋关节姿态稳定前馈补偿
        double tau_body_per_hip = -0.5 * (K_BODY_P_THRUST_ * pitch_err +
                                          K_BODY_D_THRUST_ * pitch_rate_);
        tau_body_per_hip = bbot_jump::clamp_value(tau_body_per_hip,
                                                   -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        tau_ff_hip += tau_body_per_hip;

        publish_effort_leg_control(ik.theta_hip, ik.theta_knee, q_dot_hip_des, q_dot_knee_des,
                                   tau_ff_hip, tau_ff_knee,
                                   0.0, 0.0,    // 髋：仅做接地力前馈与姿态补偿
                                   350.0, 4.0); // 膝：高刚度伸腿轨迹跟踪

        // 4. 推地期间车轮锁零速
        double cmd_x = 0.0;
        publish_wheel_cmd(cmd_x, 0.0);

        log_data(cmd_x, tau_ff_hip, tau_ff_knee, F_z_thrust * 2.0);

        // 离地状态判定
        bool stroke_completed = (current_z_ >= (H_TAKEOFF_ - 0.025));
        bool wheels_airborne = (elapsed > 0.07 && acc_z_filt_ < 6.5 &&
                                current_z_ >= state_start_z_ + 0.040);
        bool trajectory_takeoff = (quintic_traj_.is_finished(now_sec) &&
                                   current_z_ >= (H_TAKEOFF_ - 0.060));
        bool leg_collapsing = (elapsed > 0.04 && current_z_ < state_start_z_ - 0.025);
        bool thrust_timeout = (elapsed >= T_THRUST_ + 0.06);
        bool attitude_ready = (std::abs(pitch_err) < 0.35 && std::abs(pitch_rate_) < 2.5);

        if (leg_collapsing) {
            abort_jump_to_recovery(now_sec, "推地期间腿长反向缩短");
            return;
        }

        if (thrust_timeout && (!stroke_completed && !wheels_airborne && !trajectory_takeoff)) {
            abort_jump_to_recovery(now_sec, "推地超时且未确认伸腿/离地");
            return;
        }

        // 姿态失稳保护
        if (thrust_timeout && !attitude_ready) {
            transition_to_protective_landing(now_sec, "离地前姿态或角速度未稳定");
            return;
        }

        if ((stroke_completed || wheels_airborne || trajectory_takeoff) && attitude_ready) {
            const char * takeoff_reason = stroke_completed ? "伸展到位" :
                                         (wheels_airborne ? "IMU 失重" : "轨迹完成且接近伸展行程");
            RCLCPP_INFO(this->get_logger(), ">>> 离地爆发完成 [%s] (t=%.3fs, z=%.3f, v=%.2f, acc_z=%.1f)！切入腾空相 (FLIGHT)... <<<",
                        takeoff_reason, elapsed, current_z_, current_z_dot_, acc_z_filt_);
            current_state_ = bbot_jump::STATE_FLIGHT;
            state_start_time_ = now_sec;
            flight_start_z_ = current_z_;
            touchdown_knee_effort_count_ = 0;
            protective_landing_ = false;
            current_height_ = L_TOUCH_;

            // 保持力矩控制模式
            request_effort_controller();
        }
    }

    // ── 阶段 3：腾空相控制 (FLIGHT) ──
    void run_state_flight(double now_sec)
    {
        double elapsed = now_sec - state_start_time_;
        double pitch_err = pitch_ - balance_offset_;
        bool attitude_unstable = std::abs(pitch_err) > PITCH_FLIGHT_GUARD_;

        // 1. 空中收腿与展腿高度规划
        double L_target = flight_start_z_;
        if (protective_landing_) {
            L_target = L_TOUCH_;
        } else if (elapsed < T_FLIGHT_APEX_) {
            // 上升期收腿
            double tuck_progress = bbot_jump::clamp_value(elapsed / T_FLIGHT_TUCK_, 0.0, 1.0);
            L_target = flight_start_z_ - tuck_progress * (flight_start_z_ - L_RETRACT_);
        } else {
            // 下落期展腿
            double extend_progress = bbot_jump::clamp_value((elapsed - T_FLIGHT_APEX_) / T_FLIGHT_EXTEND_, 0.0, 1.0);
            L_target = L_RETRACT_ + extend_progress * (L_TOUCH_ - L_RETRACT_);
        }

        // 姿态超限提前展腿保护
        if (attitude_unstable) {
            L_target = L_TOUCH_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                 "[腾空保护] 姿态过大 (pitch=%.2f rad)，停止收腿并准备落地", pitch_);
        }

        // 保持力矩控制模式
        bool legs_deployed = protective_landing_ ||
                             elapsed >= (T_FLIGHT_APEX_ + T_FLIGHT_EXTEND_);
        if (!effort_mode_active_ && !leg_mode_switch_pending_) {
            request_effort_controller();
        }

        // 2. 空中关节轨迹跟踪 (Effort PD)
        bbot_kinematics::IKSolution ik_fl = kinematics_.inverse_kinematics(L_target, 0.0);
        if (effort_mode_active_) {
            publish_effort_leg_control(ik_fl.theta_hip, ik_fl.theta_knee, 0.0, 0.0,
                                       0.0, 0.0, 80.0, 6.0, 80.0, 6.0);
        } else {
            publish_position_leg_control(ik_fl.theta_hip, ik_fl.theta_knee);
        }

        // 3. 空中飞轮效应姿态控制 (动量轮反作用扭矩)
        double k_air_p = 0.45;
        double k_air_d = 0.35;
        double cmd_x = bbot_jump::clamp_value(-(k_air_p * pitch_err + k_air_d * pitch_rate_), -1.0, 1.0);
        publish_wheel_cmd(cmd_x, 0.0);

        log_data(cmd_x, 0.0, 0.0, 0.0);

        // 4. 触地检测逻辑 (腿压缩量 / 关节力矩突变 / IMU冲击)
        if (legs_deployed || elapsed >= T_FLIGHT_TIMEOUT_) {
            bool leg_compressed = legs_deployed && (current_z_ < (L_target - 0.010));

            if (legs_deployed && (std::abs(knee_effort_left_) > 15.0 || std::abs(knee_effort_right_) > 15.0)) {
                touchdown_knee_effort_count_++;
            } else {
                touchdown_knee_effort_count_ = 0;
            }
            bool torque_spike = (touchdown_knee_effort_count_ >= 2);
            bool imu_impact = (acc_z_filt_ > 15.0);

            if (leg_compressed || torque_spike || imu_impact || elapsed >= T_FLIGHT_TIMEOUT_) {
                RCLCPP_INFO(this->get_logger(), ">>> 触地检测触发 (t=%.3fs, z=%.3f)！进入缓冲阻抗控制", elapsed, current_z_);
                current_state_ = bbot_jump::STATE_TOUCHDOWN_BUFFER;
                state_start_time_ = now_sec;
                touchdown_buffer_initialized_ = false;
                touchdown_stable_count_ = 0;
                protective_landing_ = false;
                target_x_ = x_;
                was_moving_ = false;
                preload_touchdown_effort();
                request_effort_controller();
            }
        }
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
            buffer_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81; // 初始力设为静态重力
        }
        double elapsed = now_sec - state_start_time_;

        // 1. 任务空间阻抗计算
        double z_err = L_TOUCH_ - current_z_;
        double m_single_leg = TOTAL_MASS_ * 0.5;
        double F_z_target = K_Z_BUFFER_ * z_err - D_Z_BUFFER_ * current_z_dot_ +
                            (m_single_leg * 9.81);
        const double F_z_min = 0.25 * m_single_leg * 9.81; // 保持最小支撑力
        F_z_target = bbot_jump::clamp_value(F_z_target, F_z_min, F_Z_BUFFER_MAX_);
        // 限制支撑力变化率 (1200 N/s)
        const double max_force_step = 1200.0 * dt;
        buffer_force_per_leg_ += bbot_jump::clamp_value(
            F_z_target - buffer_force_per_leg_, -max_force_step, max_force_step);
        double F_z_base = buffer_force_per_leg_;

        // 2. 实际几何雅可比力矩映射
        double jh_left, jk_left, jh_right, jk_right;
        compute_leg_vertical_jacobian(pitch_, hip_pos_left_, knee_pos_left_, jh_left, jk_left);
        compute_leg_vertical_jacobian(pitch_, hip_pos_right_, knee_pos_right_, jh_right, jk_right);
        double tau_hip = F_z_base * 0.5 * (jh_left + jh_right);
        double tau_knee = F_z_base * 0.5 * (jk_left + jk_right);

        // 关节构型保持与吸能阻尼
        bbot_kinematics::IKSolution ik_buf = kinematics_.inverse_kinematics(L_TOUCH_, 0.0);
        double pitch_err = pitch_ - balance_offset_;
        double tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                          K_BODY_D_BUFFER_ * pitch_rate_);
        tau_body_per_hip = bbot_jump::clamp_value(tau_body_per_hip,
                                                   -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        tau_hip += tau_body_per_hip;
        publish_effort_leg_control(ik_buf.theta_hip, ik_buf.theta_knee, 0.0, 0.0,
                                   tau_hip, tau_knee,
                                   18.0, 3.0,  // 髋关节保持
                                   32.0, 4.5); // 膝关节保持

        // 3. 轮毂四状态 LQR 自平衡
        interpolate_lqr_gain();
        const double pos_error = x_ - target_x_;
        const double lqr_sum = current_gain_.k_x * pos_error +
                               current_gain_.k_x_dot * x_dot_ +
                               current_gain_.k_theta * pitch_err +
                               0.35 * current_gain_.k_theta_dot * pitch_rate_;
        double cmd_target = lqr_sum * cmd_scale_;
        const double buffer_cmd_limit = 1.0;
        cmd_target = bbot_jump::clamp_value(
            cmd_target, -buffer_cmd_limit, buffer_cmd_limit);
        const double buffer_cmd_step = 10.0 * dt;
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -buffer_cmd_step, buffer_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

        log_data(cmd_x, tau_hip, tau_knee, F_z_base * 2.0);

        // 4. 缓冲沉降稳定退出判断
        bool leg_settled = std::abs(current_z_dot_) < 0.08 &&
                           current_z_ > (L_MIN_ + 0.04) &&
                           current_z_ < (L_TOUCH_ + 0.040);
        bool body_settled = std::abs(pitch_err) < 0.10 &&
                            std::abs(pitch_rate_) < 0.80;
        bool wheels_settled = std::abs(x_dot_) < 0.30;
        bool settled_now = elapsed >= 0.25 && leg_settled &&
                           body_settled && wheels_settled;
        touchdown_stable_count_ = settled_now ? (touchdown_stable_count_ + 1) : 0;

        if (touchdown_stable_count_ >= 30) {
            RCLCPP_INFO(
                this->get_logger(),
                        ">>> 落地缓冲连续稳定 0.15s，进入阶段 5：恢复自平衡 (RECOVERY)... <<<");

            current_state_ = bbot_jump::STATE_RECOVERY;
            state_start_time_ = now_sec;
            recovery_x_ref_ = x_;
            target_x_ = x_;
            was_moving_ = false;
            recovery_stable_count_ = 0;
            post_landing_pitch_ref_ = balance_offset_;
            height_force_per_leg_ = buffer_force_per_leg_;
            height_force_initialized_ = true;
            target_height_ = L_STAND_;

            const double recovery_start_z = bbot_jump::clamp_value(
                current_z_, L_SQUAT_, L_TOUCH_);
            quintic_traj_.init(
                now_sec, 0.45, recovery_start_z, 0.0, 0.0,
                L_STAND_, 0.0, 0.0);
        } else if (elapsed >= 0.80) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[落地缓冲] 连续稳定计数 %d/30 (z=%.3f, v=%.2f, pitch=%.2f, gyro=%.2f, wheel_v=%.2f)",
                                 touchdown_stable_count_, current_z_, current_z_dot_,
                                 pitch_, pitch_rate_, x_dot_);
        }
    }

    // ── 阶段 5：平稳沉降与平衡恢复 (RECOVERY) ──
    void run_state_recovery(double now_sec)
    {
        // 落地承重后继续使用 Effort 控制。
        // 保持力矩控制模式
        request_effort_controller();
        double elapsed = now_sec - state_start_time_;
        double des_z, des_v, des_acc;
        quintic_traj_.evaluate(now_sec, des_z, des_v, des_acc);

        current_height_ = des_z;

        bbot_kinematics::IKSolution ik_rec = kinematics_.inverse_kinematics(des_z, 0.0);
        bbot_kinematics::JointTorques g_torques = kinematics_.compute_gravity_torques(0.0, ik_rec.theta_hip, ik_rec.theta_knee);
        double support_force_total = TOTAL_MASS_ * 9.81;
        if (effort_mode_active_) {
            // 任务空间高度阻抗控制
            support_force_total = publish_effort_height_control(
                des_z, des_v,
                320.0, 50.0, 170.0,
                18.0, 4.0, 30.0, 6.0,
                true);
        }
        // 静态俯仰平衡参考点平滑过渡
        constexpr double kLandingPitchEquilibrium = 0.085;
        if (elapsed > 0.60) {
            const double ref_step = 0.04 * 0.005;
            post_landing_pitch_ref_ += bbot_jump::clamp_value(
                kLandingPitchEquilibrium - post_landing_pitch_ref_, -ref_step, ref_step);
        }
        double pitch_err = pitch_ - post_landing_pitch_ref_;
        interpolate_lqr_gain();
        const double pos_error = x_ - target_x_;
        const double lqr_sum = current_gain_.k_x * pos_error +
                               current_gain_.k_x_dot * x_dot_ +
                               current_gain_.k_theta * pitch_err +
                               0.50 * current_gain_.k_theta_dot * pitch_rate_;
        double cmd_target = lqr_sum * cmd_scale_;
        const double cmd_limit = 1.0;
        cmd_target = bbot_jump::clamp_value(cmd_target, -cmd_limit, cmd_limit);
        const double max_cmd_step = 10.0 * 0.005;
        double cmd_x = last_wheel_cmd_x_ + bbot_jump::clamp_value(
            cmd_target - last_wheel_cmd_x_, -max_cmd_step, max_cmd_step);
        publish_wheel_cmd(cmd_x, 0.0);

        log_data(cmd_x, g_torques.hip_torque * 0.5, g_torques.knee_torque * 0.5,
                 support_force_total);

        // 恢复沉降与稳定退出判定
        bool trajectory_done = quintic_traj_.is_finished(now_sec);
        bool leg_recovered = std::abs(current_z_ - L_STAND_) < 0.050 &&
                             std::abs(current_z_dot_) < 0.08;
        bool body_recovered = std::abs(pitch_err) < 0.06 &&
                              std::abs(pitch_rate_) < 0.60;
        bool wheels_recovered = std::abs(x_dot_) < 0.25;
        bool stable_now = effort_mode_active_ && elapsed >= 0.45 && trajectory_done &&
                          leg_recovered && body_recovered && wheels_recovered;
        recovery_stable_count_ = stable_now ? (recovery_stable_count_ + 1) : 0;

        // 落地后保持 RECOVERY 状态开关 (为 true 时不自动切入 BALANCE)
        constexpr bool kHoldRecoveryAfterJump = true;
        if (recovery_stable_count_ >= 20) {
            if (kHoldRecoveryAfterJump) {
                recovery_stable_count_ = 20;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "[恢复保持] 落地已稳定，继续使用 RECOVERY 承重与低增益平衡；不切入 BALANCE");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "=====================================================");
            RCLCPP_INFO(
                this->get_logger(),
                ">>> 恢复连续稳定 0.10s，进入 BALANCE；方案 A 保持 Effort 控制，不再切回 Position <<<");
            RCLCPP_INFO(this->get_logger(), "=====================================================");

            current_state_ = bbot_jump::STATE_BALANCE;
            balance_entry_time_ = now_sec;

            // 轮毂平衡软接管配置
            post_landing_balance_soft_start_ = true;
            post_landing_gyro_reduced_ = true;

            // 保持力矩控制模式
            post_landing_effort_support_ = true;
            post_landing_position_handoff_requested_ = false;
            post_landing_position_settle_count_ = 0;
            position_switch_time_ = -1.0;
            post_landing_translation_feedback_ = true;

            target_height_ = L_STAND_;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;

            if (!height_force_initialized_) {
                height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
                height_force_initialized_ = true;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "[Effort连续接管 v6] 当前每腿支撑力状态=%.2f N，"
                "BALANCE 保持高度阻抗/Joint D/Body PD，1.0s 内 Joint P 从 18/30 平滑降到 14/24",
                height_force_per_leg_);
        } else if (elapsed >= 1.50) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[恢复等待] 连续稳定计数 %d/20 (z=%.3f, vz=%.2f, pitch=%.2f, gyro=%.2f, wheel_v=%.2f)",
                recovery_stable_count_, current_z_, current_z_dot_, pitch_err,
                pitch_rate_, x_dot_);
        }
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
        RCLCPP_ERROR(this->get_logger(),
                     "[跳跃保护] %s (z=%.3f m)，中止推地并保持力矩承重恢复",
                     reason, current_z_);
        current_state_ = bbot_jump::STATE_RECOVERY;
        state_start_time_ = now_sec;
        recovery_x_ref_ = x_;
        target_x_ = x_;
        was_moving_ = false;
        recovery_stable_count_ = 0;
        height_force_per_leg_ = TOTAL_MASS_ * 0.5 * 9.81;
        height_force_initialized_ = true;
        const double safe_start_height = bbot_jump::clamp_value(current_z_, L_SQUAT_, L_MAX_);
        current_height_ = safe_start_height;
        quintic_traj_.init(now_sec, 0.45, safe_start_height, 0.0, 0.0,
                           L_STAND_, 0.0, 0.0);
        target_height_ = L_STAND_;
        // THRUST 已确认处于 effort 模式；恢复环也以 effort 承重，不能在这里
        // 插入 Effort→Position→Effort 的异步往返切换。
    }

    void transition_to_protective_landing(double now_sec, const char * reason)
    {
        RCLCPP_WARN(this->get_logger(),
                    "[跳跃保护] %s (z=%.3f m)，保持展腿并等待触地，不提前进入恢复",
                    reason, current_z_);
        current_state_ = bbot_jump::STATE_FLIGHT;
        state_start_time_ = now_sec;
        flight_start_z_ = L_TOUCH_;
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
        const double f_hold = TOTAL_MASS_ * 0.5 * 9.81;
        double tau_hip = f_hold * 0.5 * (jh_left + jh_right);
        double tau_knee = f_hold * 0.5 * (jk_left + jk_right);
        const double pitch_err = pitch_ - balance_offset_;
        double tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                          K_BODY_D_BUFFER_ * pitch_rate_);
        tau_body_per_hip = bbot_jump::clamp_value(tau_body_per_hip,
                                                   -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
        tau_hip += tau_body_per_hip;

        const auto ik_touch = kinematics_.inverse_kinematics(L_TOUCH_, 0.0);
        publish_effort_leg_control(ik_touch.theta_hip, ik_touch.theta_knee, 0.0, 0.0,
                                   tau_hip, tau_knee, 22.0, 3.0, 38.0, 5.0);
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

        if (stabilize_body) {
            const bool use_landing_pitch_ref =
                (current_state_ == bbot_jump::STATE_RECOVERY) || post_landing_gyro_reduced_;
            const double pitch_ref = use_landing_pitch_ref ?
                post_landing_pitch_ref_ : balance_offset_;
            const double pitch_err = pitch_ - pitch_ref;
            double tau_body_per_hip = -0.5 * (K_BODY_P_BUFFER_ * pitch_err +
                                              K_BODY_D_BUFFER_ * pitch_rate_);
            tau_body_per_hip = bbot_jump::clamp_value(
                tau_body_per_hip, -TAU_HIP_BODY_MAX_, TAU_HIP_BODY_MAX_);
            tau_hip_l += tau_body_per_hip;
            tau_hip_r += tau_body_per_hip;
        }

        const auto ik = kinematics_.inverse_kinematics(z_des, 0.0);
        publish_effort_leg_control_lr(ik.theta_hip, ik.theta_knee, 0.0, 0.0,
                                      tau_hip_l, tau_knee_l,
                                      tau_hip_r, tau_knee_r,
                                      kp_hip, kd_hip, kp_knee, kd_knee);
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

    // ── 力矩阶段：支持左右腿独立前馈力矩 + 关节保持 PD ──
    void publish_effort_leg_control_lr(
        double q_hip_des, double q_knee_des,
        double q_dot_hip_des, double q_dot_knee_des,
        double tau_ff_hip_l, double tau_ff_knee_l,
        double tau_ff_hip_r, double tau_ff_knee_r,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee)
    {
        // 1. 左腿力位混控计算
        double tau_hip_left = tau_ff_hip_l + kp_hip * (q_hip_des - hip_pos_left_) + kd_hip * (q_dot_hip_des - hip_vel_left_);
        double tau_knee_left = tau_ff_knee_l + kp_knee * (q_knee_des - knee_pos_left_) + kd_knee * (q_dot_knee_des - knee_vel_left_);

        // 2. 右腿力位混控计算 (独立前馈 + 独立反馈)
        double tau_hip_right = tau_ff_hip_r + kp_hip * (q_hip_des - hip_pos_right_) + kd_hip * (q_dot_hip_des - hip_vel_right_);
        double tau_knee_right = tau_ff_knee_r + kp_knee * (q_knee_des - knee_pos_right_) + kd_knee * (q_dot_knee_des - knee_vel_right_);

        // 3. 电机物理力矩限幅保护 (髋 75Nm, 膝 60Nm)
        tau_hip_left = bbot_jump::clamp_value(tau_hip_left, -75.0, 75.0);
        tau_knee_left = bbot_jump::clamp_value(tau_knee_left, -60.0, 60.0);
        tau_hip_right = bbot_jump::clamp_value(tau_hip_right, -75.0, 75.0);
        tau_knee_right = bbot_jump::clamp_value(tau_knee_right, -60.0, 60.0);

        hip_cmd_left_ = tau_hip_left;
        knee_cmd_left_ = tau_knee_left;
        hip_cmd_right_ = tau_hip_right;
        knee_cmd_right_ = tau_knee_right;

        // 下发力矩命令；位置控制器在该阶段处于 inactive，绝不同时下发位置命令。
        std_msgs::msg::Float64MultiArray effort_cmd;
        effort_cmd.data = {tau_hip_left, tau_knee_left, tau_hip_right, tau_knee_right};
        leg_effort_pub_->publish(effort_cmd);
    }

    void publish_effort_leg_control(
        double q_hip_des, double q_knee_des,
        double q_dot_hip_des, double q_dot_knee_des,
        double tau_ff_hip, double tau_ff_knee,
        double kp_hip, double kd_hip,
        double kp_knee, double kd_knee)
    {
        publish_effort_leg_control_lr(
            q_hip_des, q_knee_des, q_dot_hip_des, q_dot_knee_des,
            tau_ff_hip, tau_ff_knee, tau_ff_hip, tau_ff_knee,
            kp_hip, kd_hip, kp_knee, kd_knee);
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
    void request_position_controller(bool preload_current_pose = false)
    {
        if (!effort_mode_active_ || leg_mode_switch_pending_) return;
        if (!switch_ctrl_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[控制器切换] switch_controller 服务尚未就绪");
            return;
        }
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        if (preload_current_pose) {
            // 构型预压偏置
            const double hip_preload  = 0.008;
            const double knee_preload = -0.040;
        
            const double hip_cmd_left =
                hip_pos_left_ + hip_preload;
        
            const double knee_cmd_left =
                knee_pos_left_ + knee_preload;
        
            const double hip_cmd_right =
                hip_pos_right_ + hip_preload;
        
            const double knee_cmd_right =
                knee_pos_right_ + knee_preload;
        
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
            publish_position_leg_control(ik_stand.theta_hip + 0.008, ik_stand.theta_knee - 0.040);
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

                    if (post_landing_position_handoff_requested_) {
                        post_landing_effort_support_ = false;
                        post_landing_position_hold_until_ =
                            position_switch_time_ + 0.50;
                        RCLCPP_INFO(
                            this->get_logger(),
                            "[落地交接] Position 已接管：持续发布左右独立目标，保持 0.50s 后慢速回到站立目标");
                    }

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
                } else {
                    RCLCPP_WARN(this->get_logger(), "[控制器切换] Effort → Position 切换失败！");
                }
            });
        RCLCPP_INFO(this->get_logger(), "[控制器切换] 请求 Effort → Position ...");
    }

    void open_log_files()
    {
        jump_log_file_.open(data_path_ + "jump_control_log.csv");
        if (jump_log_file_.is_open()) {
            jump_log_file_ << "timestamp,state,state_name,z,z_dot,pitch,pitch_rate,acc_z,"
                           << "cmd_x,x,x_dot,hip_pos_left,knee_pos_left,hip_pos_right,knee_pos_right,"
                           << "hip_effort_left,knee_effort_left,hip_cmd_left,knee_cmd_left,"
                           << "hip_cmd_right,knee_cmd_right,"
                           << "hip_pos_cmd_left,knee_pos_cmd_left,hip_pos_cmd_right,knee_pos_cmd_right,"
                           << "tau_ff_hip,tau_ff_knee,F_z\n";
        }
    }

    void close_log_files()
    {
        if (jump_log_file_.is_open()) {
            jump_log_file_.close();
        }
    }

    void log_data(double cmd_x, double tau_hip, double tau_knee, double f_z)
    {
        double t = (this->now() - start_time_).seconds();
        if (jump_log_file_.is_open()) {
            jump_log_file_ << t << ","
                           << static_cast<int>(current_state_) << ","
                           << bbot_jump::state_to_string(current_state_) << ","
                           << current_z_ << ","
                           << current_z_dot_ << ","
                           << pitch_ << ","
                           << pitch_rate_ << ","
                           << acc_z_filt_ << ","
                           << cmd_x << ","
                           << x_ << ","
                           << x_dot_ << ","
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
                           << hip_pos_cmd_left_ << ","
                           << knee_pos_cmd_left_ << ","
                           << hip_pos_cmd_right_ << ","
                           << knee_pos_cmd_right_ << ","
                           << tau_hip << ","
                           << tau_knee << ","
                           << f_z << "\n";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BBotJumpController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
