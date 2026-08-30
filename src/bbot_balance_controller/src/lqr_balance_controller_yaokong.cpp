#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "sensor_msgs/msg/joy.hpp" // 遥控器消息头文件 (暂时注释)

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "bbot_balance_controller/keyboard_reader.h"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

static double clamp_value(double value, double min_value, double max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

static double low_pass_filter(double new_value, double old_value, double alpha)
{
    return alpha * new_value + (1.0 - alpha) * old_value;
}

static double lerp(double a, double b, double ratio)
{
    return a + (b - a) * ratio;
}

struct LQRGain
{
    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;
};

class LQRBalanceController : public rclcpp::Node
{
public:
    LQRBalanceController()
        : Node("lqr_balance_controller")
    {
        gain_low_ = {-6.1624, -46.8436, -197.6985, -46.8109};
        gain_high_ = {-6.3650, -48.5719, -229.4004, -58.6391};
        current_gain_ = gain_high_;

        balance_offset_ = 0.034;
        cmd_scale_ = 0.05;
        cmd_sign_ = 1.0;
        wheel_radius_ = 0.07;
        max_cmd_x_ = 5.0;
        max_safe_pitch_ = 1.20;

        walk_speed_ = 0.3;
        turn_speed_ = 0.5;
        speed_ramp_time_ = 1.0;
        target_speed_smoothed_ = 0.0;

        L_MIN_ = 0.30;
        L_MAX_ = 0.50;
        
        target_height_ = L_MAX_;
        current_height_ = target_height_;
        
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0;

        const char * home_dir = getenv("HOME");
        data_path_ = std::string(home_dir ? home_dir : "/home/admin") + "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/";
        open_log_files();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&LQRBalanceController::imu_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&LQRBalanceController::joint_state_callback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
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
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                target_height_ = clamp_value(msg->data, L_MIN_, L_MAX_);
            });

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "standup" || msg->data == "r" || msg->data == "R")
                {
                    control_mode_ = MODE_STANDUP;
                    target_speed_const_ = 0.0;
                    target_yaw_rate_ = 0.0;
                    target_x_ = x_;
                    was_moving_ = false;
                    vel_integral_ = 0.0;
                }
                else if (msg->data == "emergency" || msg->data == "x" || msg->data == "X")
                {
                    control_mode_ = MODE_EMERGENCY;
                    target_speed_const_ = 0.0;
                    target_yaw_rate_ = 0.0;
                }
                else if (msg->data == "balance")
                {
                    control_mode_ = MODE_BALANCE;
                }
            });

        // 暂时注释遥控器订阅，切换为纯键盘控制
        // joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "/rc_input", 10, std::bind(&LQRBalanceController::joy_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);

        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        timer_ = this->create_wall_timer(5ms, std::bind(&LQRBalanceController::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "LQRBalanceController started in KEYBOARD mode.");
        print_key_help();
    }

    ~LQRBalanceController()
    {
        close_log_files();
    }

private:
    enum RobotControlMode
    {
        MODE_STANDUP,
        MODE_BALANCE,
        MODE_EMERGENCY
    };
    RobotControlMode control_mode_ = MODE_BALANCE;

    void print_key_help()
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  LQR 控制器键盘控制说明:");
        RCLCPP_INFO(this->get_logger(), "  W     — 前进 (%.2f m/s)", walk_speed_);
        RCLCPP_INFO(this->get_logger(), "  S     — 后退 (%.2f m/s)", walk_speed_);
        RCLCPP_INFO(this->get_logger(), "  A     — 左转 (%.2f rad/s)", turn_speed_);
        RCLCPP_INFO(this->get_logger(), "  D     — 右转 (%.2f rad/s)", turn_speed_);
        RCLCPP_INFO(this->get_logger(), "  Space — 停止移动保持平衡");
        RCLCPP_INFO(this->get_logger(), "  R     — 触发倒地自恢复起立");
        RCLCPP_INFO(this->get_logger(), "  Q     — 升高身体 (+1cm,  %.2f~%.2fm)", L_MIN_, L_MAX_);
        RCLCPP_INFO(this->get_logger(), "  E     — 降低身体 (-1cm,  %.2f~%.2fm)", L_MIN_, L_MAX_);
        RCLCPP_INFO(this->get_logger(), "  X     — 紧急停机");
        RCLCPP_INFO(this->get_logger(), "============================================");
    }

    void process_keyboard()
    {
        std::string seq = keyboard_.read_sequence();
        if (seq.empty())
            return;

        if (seq == "w" || seq == "W")
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
            RCLCPP_INFO(this->get_logger(), "[键盘] 停止移动");
        }
        else if (seq == "q" || seq == "Q")
        {
            target_height_ = clamp_value(target_height_ + 0.01, L_MIN_, L_MAX_);
            RCLCPP_INFO(this->get_logger(), "[键盘] 升高  目标高度 → %.3f m", target_height_);
        }
        else if (seq == "e" || seq == "E")
        {
            target_height_ = clamp_value(target_height_ - 0.01, L_MIN_, L_MAX_);
            RCLCPP_INFO(this->get_logger(), "[键盘] 降低  目标高度 → %.3f m", target_height_);
        }
        else if (seq == "r" || seq == "R")
        {
            control_mode_ = MODE_STANDUP;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 触发自适应起立恢复模式！");
        }
        else if (seq == "x" || seq == "X")
        {
            control_mode_ = MODE_EMERGENCY;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "[键盘] 紧急停机！");
        }
    }

    /*
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 3 || msg->buttons.size() < 3)
            return;

        double ly_roll = msg->axes[0];
        double lx_pitch = msg->axes[1];
        double aux1_axis = msg->axes[2];
        int aux2_safe_ch = msg->buttons[2];

        if (aux2_safe_ch == 1)
        {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            return;
        }

        if (std::abs(lx_pitch) > 0.05)
        {
            target_speed_const_ = lx_pitch * walk_speed_;
        }
        else
        {
            target_speed_const_ = 0.0;
        }

        if (std::abs(ly_roll) > 0.05)
        {
            target_yaw_rate_ = -ly_roll * turn_speed_;
        }
        else
        {
            target_yaw_rate_ = 0.0;
        }

        double height_ratio = (aux1_axis + 1.0) / 2.0;
        target_height_ = L_MIN_ + height_ratio * (L_MAX_ - L_MIN_);
    }
    */

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 轮轴沿 X 轴，前后俯仰对应绕 X 轴旋转 (roll)，前倾 (+Y) 对应负 roll，取负号使前倾为正
        pitch_ = -roll;
        pitch_rate_raw_ = -msg->angular_velocity.x;

        if (!pitch_rate_filter_init_)
        {
            pitch_rate_filt_ = pitch_rate_raw_;
            pitch_rate_filter_init_ = true;
        }
        else
        {
            pitch_rate_filt_ = low_pass_filter(pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
        }
        pitch_rate_ = pitch_rate_filt_;
        imu_received_ = true;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        bool has_left = false, has_right = false;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "link_004_joint")
            {
                if (i < msg->position.size())
                    left_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size())
                    left_wheel_vel_ = msg->velocity[i];
                has_left = true;
            }
            else if (msg->name[i] == "link_007_joint")
            {
                if (i < msg->position.size())
                    right_wheel_pos_ = msg->position[i];
                if (i < msg->velocity.size())
                    right_wheel_vel_ = msg->velocity[i];
                has_right = true;
            }
            else if (msg->name[i] == "link_002_joint")
            {
                if (i < msg->effort.size())
                    hip_effort_left_ = msg->effort[i];
            }
            else if (msg->name[i] == "link_003_joint")
            {
                if (i < msg->effort.size())
                    knee_effort_left_ = msg->effort[i];
            }
        }

        if (has_left && has_right)
        {
            if (!wheel_origin_set_)
            {
                left_wheel_pos_origin_ = left_wheel_pos_;
                right_wheel_pos_origin_ = right_wheel_pos_;
                wheel_origin_set_ = true;
            }
            // 轮子负转角对应物理前向 (+Y) 滚动
            x_dot_raw_ = -wheel_radius_ * 0.5 * (left_wheel_vel_ + right_wheel_vel_);
            if (!x_dot_filter_init_)
            {
                x_dot_filt_ = x_dot_raw_;
                x_dot_filter_init_ = true;
            }
            else
            {
                x_dot_filt_ = low_pass_filter(x_dot_raw_, x_dot_filt_, x_dot_alpha_);
            }
            x_dot_ = x_dot_filt_;

            double left_delta = left_wheel_pos_ - left_wheel_pos_origin_;
            double right_delta = right_wheel_pos_ - right_wheel_pos_origin_;
            x_ = -wheel_radius_ * 0.5 * (left_delta + right_delta);
        }
    }

    void control_loop()
    {
        publish_leg_pose();
    
        
        if (!imu_received_ || !wheel_origin_set_)
            return;

        // 处理键盘输入
        process_keyboard();

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0001)
            dt = 0.005;

        update_leg_height_by_dt(dt);
        publish_leg_pose();
        interpolate_lqr_gain();

        if (control_mode_ == MODE_EMERGENCY)
        {
            publish_cmd(0.0, 0.0);
            return;
        }

        double pitch_err = pitch_ - balance_offset_;

        // 状态机：自适应起立恢复与 LQR 平衡切换
        if (control_mode_ == MODE_STANDUP)
        {
            // 当机身接近平衡点且角速度平稳时，平滑切入 LQR 平衡模式
            if (std::abs(pitch_err) < 0.18 && std::abs(pitch_rate_) < 1.5)
            {
                control_mode_ = MODE_BALANCE;
                target_x_ = x_;
                was_moving_ = false;
                vel_integral_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "[平衡控制器] 机身摆起成功，切入 LQR 自平衡！");
            }
            else
            {
                // 倒地冲量恢复控制：车轮向前/向后加速回正
                double standup_vel = 0.0;
                if (pitch_err < -0.15)
                {
                    standup_vel = 2.5;
                }
                else if (pitch_err > 0.15)
                {
                    standup_vel = -2.5;
                }
                publish_cmd(standup_vel, 0.0);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 200,
                    "[起立恢复] pitch=%.3f pitch_rate=%.2f cmd=%.2f (冲量拉起中...)",
                    pitch_, pitch_rate_, standup_vel);
                return;
            }
        }
        else if (control_mode_ == MODE_BALANCE)
        {
            // 若失衡过大（> 45度），自动退回起立恢复模式
            if (std::abs(pitch_err) > 0.80)
            {
                control_mode_ = MODE_STANDUP;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[平衡控制器] 倾角失衡，进入自恢复起立模式...");
                return;
            }
        }

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

        double pos_error = x_ - target_x_;
        double vel_error = x_dot_ - target_speed;
        double gyro_val = pitch_rate_;
        double dynamic_target_pitch = balance_offset_;
        double u_pitch = 0.0;
        double cmd_x = 0.0;

        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            double theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_x * pos_error +
                        current_gain_.k_x_dot * vel_error +
                        current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
            cmd_x = -u_pitch * cmd_scale_;
            vel_integral_ = 0.0;
        }
        else
        {
            double vel_error_v = target_speed - x_dot_;
            vel_integral_ += vel_error_v * dt;
            vel_integral_ = clamp_value(vel_integral_, -0.5, 0.5);

            double kp_v = 0.25;
            double ki_v = 0.05;

            dynamic_target_pitch = balance_offset_ + (kp_v * vel_error_v + ki_v * vel_integral_);
            dynamic_target_pitch = clamp_value(dynamic_target_pitch, -0.2, 0.2);

            double theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
            cmd_x = -u_pitch * cmd_scale_ - target_speed;
        }

        cmd_x = clamp_value(cmd_x, -max_cmd_x_, max_cmd_x_);
        double cmd_yaw = target_yaw_rate_;

        publish_cmd(cmd_x, cmd_yaw);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "[LQR] x=%.3f x_dot=%.2f pitch=%.3f u=%.2f cmd=%.2f L_leg=%.3f Kθ=%.1f",
            x_, x_dot_, pitch_, u_pitch, cmd_x, current_height_, current_gain_.k_theta);

        log_data(target_speed, u_pitch, cmd_x, dynamic_target_pitch, 0.0);
    }

    double compute_height_ratio()
    {
        return clamp_value((current_height_ - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
    }

    void interpolate_lqr_gain()
    {
        double ratio = compute_height_ratio();
        current_gain_.k_x = lerp(gain_low_.k_x, gain_high_.k_x, ratio);
        current_gain_.k_x_dot = lerp(gain_low_.k_x_dot, gain_high_.k_x_dot, ratio);
        current_gain_.k_theta = lerp(gain_low_.k_theta, gain_high_.k_theta, ratio);
        current_gain_.k_theta_dot = lerp(gain_low_.k_theta_dot, gain_high_.k_theta_dot, ratio);
    }

    void publish_cmd(double linear_x, double angular_z)
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = linear_x;
        cmd.twist.angular.z = angular_z;
        cmd_pub_->publish(cmd);
    }

    void publish_leg_pose()
    {
        bbot_kinematics::IKSolution ik = kinematics_.inverse_kinematics(current_height_, 0.0);
        double hip = ik.theta_hip;
        double knee = ik.theta_knee;

        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip, knee, hip, knee};
        leg_pub_->publish(leg_cmd);

        if (ht_log_file_.is_open())
        {
            bbot_kinematics::JointTorques torques = kinematics_.compute_gravity_torques(0.0, ik.theta_hip, ik.theta_knee);
            double t = (this->now() - start_time_).seconds();
            ht_log_file_ << t << "," << current_height_ << "," << hip << "," << knee << ","
                         << torques.hip_torque / 2.0 << "," << torques.knee_torque / 2.0 << ","
                         << hip_effort_left_ << "," << knee_effort_left_ << "\n";
        }
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
        current_height_ = clamp_value(current_height_, L_MIN_, L_MAX_);
    }

    void open_log_files()
    {
        lqr_log_file_.open(data_path_ + "lqr_control_log.csv");
        if (lqr_log_file_.is_open())
        {
            lqr_log_file_ << "timestamp,x,x_dot,pitch,pitch_rate,u_pitch,cmd_x,"
                          << "target_speed,height,k_x,k_x_dot,k_theta,k_theta_dot,"
                          << "eff_target_pitch,eff_target_pitch_rate\n";
        }

        angle_file_.open(data_path_ + "angle_data.txt");
        target_angle_file_.open(data_path_ + "target_angle_data.txt");
        angle_time_file_.open(data_path_ + "timestamp_angle.txt");
        target_angle_time_file_.open(data_path_ + "timestamp_target_angle.txt");
        speed_file_.open(data_path_ + "speed_data.txt");
        target_speed_file_.open(data_path_ + "target_speed_data.txt");
        speed_time_file_.open(data_path_ + "timestamp_speed.txt");
        target_speed_time_file_.open(data_path_ + "timestamp_target_speed.txt");
        gyro_file_.open(data_path_ + "gyro_data.txt");
        target_gyro_file_.open(data_path_ + "target_gyro_data.txt");
        gyro_time_file_.open(data_path_ + "timestamp_gyro.txt");
        target_gyro_time_file_.open(data_path_ + "timestamp_target_gyro.txt");

        ht_log_file_.open(data_path_ + "height_torque_log.csv");
        if (ht_log_file_.is_open())
        {
            ht_log_file_ << "timestamp,com_height,hip_angle_rad,knee_angle_rad,hip_torque,knee_torque,hip_effort,knee_effort\n";
        }
    }

    void close_log_files()
    {
        if (lqr_log_file_.is_open())
            lqr_log_file_.close();
        angle_file_.close();
        target_angle_file_.close();
        angle_time_file_.close();
        target_angle_time_file_.close();
        speed_file_.close();
        target_speed_file_.close();
        speed_time_file_.close();
        target_speed_time_file_.close();
        gyro_file_.close();
        target_gyro_file_.close();
        gyro_time_file_.close();
        target_gyro_time_file_.close();
        if (ht_log_file_.is_open())
            ht_log_file_.close();
    }

    void log_data(double target_speed, double u_pitch, double cmd_x, double real_target_pitch, double real_target_pitch_rate)
    {
        double t = (this->now() - start_time_).seconds();
        if (lqr_log_file_.is_open())
        {
            lqr_log_file_ << t << "," << x_ << "," << x_dot_ << "," << pitch_ << "," << pitch_rate_ << ","
                          << u_pitch << "," << cmd_x << "," << target_speed << "," << current_height_ << ","
                          << current_gain_.k_x << "," << current_gain_.k_x_dot << "," << current_gain_.k_theta << "," << current_gain_.k_theta_dot << ","
                          << real_target_pitch << "," << real_target_pitch_rate << "\n";
        }

        if (angle_file_.is_open())
        {
            angle_file_ << pitch_ << '\n';
            target_angle_file_ << real_target_pitch << '\n';
            angle_time_file_ << t << '\n';
            target_angle_time_file_ << t << '\n';
        }
        if (speed_file_.is_open())
        {
            speed_file_ << x_dot_ << '\n';
            target_speed_file_ << target_speed << '\n';
            speed_time_file_ << t << '\n';
            target_speed_time_file_ << t << '\n';
        }
        if (gyro_file_.is_open())
        {
            gyro_file_ << pitch_rate_ << '\n';
            target_gyro_file_ << real_target_pitch_rate << '\n';
            gyro_time_file_ << t << '\n';
            target_gyro_time_file_ << t << '\n';
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; // 暂时注释遥控器订阅
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    KeyboardReader keyboard_; // 键盘输入读取器

    bool imu_received_ = false;
    bool wheel_origin_set_ = false;
    bool was_moving_ = false;

    bbot_kinematics::Kinematics kinematics_;
    rclcpp::Time last_time_;
    rclcpp::Time start_time_;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double x_ = 0.0;
    double x_dot_ = 0.0;

    double left_wheel_pos_ = 0.0, right_wheel_pos_ = 0.0;
    double left_wheel_vel_ = 0.0, right_wheel_vel_ = 0.0;
    double left_wheel_pos_origin_ = 0.0, right_wheel_pos_origin_ = 0.0;

    double pitch_rate_raw_ = 0.0, pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.10;

    double x_dot_raw_ = 0.0, x_dot_filt_ = 0.0;
    bool x_dot_filter_init_ = false;
    double x_dot_alpha_ = 0.05;

    LQRGain gain_low_;
    LQRGain gain_high_;
    LQRGain current_gain_;

    double balance_offset_;
    double cmd_scale_;
    double cmd_sign_;
    double wheel_radius_;
    double max_cmd_x_;
    double max_safe_pitch_;

    double target_speed_const_ = 0.0;
    double target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_;
    double turn_speed_;
    double speed_ramp_time_;
    double target_x_ = 0.0;

    double current_height_ = 0.5490;
    double target_height_ = 0.5490;
    double leg_transition_speed_;
    double L_MIN_;
    double L_MAX_;

    std::string data_path_;
    std::ofstream lqr_log_file_;
    std::ofstream angle_file_, target_angle_file_, angle_time_file_, target_angle_time_file_;
    std::ofstream speed_file_, target_speed_file_, speed_time_file_, target_speed_time_file_;
    std::ofstream gyro_file_, target_gyro_file_, gyro_time_file_, target_gyro_time_file_;
    std::ofstream ht_log_file_;

    double hip_effort_left_ = 0.0;
    double knee_effort_left_ = 0.0;
    double vel_integral_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LQRBalanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
