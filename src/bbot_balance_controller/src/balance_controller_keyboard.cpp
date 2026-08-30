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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "bbot_balance_controller/pid.h"
#include "bbot_balance_controller/keyboard_reader.h"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

// 限幅函数
static double clamp_value(double value, double min_value, double max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}
// 低通滤波器
static double low_pass_filter(double new_value, double old_value, double alpha)
{
    return alpha * new_value + (1.0 - alpha) * old_value;
}
// 线性插值函数
static double lerp(double a, double b, double ratio)
{
    return a + (b - a) * ratio;
}

struct PIDParam
{
    float p;
    float i;
    float d;
    float ramp;
    float limit;
};

class BalanceControllerKeyboard : public rclcpp::Node
{
public:
    BalanceControllerKeyboard()
        : Node("balance_controller_keyboard"),
          pid_speed_(0.50f, 0.003f, 0.025f, 0.0f, 0.50f),
          pid_angle_(6.0f, 0.0f, 0.12f, 0.0f, 1.5f),
          pid_gyro_(2.2f, 0.0f, 0.012f, 8.0f, 5.0f)
    {
        // ── 平衡与控制参数 ──
        balance_offset_ = 0.034; // 真实 CAD 垂直水平平衡倾角 (0.0 rad)
        cmd_sign_ = 1.0;

        k_position_ = 0.3;
        max_target_speed_ = 0.3;

        wheel_radius_ = 0.07;
        max_cmd_x_ = 5.0;
        max_safe_pitch_ = 1.20;

        // 键盘控制时的行走速度与转向速度
        walk_speed_ = 0.3;
        turn_speed_ = 0.5; // rad/s

        // 目标速度 ramp 时间（秒）：目标速度变化经过此时间才完全生效
        speed_ramp_time_ = 1.0;
        target_speed_smoothed_ = 0.0;

        // ── 腿部参数  ──
        L_MIN_ = 0.30; // 最低质心高度 (蹲下)
        L_MAX_ = 0.50; // 最高质心高度 (站立)

        // 起立 PID
        speed_pid_stand_ = {0.55f, 0.005f, 0.025f, 0.0f, 0.50f};
        angle_pid_stand_ = {6.0f, 0.0f, 0.12f, 0.0f, 2.5f};
        gyro_pid_stand_ = {2.2f, 0.0f, 0.012f, 10.0f, 5.0f};

        // 蹲下 PID
        speed_pid_squat_ = {0.45f, 0.004f, 0.020f, 0.0f, 0.45f};
        angle_pid_squat_ = {5.0f, 0.0f, 0.10f, 0.0f, 2.2f};
        gyro_pid_squat_ = {1.8f, 0.0f, 0.010f, 10.0f, 4.5f};

        // 初始高度设置为站立高度
        target_height_ = L_MAX_;                         // 初始目标
        current_height_ = target_height_;                        // 初始高度
  
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0; // 4秒完成全程过渡

        // ── 数据日志 ──
        const char * home_dir = getenv("HOME");
        data_path_ = std::string(home_dir ? home_dir : "/home/admin") + "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/";
        open_log_files();

        // ── ROS 接口 ──
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&BalanceControllerKeyboard::imu_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&BalanceControllerKeyboard::joint_state_callback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                target_speed_const_ = clamp_value(msg->linear.x, -max_target_speed_, max_target_speed_);
                target_yaw_rate_ = msg->angular.z;
                if (std::abs(msg->linear.x) < 0.001 && std::abs(msg->angular.z) < 0.001)
                {
                    target_speed_const_ = 0.0;
                    target_yaw_rate_ = 0.0;
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
                    pid_speed_.reset();
                    pid_angle_.reset();
                    pid_gyro_.reset();
                }
                else if (msg->data == "emergency" || msg->data == "x" || msg->data == "X")
                {
                    control_mode_ = MODE_EMERGENCY;
                    target_speed_const_ = 0.0;
                    target_yaw_rate_ = 0.0;
                    pid_speed_.reset();
                    pid_angle_.reset();
                    pid_gyro_.reset();
                }
                else if (msg->data == "balance")
                {
                    control_mode_ = MODE_BALANCE;
                }
            });

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);

        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        timer_ = this->create_wall_timer(
            5ms, std::bind(&BalanceControllerKeyboard::control_loop, this));

        last_time_ = this->now();
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "BalanceControllerKeyboard started.");
        print_key_help();
    }

    ~BalanceControllerKeyboard()
    {
        close_log_files();
    }

private:
    // 传感器回调
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

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
            pitch_rate_filt_ = low_pass_filter(
                pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
        }

        pitch_rate_ = pitch_rate_filt_;
        imu_received_ = true;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        bool has_left = false;
        bool has_right = false;

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
            // 读取腿部关节实际力矩
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
                x_dot_filt_ = low_pass_filter(
                    x_dot_raw_, x_dot_filt_, x_dot_alpha_);
            }

            x_dot_ = x_dot_filt_;

            double left_delta = left_wheel_pos_ - left_wheel_pos_origin_;
            double right_delta = right_wheel_pos_ - right_wheel_pos_origin_;
            x_ = -wheel_radius_ * 0.5 * (left_delta + right_delta);
        }
    }

    // 键盘处理

    enum RobotControlMode
    {
        MODE_STANDUP,
        MODE_BALANCE,
        MODE_EMERGENCY
    };
    RobotControlMode control_mode_ = MODE_STANDUP;

    void print_key_help()
    {
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  键盘控制说明:");
        RCLCPP_INFO(this->get_logger(), "  W     — 前进 (%.2f m/s)", walk_speed_);
        RCLCPP_INFO(this->get_logger(), "  S     — 后退 (%.2f m/s)", walk_speed_);
        RCLCPP_INFO(this->get_logger(), "  A     — 左转 (%.2f rad/s)", turn_speed_);
        RCLCPP_INFO(this->get_logger(), "  D     — 右转 (%.2f rad/s)", turn_speed_);
        RCLCPP_INFO(this->get_logger(), "  Space — 停止移动保持平衡");
        RCLCPP_INFO(this->get_logger(), "  R     — 触发倒地自恢复起立");
        RCLCPP_INFO(this->get_logger(), "  Q     — 升高身体 (+1cm,  %.2f~%.2fm)", L_MIN_, L_MAX_);
        RCLCPP_INFO(this->get_logger(), "  E     — 降低身体 (-1cm,  %.2f~%.2fm)", L_MIN_, L_MAX_);
        RCLCPP_INFO(this->get_logger(), "  X     — 紧急停机 (停止+复位PID)");
        RCLCPP_INFO(this->get_logger(), "============================================");
    }

    void process_keyboard()
    {
        std::string seq = keyboard_.read_sequence();

        if (seq.empty())
            return;

        // ── 移动控制 ──
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
        // ── 停止 ──
        else if (seq == " ")
        {
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[键盘] 停止移动");
        }
        // ── 高度控制（增量式，每次按 Q/E 调整 1cm）──
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
            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();
            RCLCPP_INFO(this->get_logger(), "[键盘] 触发自适应起立恢复模式！");
        }
        // ── 紧急停机 ──
        else if (seq == "x" || seq == "X")
        {
            control_mode_ = MODE_EMERGENCY;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();
            RCLCPP_WARN(this->get_logger(), "[键盘] 停机！ PID 已复位");
        }
    }

    // 主控制循环

    void control_loop()
    {
        // 确保启动时就发站立腿姿
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
        update_pid_by_leg_height();

        if (control_mode_ == MODE_EMERGENCY)
        {
            publish_cmd(0.0, 0.0);
            return;
        }

        double pitch_err = pitch_ - balance_offset_;

        // 状态机：自适应起立恢复与 PID 平衡切换
        if (control_mode_ == MODE_STANDUP)
        {
            if (std::abs(pitch_err) < 0.18 && std::abs(pitch_rate_) < 1.5)
            {
                control_mode_ = MODE_BALANCE;
                pid_speed_.reset();
                pid_angle_.reset();
                pid_gyro_.reset();
                RCLCPP_INFO(this->get_logger(), "[平衡控制器] 机身摆起成功，切入 PID 自平衡！");
            }
            else
            {
                // 倒地冲量恢复控制：利用车轮反向加速的惯性力矩将机身拉起
                double standup_vel = 0.0;
                if (pitch_err < -0.15)
                {
                    standup_vel = 2.5; // 向后倒：车轮后冲产生前倾力矩
                }
                else if (pitch_err > 0.15)
                {
                    standup_vel = -2.5;  // 向前倒：车轮前冲产生后倾力矩
                }
                publish_cmd(standup_vel, 0.0);
                return;
            }
        }
        else if (control_mode_ == MODE_BALANCE)
        {
            if (std::abs(pitch_err) > 0.80)
            {
                control_mode_ = MODE_STANDUP;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[平衡控制器] 倾角失衡，进入自恢复起立模式...");
                return;
            }
        }

        // 1. 目标速度 ramp：避免阶跃冲击，平滑过渡到键盘设定的目标速度
        double target_speed_step = dt / speed_ramp_time_; // 每周期的最大速度变化量
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
        target_speed = clamp_value(target_speed, -max_target_speed_, max_target_speed_);

        // 2. 速度环
        double velocity_error = target_speed - x_dot_;

        double target_pitch = pid_speed_(velocity_error, dt);
        target_pitch = clamp_value(target_pitch, -1.5, 1.5);
        // 目标倾角滤波
        target_pitch_filtered_ = 0.65 * target_pitch_filtered_ + 0.35 * target_pitch;
        target_pitch = target_pitch_filtered_ + balance_offset_;

        // 3. 角度环
        double angle_error = target_pitch - pitch_;
        double target_pitch_rate = pid_angle_(angle_error, dt);
        target_pitch_rate = clamp_value(target_pitch_rate, -3.2, 3.2);

        // 4. 角速度环
        double gyro_error = target_pitch_rate - pitch_rate_;
        double cmd_raw = pid_gyro_(gyro_error, dt);

        double cmd_x = cmd_raw * cmd_sign_;
        cmd_x = clamp_value(cmd_x, -max_cmd_x_, max_cmd_x_);

        // 5. 转向命令
        double cmd_yaw = target_yaw_rate_;

        publish_cmd(cmd_x, cmd_yaw);

        log_data(target_speed, target_pitch, target_pitch_rate);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "x=%.3f x_dot=%.3f spd=%.3f pitch=%.3f yaw=%.2f cmd=%.3f gyro_err=%.4f cmd_raw=%.4f h=%.3f",
            x_, x_dot_, target_speed, pitch_, cmd_yaw, cmd_x, gyro_error, cmd_raw, current_height_);
            
    }

    // 发布函数

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

        // 计算理论关节力矩
        bbot_kinematics::JointTorques torques = kinematics_.compute_gravity_torques(
            0.0, ik.theta_hip, ik.theta_knee);

        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip, knee, hip, knee};
        leg_pub_->publish(leg_cmd);

        // 记录高度-力矩数据到 CSV
        if (ht_log_file_.is_open())
        {
            double t = (this->now() - start_time_).seconds();
            ht_log_file_ << t << ","
                         << current_height_ << ","
                         << hip << "," << knee << ","
                         << torques.hip_torque / 2.0 << "," << torques.knee_torque / 2.0 << ","
                         << hip_effort_left_ << "," << knee_effort_left_ << "\n";
        }
    }

    // 腿高过渡

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

    // PID 插值

    PIDParam interpolate_pid(const PIDParam &squat, const PIDParam &stand, double ratio)
    {
        PIDParam out;
        out.p = static_cast<float>(lerp(squat.p, stand.p, ratio));
        out.i = static_cast<float>(lerp(squat.i, stand.i, ratio));
        out.d = static_cast<float>(lerp(squat.d, stand.d, ratio));
        out.ramp = static_cast<float>(lerp(squat.ramp, stand.ramp, ratio));
        out.limit = static_cast<float>(lerp(squat.limit, stand.limit, ratio));
        return out;
    }

    void apply_pid_param(PIDController &pid, const PIDParam &param)
    {
        pid.P = param.p;
        pid.I = param.i;
        pid.D = param.d;
        pid.output_ramp = param.ramp;
        pid.limit = param.limit;
    }

    double compute_height_ratio()
    {
        return clamp_value((current_height_ - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
    }

    void update_pid_by_leg_height()
    {
        double ratio = compute_height_ratio();

        apply_pid_param(pid_speed_, interpolate_pid(speed_pid_squat_, speed_pid_stand_, ratio));
        apply_pid_param(pid_angle_, interpolate_pid(angle_pid_squat_, angle_pid_stand_, ratio));
        apply_pid_param(pid_gyro_, interpolate_pid(gyro_pid_squat_, gyro_pid_stand_, ratio));
    }

    // 数据日志

    void open_log_files()
    {
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
        ht_log_file_ << "timestamp,com_height,hip_angle_rad,knee_angle_rad,"
                     << "hip_torque,knee_torque,hip_effort,knee_effort\n";

        if (!angle_file_.is_open() || !speed_file_.is_open() || !gyro_file_.is_open())
            RCLCPP_WARN(this->get_logger(), "Some log files failed to open. Check data_path.");
        else
            RCLCPP_INFO(this->get_logger(), "Log files opened at: %s", data_path_.c_str());
    }

    void close_log_files()
    {
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

    void log_data(double target_speed, double target_pitch, double target_pitch_rate)
    {
        double t = (this->now() - start_time_).seconds();

        if (angle_file_.is_open())
        {
            angle_file_ << pitch_ << '\n';
            target_angle_file_ << target_pitch << '\n';
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
            target_gyro_file_ << target_pitch_rate << '\n';
            gyro_time_file_ << t << '\n';
            target_gyro_time_file_ << t << '\n';
        }
    }

    // 成员变量

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PIDController pid_speed_;
    PIDController pid_angle_;
    PIDController pid_gyro_;

    KeyboardReader keyboard_;

    rclcpp::Time last_time_;
    rclcpp::Time start_time_;

    bool imu_received_ = false;
    bool wheel_origin_set_ = false;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double x_ = 0.0;
    double x_dot_ = 0.0;

    double left_wheel_pos_ = 0.0;
    double right_wheel_pos_ = 0.0;
    double left_wheel_vel_ = 0.0;
    double right_wheel_vel_ = 0.0;
    double left_wheel_pos_origin_ = 0.0;
    double right_wheel_pos_origin_ = 0.0;

    double hip_effort_left_ = 0.0;  //
    double knee_effort_left_ = 0.0; //

    double balance_offset_;
    double cmd_sign_;
    double wheel_radius_;
    double max_cmd_x_;
    double max_safe_pitch_;
    double k_position_;
    double max_target_speed_;
    double target_pitch_filtered_ = 0.0;

    // 键盘控制的目标值
    double target_speed_const_ = 0.0;
    double target_speed_smoothed_ = 0.0; // ramp 平滑后的实际目标速度
    double target_yaw_rate_ = 0.0;
    double walk_speed_;
    double turn_speed_;
    double speed_ramp_time_; // 目标速度 ramp 时间（秒）

    double target_x_ = 0.0;

    double current_height_ = 0.5490; // 当前质心高度 [m]
    double target_height_ = 0.5490;  // 目标质心高度 [m]
    double leg_transition_speed_;    // 高度变化速率 [m/s]
    double L_MIN_;                   // 最低高度 [m]
    double L_MAX_;                   // 最高高度 [m]

    bbot_kinematics::Kinematics kinematics_; // 运动学求解器

    PIDParam speed_pid_stand_, angle_pid_stand_, gyro_pid_stand_;
    PIDParam speed_pid_squat_, angle_pid_squat_, gyro_pid_squat_;

    double pitch_rate_raw_ = 0.0;
    double pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.20;

    double x_dot_raw_ = 0.0;
    double x_dot_filt_ = 0.0;
    bool x_dot_filter_init_ = false;
    double x_dot_alpha_ = 0.05;

    std::string data_path_;

    std::ofstream angle_file_, target_angle_file_, angle_time_file_, target_angle_time_file_;
    std::ofstream speed_file_, target_speed_file_, speed_time_file_, target_speed_time_file_;
    std::ofstream gyro_file_, target_gyro_file_, gyro_time_file_, target_gyro_time_file_;
    std::ofstream ht_log_file_; // 高度-力矩数据日志
};

//  main

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalanceControllerKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
