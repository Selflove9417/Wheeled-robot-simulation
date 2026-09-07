#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "bbot_balance_controller/keyboard_reader.h"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

// ============================================================
// Utility
// ============================================================

static double clamp_value(
    double value,
    double min_value,
    double max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

static double lerp(
    double a,
    double b,
    double ratio)
{
    return a + (b - a) * ratio;
}

static double low_pass_filter(
    double new_value,
    double old_value,
    double alpha)
{
    return alpha * new_value + (1.0 - alpha) * old_value;
}

// ============================================================
// Gain scheduling data
// ============================================================

struct GainPoint
{
    double height;

    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;

    // Nominal equilibrium pitch:
    //
    // theta_eq = -theta_COM
    //
    // IMU convention:
    // forward lean = positive pitch
    double theta_eq;
};

struct LQRGain
{
    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;
};

// ============================================================
// Controller
// ============================================================

class LQRGainScheduledController : public rclcpp::Node
{
public:
    LQRGainScheduledController()
        : Node("lqr_gain_scheduled_controller")
    {
        // ====================================================
        // Robot parameters
        // ====================================================

        const auto &robot_params = kinematics_.get_params();

        wheel_radius_ = robot_params.wheel_radius;

        L_MIN_ = robot_params.L_MIN;
        L_MAX_ = robot_params.L_MAX;

        wheel_torque_max_ = robot_params.wheel_torque_max;

        // The current GS-LQR table was generated only for:
        //
        // H = 0.30 ... 0.50 m
        //
        // Therefore RobotParams must match this model.
        if (std::abs(L_MIN_ - 0.30) > 1e-6 ||
            std::abs(L_MAX_ - 0.50) > 1e-6)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "RobotParams height range is [%.3f, %.3f], "
                "but current GS-LQR model was designed for [0.30, 0.50] m.",
                L_MIN_,
                L_MAX_);
        }

        // ====================================================
        // Initial height
        // ====================================================

        target_height_ = L_MAX_;
        current_height_ = target_height_;

        // 0.20 m range / 4 s = 0.05 m/s
        leg_transition_speed_ =
            (L_MAX_ - L_MIN_) / 4.0;

        // ====================================================
        // Filters
        // ====================================================

        pitch_rate_alpha_ = 0.10;
        x_dot_alpha_ = 0.05;

        // ====================================================
        // Safety
        // ====================================================

        // LQR model input u is TOTAL wheel torque.
        //
        // Each wheel maximum torque = 10 Nm
        // therefore total available torque = 20 Nm.
        total_torque_max_ =
            2.0 * wheel_torque_max_;

        // If pitch error exceeds this value,
        // stop LQR output.
        max_pitch_error_ = 0.50;

        // ====================================================
        // Publishers
        // ====================================================

        wheel_effort_pub_ =
            this->create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/wheel_effort_controller/commands",
                10);

        leg_pub_ =
            this->create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/leg_position_controller/commands",
                10);

        // ====================================================
        // Subscribers
        // ====================================================

        imu_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Imu>(
                "/imu",
                10,
                std::bind(
                    &LQRGainScheduledController::imu_callback,
                    this,
                    std::placeholders::_1));

        joint_state_sub_ =
            this->create_subscription<
                sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &LQRGainScheduledController::joint_state_callback,
                    this,
                    std::placeholders::_1));

        target_height_sub_ =
            this->create_subscription<
                std_msgs::msg::Float64>(
                "/target_height",
                10,
                [this](
                    const std_msgs::msg::Float64::SharedPtr msg)
                {
                    target_height_ =
                        clamp_value(
                            msg->data,
                            L_MIN_,
                            L_MAX_);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[GS-LQR] target height = %.3f m",
                        target_height_);
                });

        // ====================================================
        // Timer
        // ====================================================

        timer_ =
            this->create_wall_timer(
                5ms,
                std::bind(
                    &LQRGainScheduledController::control_loop,
                    this));

        last_time_ = this->now();
        start_time_ = this->now();

        // ====================================================
        // Logging
        // ====================================================

        const char *home_dir = getenv("HOME");

        data_path_ =
            std::string(
                home_dir ? home_dir : "/home/admin") +
            "/bbot_ws_new/src/"
            "bbot_balance_controller/src/data_logs/";

        open_log_file();

        // ====================================================
        // Info
        // ====================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==============================================");

        RCLCPP_INFO(
            this->get_logger(),
            "Torque-input Gain-Scheduled LQR started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Height range: %.2f ~ %.2f m",
            L_MIN_,
            L_MAX_);

        RCLCPP_INFO(
            this->get_logger(),
            "Per-wheel torque limit: %.2f Nm",
            wheel_torque_max_);

        RCLCPP_INFO(
            this->get_logger(),
            "Control rate: 200 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "==============================================");

        print_key_help();
    }

    ~LQRGainScheduledController()
    {
        publish_zero_torque();

        if (log_file_.is_open())
            log_file_.close();
    }

private:
    // ========================================================
    // GS-LQR table
    // ========================================================

    //
    // Gains generated from LQR_K_new.m
    //
    // State:
    //
    // X =
    // [
    //   x - x_ref
    //   x_dot
    //   pitch - theta_eq(H)
    //   pitch_rate
    // ]
    //
    // Model:
    //
    // u_model = -K(H) X
    //
    //
    // theta_eq:
    //
    // From URDF/CAD suspended-body COM geometry:
    //
    // theta_eq = -atan2(y_COM, z_COM)
    //
    //

    static constexpr std::array<GainPoint, 5>
        gain_table_{{{0.30,
                      -5.622712,
                      -42.666506,
                      -156.508986,
                      -35.846746,
                      +0.118440},

                     {0.35,
                      -5.791278,
                      -43.976682,
                      -169.544125,
                      -39.563240,
                      +0.102967},

                     {0.40,
                      -5.931603,
                      -45.087129,
                      -181.972969,
                      -43.390225,
                      +0.087823},

                     {0.45,
                      -6.052411,
                      -46.061241,
                      -193.948669,
                      -47.349665,
                      +0.074136},

                     {0.50,
                      -6.157159,
                      -46.922551,
                      -205.518217,
                      -51.430573,
                      +0.061787}}};

    enum class GainMode
    {
        FIXED,
        SCHEDULED
    };

    GainMode gain_mode_ = GainMode::SCHEDULED;

    const LQRGain fixed_gain_{
        -5.931603,
        -45.087129,
        -181.972969,
        -43.390225};

    // ========================================================
    // Keyboard
    // ========================================================

    void print_key_help()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "F     : Fixed-LQR, use K(H=0.40 m)");

        RCLCPP_INFO(
            this->get_logger(),
            "G     : Gain-Scheduled LQR, use K(H)");

        RCLCPP_INFO(
            this->get_logger(),
            "----------------------------------------------");

        RCLCPP_INFO(
            this->get_logger(),
            "GS-LQR keyboard:");

        RCLCPP_INFO(
            this->get_logger(),
            "Q     : raise body +0.01 m");

        RCLCPP_INFO(
            this->get_logger(),
            "E     : lower body -0.01 m");

        RCLCPP_INFO(
            this->get_logger(),
            "Space : reset position reference");

        RCLCPP_INFO(
            this->get_logger(),
            "X     : emergency stop");

        RCLCPP_INFO(
            this->get_logger(),
            "B     : resume balance control");

        RCLCPP_INFO(
            this->get_logger(),
            "----------------------------------------------");
    }

    void process_keyboard()
    {
        std::string seq = keyboard_.read_sequence();

        if (seq.empty())
            return;

        if (seq == "q" || seq == "Q")
        {
            target_height_ =
                clamp_value(
                    target_height_ + 0.01,
                    L_MIN_,
                    L_MAX_);

            RCLCPP_INFO(
                this->get_logger(),
                "[GS-LQR] target height -> %.3f m",
                target_height_);
        }

        else if (seq == "e" || seq == "E")
        {
            target_height_ =
                clamp_value(
                    target_height_ - 0.01,
                    L_MIN_,
                    L_MAX_);

            RCLCPP_INFO(
                this->get_logger(),
                "[GS-LQR] target height -> %.3f m",
                target_height_);
        }

        else if (seq == " ")
        {
            target_x_ = x_;

            RCLCPP_INFO(
                this->get_logger(),
                "[GS-LQR] position reference reset: x_ref=%.3f",
                target_x_);
        }

        else if (seq == "x" || seq == "X")
        {
            control_enabled_ = false;

            publish_zero_torque();

            RCLCPP_WARN(
                this->get_logger(),
                "[GS-LQR] EMERGENCY STOP");
        }

        else if (seq == "b" || seq == "B")
        {
            target_x_ = x_;

            control_enabled_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "[GS-LQR] balance resumed, x_ref=%.3f",
                target_x_);
        }

        else if (seq == "f" || seq == "F")
        {
            gain_mode_ = GainMode::FIXED;
            target_x_ = x_;

            RCLCPP_INFO(
                this->get_logger(),
                "[LQR MODE] Fixed-LQR: K = K(0.40 m)");
        }

        else if (seq == "g" || seq == "G")
        {
            gain_mode_ = GainMode::SCHEDULED;
            target_x_ = x_;

            RCLCPP_INFO(
                this->get_logger(),
                "[LQR MODE] Gain-Scheduled LQR: K = K(H)");
        }
    }

    // ========================================================
    // IMU
    // ========================================================

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        double roll;
        double pitch_unused;
        double yaw;

        tf2::Matrix3x3(q).getRPY(
            roll,
            pitch_unused,
            yaw);

        //
        // Current robot convention:
        //
        // physical forward lean -> negative roll
        //
        // therefore:
        //
        // pitch > 0  means forward lean
        //

        pitch_ = -roll;

        pitch_rate_raw_ =
            -msg->angular_velocity.x;

        if (!pitch_rate_filter_init_)
        {
            pitch_rate_filt_ =
                pitch_rate_raw_;

            pitch_rate_filter_init_ = true;
        }
        else
        {
            pitch_rate_filt_ =
                low_pass_filter(
                    pitch_rate_raw_,
                    pitch_rate_filt_,
                    pitch_rate_alpha_);
        }

        pitch_rate_ =
            pitch_rate_filt_;

        imu_received_ = true;
    }

    // ========================================================
    // Wheel state
    // ========================================================

    void joint_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        bool has_004 = false;
        bool has_007 = false;

        for (size_t i = 0;
             i < msg->name.size();
             ++i)
        {
            if (msg->name[i] == "link_004_joint")
            {
                if (i < msg->position.size())
                    wheel_004_pos_ =
                        msg->position[i];

                if (i < msg->velocity.size())
                    wheel_004_vel_ =
                        msg->velocity[i];

                has_004 = true;
            }

            else if (
                msg->name[i] == "link_007_joint")
            {
                if (i < msg->position.size())
                    wheel_007_pos_ =
                        msg->position[i];

                if (i < msg->velocity.size())
                    wheel_007_vel_ =
                        msg->velocity[i];

                has_007 = true;
            }
        }

        if (!(has_004 && has_007))
            return;

        // ----------------------------------------------------
        // Establish wheel position origin
        // ----------------------------------------------------

        if (!wheel_origin_set_)
        {
            wheel_004_origin_ =
                wheel_004_pos_;

            wheel_007_origin_ =
                wheel_007_pos_;

            wheel_origin_set_ = true;

            target_x_ = 0.0;
        }

        // ----------------------------------------------------
        // Robot forward direction:
        //
        // wheel negative rotation = robot forward
        //
        // therefore:
        //
        // x_dot = -r * average(wheel speed)
        // ----------------------------------------------------

        x_dot_raw_ =
            -wheel_radius_ * 0.5 * (wheel_004_vel_ + wheel_007_vel_);

        if (!x_dot_filter_init_)
        {
            x_dot_filt_ =
                x_dot_raw_;

            x_dot_filter_init_ = true;
        }
        else
        {
            x_dot_filt_ =
                low_pass_filter(
                    x_dot_raw_,
                    x_dot_filt_,
                    x_dot_alpha_);
        }

        x_dot_ =
            x_dot_filt_;

        const double delta_004 =
            wheel_004_pos_ - wheel_004_origin_;

        const double delta_007 =
            wheel_007_pos_ - wheel_007_origin_;

        x_ =
            -wheel_radius_ * 0.5 * (delta_004 + delta_007);
    }

    // ========================================================
    // Gain interpolation
    // ========================================================

    void interpolate_schedule(
        double height,
        LQRGain &gain,
        double &theta_eq)
    {
        // Below minimum
        if (height <= gain_table_.front().height)
        {
            const auto &p =
                gain_table_.front();

            gain = {
                p.k_x,
                p.k_x_dot,
                p.k_theta,
                p.k_theta_dot};

            theta_eq = p.theta_eq;

            return;
        }

        // Above maximum
        if (height >= gain_table_.back().height)
        {
            const auto &p =
                gain_table_.back();

            gain = {
                p.k_x,
                p.k_x_dot,
                p.k_theta,
                p.k_theta_dot};

            theta_eq = p.theta_eq;

            return;
        }

        // Piecewise-linear interpolation
        for (size_t i = 0;
             i < gain_table_.size() - 1;
             ++i)
        {
            const auto &p0 =
                gain_table_[i];

            const auto &p1 =
                gain_table_[i + 1];

            if (height >= p0.height &&
                height <= p1.height)
            {
                const double ratio =
                    (height - p0.height) /
                    (p1.height - p0.height);

                gain.k_x =
                    lerp(
                        p0.k_x,
                        p1.k_x,
                        ratio);

                gain.k_x_dot =
                    lerp(
                        p0.k_x_dot,
                        p1.k_x_dot,
                        ratio);

                gain.k_theta =
                    lerp(
                        p0.k_theta,
                        p1.k_theta,
                        ratio);

                gain.k_theta_dot =
                    lerp(
                        p0.k_theta_dot,
                        p1.k_theta_dot,
                        ratio);

                theta_eq =
                    lerp(
                        p0.theta_eq,
                        p1.theta_eq,
                        ratio);

                return;
            }
        }
    }

    // ========================================================
    // Leg height
    // ========================================================

    void update_leg_height(double dt)
    {
        const double step =
            leg_transition_speed_ * dt;

        if (current_height_ <
            target_height_)
        {
            current_height_ += step;

            if (current_height_ >
                target_height_)
            {
                current_height_ =
                    target_height_;
            }
        }

        else if (
            current_height_ >
            target_height_)
        {
            current_height_ -= step;

            if (current_height_ <
                target_height_)
            {
                current_height_ =
                    target_height_;
            }
        }

        current_height_ =
            clamp_value(
                current_height_,
                L_MIN_,
                L_MAX_);
    }

    void publish_leg_pose()
    {
        const auto ik =
            kinematics_.inverse_kinematics(
                current_height_,
                0.0);

        std_msgs::msg::Float64MultiArray msg;

        // Existing leg_position_controller order:
        //
        // left hip
        // left knee
        // right hip
        // right knee

        msg.data =
            {
                ik.theta_hip,
                ik.theta_knee,
                ik.theta_hip,
                ik.theta_knee};

        leg_pub_->publish(msg);
    }

    // ========================================================
    // Wheel torque command
    // ========================================================

    void publish_wheel_torque(
        double tau_each)
    {
        std_msgs::msg::Float64MultiArray msg;

        //
        // IMPORTANT:
        //
        // Experimental verification:
        //
        // command [ +0.5, +0.5 ]
        // -> robot moves backward
        //
        // command [ -0.5, -0.5 ]
        // -> robot moves forward
        //
        //
        // Straight balancing uses equal torque,
        // therefore channel order does not change
        // the balance dynamics.
        //
        // Your physical test also showed:
        //
        // data[0] = physical right wheel
        // data[1] = physical left wheel
        //

        msg.data =
            {
                tau_each,
                tau_each};

        wheel_effort_pub_->publish(msg);
    }

    void publish_zero_torque()
    {
        publish_wheel_torque(0.0);
    }

    // ========================================================
    // Main control loop
    // ========================================================

    void control_loop()
    {
        // ----------------------------------------------------
        // Keyboard
        // ----------------------------------------------------

        process_keyboard();

        // ----------------------------------------------------
        // dt
        // ----------------------------------------------------

        rclcpp::Time now =
            this->now();

        double dt =
            (now - last_time_).seconds();

        last_time_ = now;

        if (dt <= 0.0001 ||
            dt > 0.05)
        {
            dt = 0.005;
        }

        // ----------------------------------------------------
        // Height update
        // ----------------------------------------------------

        update_leg_height(dt);

        publish_leg_pose();

        // ----------------------------------------------------
        // Sensor readiness
        // ----------------------------------------------------

        if (!imu_received_ ||
            !wheel_origin_set_)
        {
            publish_zero_torque();

            return;
        }

        // ----------------------------------------------------
        // Emergency state
        // ----------------------------------------------------

        if (!control_enabled_)
        {
            publish_zero_torque();

            return;
        }

        // ----------------------------------------------------
        // Gain scheduling
        // ----------------------------------------------------

        LQRGain scheduled_gain;

        interpolate_schedule(
            current_height_,
            scheduled_gain,
            theta_eq_nominal_);

        if (gain_mode_ == GainMode::SCHEDULED)
        {
            current_gain_ = scheduled_gain;
        }
        else
        {
            current_gain_ = fixed_gain_;
        }
        // ----------------------------------------------------
        // LQR states
        // ----------------------------------------------------

        const double x_error =
            x_ - target_x_;

        const double theta_error =
            pitch_ - theta_eq_nominal_;

        // ----------------------------------------------------
        // Safety
        // ----------------------------------------------------

        if (std::abs(theta_error) >
            max_pitch_error_)
        {
            control_enabled_ = false;

            publish_zero_torque();

            RCLCPP_ERROR(
                this->get_logger(),
                "[GS-LQR] pitch error %.3f rad exceeds "
                "safety limit %.3f rad. Controller disabled.",
                theta_error,
                max_pitch_error_);

            return;
        }

        // ----------------------------------------------------
        // LQR
        //
        // X =
        //
        // [ x_error
        //   x_dot
        //   theta_error
        //   pitch_rate ]
        //
        //
        // MATLAB:
        //
        // u_model = -K X
        //
        // ----------------------------------------------------

        const double feedback_sum =
            current_gain_.k_x * x_error

            + current_gain_.k_x_dot * x_dot_

            + current_gain_.k_theta * theta_error

            + current_gain_.k_theta_dot * pitch_rate_;

        const double u_raw =
            -feedback_sum;

        // ----------------------------------------------------
        // Total model torque saturation
        // ----------------------------------------------------

        const double u_model =
            clamp_value(
                u_raw,
                -total_torque_max_,
                total_torque_max_);

        // ----------------------------------------------------
        // Model torque -> Gazebo effort
        //
        // Model:
        //
        // +u  => forward
        //
        // Experiment:
        //
        // -effort => forward
        //
        // therefore:
        //
        // tau_cmd_each = -u / 2
        // ----------------------------------------------------

        double tau_each =
            -0.5 * u_model;

        tau_each =
            clamp_value(
                tau_each,
                -wheel_torque_max_,
                wheel_torque_max_);

        publish_wheel_torque(
            tau_each);

        const char *mode_name =
            (gain_mode_ == GainMode::SCHEDULED)
                ? "GS"
                : "FIXED";
        // ----------------------------------------------------
        // Diagnostics
        // ----------------------------------------------------

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            250,

            "[LQR] "
            "mode=%s "
            "H=%.3f "
            "x=%.3f "
            "xdot=%.3f "
            "pitch=%.4f "
            "eq=%.4f "
            "err=%.4f "
            "u=%.2fNm "
            "tau=%.2fNm "
            "Ktheta=%.2f",

            mode_name,
            current_height_,
            x_,
            x_dot_,
            pitch_,
            theta_eq_nominal_,
            theta_error,
            u_model,
            tau_each,
            current_gain_.k_theta);

        log_data(
            x_error,
            theta_error,
            u_raw,
            u_model,
            tau_each);
    }

    // ========================================================
    // Logging
    // ========================================================

    void open_log_file()
    {
        log_file_.open(
            data_path_ + "gs_lqr_torque_log.csv");

        if (!log_file_.is_open())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to open GS-LQR log file.");

            return;
        }

        log_file_
            << "time,"
            << "gain_mode,"
            << "height,"
            << "x,"
            << "x_ref,"
            << "x_error,"
            << "x_dot,"
            << "pitch,"
            << "pitch_rate,"
            << "theta_eq,"
            << "theta_error,"
            << "k_x,"
            << "k_x_dot,"
            << "k_theta,"
            << "k_theta_dot,"
            << "u_raw,"
            << "u_model,"
            << "tau_each"
            << "\n";
    }

    void log_data(
        double x_error,
        double theta_error,
        double u_raw,
        double u_model,
        double tau_each)
    {
        if (!log_file_.is_open())
            return;

        const double t =
            (this->now() -
             start_time_)
                .seconds();

        const int gain_mode_id =
            (gain_mode_ == GainMode::SCHEDULED) ? 1 : 0;

        log_file_
            << t << ","
            << gain_mode_id << ","
            << current_height_ << ","
            << x_ << ","
            << target_x_ << ","
            << x_error << ","
            << x_dot_ << ","
            << pitch_ << ","
            << pitch_rate_ << ","
            << theta_eq_nominal_ << ","
            << theta_error << ","
            << current_gain_.k_x << ","
            << current_gain_.k_x_dot << ","
            << current_gain_.k_theta << ","
            << current_gain_.k_theta_dot << ","
            << u_raw << ","
            << u_model << ","
            << tau_each
            << "\n";
    }

    // ========================================================
    // ROS interfaces
    // ========================================================

    rclcpp::Subscription<
        sensor_msgs::msg::Imu>::SharedPtr
        imu_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Float64>::SharedPtr
        target_height_sub_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        wheel_effort_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        leg_pub_;

    rclcpp::TimerBase::SharedPtr
        timer_;

    // ========================================================
    // Robot / controller objects
    // ========================================================

    bbot_kinematics::Kinematics
        kinematics_;

    KeyboardReader
        keyboard_;

    // ========================================================
    // Timing
    // ========================================================

    rclcpp::Time
        last_time_;

    rclcpp::Time
        start_time_;

    // ========================================================
    // Sensor state
    // ========================================================

    bool imu_received_ = false;

    bool wheel_origin_set_ = false;

    bool control_enabled_ = true;

    double pitch_ = 0.0;

    double pitch_rate_ = 0.0;

    double pitch_rate_raw_ = 0.0;

    double pitch_rate_filt_ = 0.0;

    bool pitch_rate_filter_init_ = false;

    double pitch_rate_alpha_ = 0.10;

    // ========================================================
    // Wheel state
    // ========================================================

    double wheel_004_pos_ = 0.0;

    double wheel_007_pos_ = 0.0;

    double wheel_004_vel_ = 0.0;

    double wheel_007_vel_ = 0.0;

    double wheel_004_origin_ = 0.0;

    double wheel_007_origin_ = 0.0;

    double x_ = 0.0;

    double x_dot_ = 0.0;

    double x_dot_raw_ = 0.0;

    double x_dot_filt_ = 0.0;

    bool x_dot_filter_init_ = false;

    double x_dot_alpha_ = 0.05;

    // ========================================================
    // References
    // ========================================================

    double target_x_ = 0.0;

    double current_height_ = 0.0;

    double target_height_ = 0.0;

    double leg_transition_speed_ = 0.0;

    // ========================================================
    // Gain scheduling
    // ========================================================

    LQRGain current_gain_{
        0.0,
        0.0,
        0.0,
        0.0};

    double theta_eq_nominal_ = 0.0;

    // ========================================================
    // Parameters
    // ========================================================

    double wheel_radius_ = 0.07;

    double wheel_torque_max_ = 10.0;

    double total_torque_max_ = 20.0;

    double L_MIN_ = 0.30;

    double L_MAX_ = 0.50;

    double max_pitch_error_ = 0.50;

    // ========================================================
    // Logging
    // ========================================================

    std::string data_path_;

    std::ofstream log_file_;
};

// ============================================================
// Static table definition
// ============================================================

constexpr std::array<GainPoint, 5>
    LQRGainScheduledController::gain_table_;

// ============================================================
// main
// ============================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);

    auto node =
        std::make_shared<
            LQRGainScheduledController>();

    rclcpp::spin(
        node);

    rclcpp::shutdown();

    return 0;
}