#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "bbot_balance_controller/adaptive_equilibrium_estimator.hpp"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

namespace
{

  double clamp_value(double value, double minimum, double maximum)
  {
    return std::max(minimum, std::min(value, maximum));
  }

  double lerp(double a, double b, double ratio)
  {
    return a + (b - a) * ratio;
  }

  double low_pass_filter(double input, double previous, double alpha)
  {
    return alpha * input + (1.0 - alpha) * previous;
  }

  struct GainPoint
  {
    double height;
    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;
    double y_com;
    double z_com;
  };

  struct LQRGain
  {
    double k_x{0.0};
    double k_x_dot{0.0};
    double k_theta{0.0};
    double k_theta_dot{0.0};
  };

  enum class ExperimentMode
  {
    Nominal = 0,
    Oracle = 1,
    Adaptive = 2
  };

  ExperimentMode parse_experiment_mode(const std::string &mode)
  {
    if (mode == "nominal")
    {
      return ExperimentMode::Nominal;
    }
    if (mode == "oracle")
    {
      return ExperimentMode::Oracle;
    }
    if (mode == "adaptive")
    {
      return ExperimentMode::Adaptive;
    }
    throw std::invalid_argument(
        "experiment.mode must be nominal, oracle, or adaptive");
  }

  const char *experiment_mode_name(ExperimentMode mode)
  {
    switch (mode)
    {
    case ExperimentMode::Nominal:
      return "nominal";
    case ExperimentMode::Oracle:
      return "oracle";
    case ExperimentMode::Adaptive:
      return "adaptive";
    }
    return "invalid";
  }

} // namespace

class AdaptiveLQRBalanceController : public rclcpp::Node
{
public:
  AdaptiveLQRBalanceController()
      : Node("adaptive_lqr_balance_controller"),
        estimator_(read_estimator_config())
  {
    const auto &robot = kinematics_.get_params();
    wheel_radius_ = robot.wheel_radius;
    wheel_torque_max_ = robot.wheel_torque_max;
    total_torque_max_ = 2.0 * wheel_torque_max_;

    // Match the real-robot interface: height means the vertical distance from
    // the wheel axle to the hip joint.  The simulation IK still expects the
    // base_link height above ground, so that coordinate conversion remains an
    // internal implementation detail.
    height_min_ = declare_parameter<double>("height.hip_axle_min", 0.30);
    height_max_ = declare_parameter<double>("height.hip_axle_max", 0.50);
    base_to_hip_height_ = declare_parameter<double>("height.base_to_hip", 0.07);
    if (!std::isfinite(height_min_) || !std::isfinite(height_max_) ||
        !std::isfinite(base_to_hip_height_) || height_min_ >= height_max_ ||
        base_to_hip_height_ < 0.0)
    {
      throw std::invalid_argument("invalid hip-axle height mapping parameters");
    }

    target_height_ = declare_parameter<double>("target_height", height_max_);
    target_height_ = clamp_value(target_height_, height_min_, height_max_);
    startup_height_ = declare_parameter<double>("height.startup_hip_axle", 0.36);
    if (!std::isfinite(startup_height_))
    {
      throw std::invalid_argument("height.startup_hip_axle must be finite");
    }
    startup_height_ = clamp_value(startup_height_, height_min_, height_max_);
    current_height_ = startup_height_;
    previous_height_ = current_height_;
    startup_hold_time_ = declare_parameter<double>("height.startup_hold_time", 2.0);
    if (!std::isfinite(startup_hold_time_) || startup_hold_time_ < 0.0)
    {
      throw std::invalid_argument("height.startup_hold_time must be finite and non-negative");
    }
    leg_transition_speed_ = declare_parameter<double>("leg_transition_speed", 0.05);
    max_pitch_error_ = declare_parameter<double>("max_pitch_error", 0.50);
    adaptation_enabled_ = declare_parameter<bool>("adaptation_enabled", true);
    experiment_mode_ = parse_experiment_mode(
        declare_parameter<std::string>("experiment.mode", "adaptive"));
    injected_com_y_bias_ = declare_parameter<double>("experiment.com_y_bias", 0.0);
    if (!std::isfinite(injected_com_y_bias_) || std::abs(injected_com_y_bias_) > 0.010)
    {
      throw std::invalid_argument(
          "experiment.com_y_bias must be finite and within +/-0.010 m");
    }
    log_enabled_ = declare_parameter<bool>("log_enabled", true);

    wheel_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wheel_effort_controller/commands", 10);
    leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/leg_position_controller/commands", 10);
    offset_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/adaptive_lqr/equivalent_com_offset", 10);
    equilibrium_pitch_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/adaptive_lqr/equilibrium_pitch", 10);
    gate_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/adaptive_lqr/gate_open", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&AdaptiveLQRBalanceController::imu_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(
            &AdaptiveLQRBalanceController::joint_state_callback, this,
            std::placeholders::_1));
    target_height_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/target_height", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg)
        {
          target_height_ = clamp_value(msg->data, height_min_, height_max_);
        });
    adaptive_command_sub_ = create_subscription<std_msgs::msg::String>(
        "/adaptive_lqr/command", 10,
        std::bind(
            &AdaptiveLQRBalanceController::adaptive_command_callback, this,
            std::placeholders::_1));
    robot_mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_mode", 10,
        std::bind(
            &AdaptiveLQRBalanceController::robot_mode_callback, this,
            std::placeholders::_1));

    if (log_enabled_)
    {
      open_log_file();
    }

    last_time_ = now();
    start_time_ = now();
    timer_ = create_wall_timer(
        5ms, std::bind(&AdaptiveLQRBalanceController::control_loop, this));

    RCLCPP_INFO(
        get_logger(),
        "Adaptive GS-LQR started: mode=%s, injected COM-y bias=%+.3f mm, "
        "hip-axle range=[%.3f, %.3f] m, startup=%.3f m, target=%.3f m, "
        "startup hold=%.1f s, base_link offset=%.3f m, inner loop 200 Hz.",
        experiment_mode_name(experiment_mode_), 1000.0 * injected_com_y_bias_,
        height_min_, height_max_, startup_height_, target_height_, startup_hold_time_,
        base_link_height_offset());
  }

  ~AdaptiveLQRBalanceController() override
  {
    publish_wheel_torque(0.0);
    if (log_file_.is_open())
    {
      log_file_.close();
    }
  }

private:
  static constexpr std::array<GainPoint, 5> gain_table_{{// height is H_hip-axle.  The corresponding base_link heights used by the
                                                         // URDF/MATLAB model are 0.44, 0.49, 0.54, 0.59 and 0.64 m.
                                                         {0.3000, -6.029616, -45.875982, -191.586929, -46.547541,
                                                          -0.0276264, 0.3592154},
                                                         {0.3500, -6.137382, -46.758575, -203.235631, -50.605491,
                                                          -0.0258066, 0.4016316},
                                                         {0.4000, -6.230876, -47.540094, -214.496292, -54.771166,
                                                          -0.0234056, 0.4443432},
                                                         {0.4500, -6.311959, -48.232783, -225.386480, -59.027574,
                                                          -0.0203401, 0.4872441},
                                                         {0.5000, -6.382279, -48.847665, -235.922994, -63.359510,
                                                          -0.0164269, 0.5302655}}};

  bbot_balance_controller::AdaptiveEquilibriumConfig read_estimator_config()
  {
    bbot_balance_controller::AdaptiveEquilibriumConfig config;
    config.two_stage_enabled =
        declare_parameter<bool>("adaptation.two_stage_enabled", true);
    config.early_capture_time =
        declare_parameter<double>("adaptation.early_capture_time", 1.0);
    config.early_observation_window_range_max = declare_parameter<double>(
        "adaptation.early_observation_window_range_max", 0.00010);
    config.early_speed_max =
        declare_parameter<double>("adaptation.early_speed_max", 0.080);
    config.early_accel_threshold =
        declare_parameter<double>("adaptation.early_accel_threshold", 0.015);
    config.early_pitch_rate_threshold =
        declare_parameter<double>("adaptation.early_pitch_rate_threshold", 0.010);

    config.capture_time =
        declare_parameter<double>("adaptation.capture_time", 1.0);
    config.observation_window_range_max = declare_parameter<double>(
        "adaptation.observation_window_range_max", 0.00010);
    config.apply_rate_max =
        declare_parameter<double>("adaptation.apply_rate_max", 0.0010);
    config.target_tolerance =
        declare_parameter<double>("adaptation.target_tolerance", 0.00001);
    config.verify_time =
        declare_parameter<double>("adaptation.verify_time", 0.50);
    config.speed_safety_max =
        declare_parameter<double>("adaptation.speed_safety_max", 0.015);
    config.accel_threshold =
        declare_parameter<double>("adaptation.accel_threshold", 0.005);
    config.accel_filter_time_constant = declare_parameter<double>(
        "adaptation.accel_filter_time_constant", 0.10);
    config.fast_pitch_rate_threshold = declare_parameter<double>(
        "adaptation.fast_pitch_rate_threshold", 0.01);
    config.fast_torque_threshold =
        declare_parameter<double>("adaptation.fast_torque_threshold", 0.05);
    config.reacquire_threshold =
        declare_parameter<double>("adaptation.reacquire_threshold", 0.00005);

    // Legacy progressive-update parameters remain declared for compatibility,
    // but the fast-capture state machine no longer uses them.
    config.averaging_time = declare_parameter<double>("adaptation.averaging_time", 2.0);
    config.cooldown_time = declare_parameter<double>("adaptation.cooldown_time", 4.0);
    config.filter_time_constant =
        declare_parameter<double>("adaptation.filter_time_constant", 0.50);
    config.position_error_deadband =
        declare_parameter<double>("adaptation.position_error_deadband", 0.005);
    config.observation_deadband =
        declare_parameter<double>("adaptation.observation_deadband", 0.00005);
    config.adaptation_sign = declare_parameter<double>("adaptation.sign", 1.0);
    config.correction_fraction =
        declare_parameter<double>("adaptation.correction_fraction", 0.50);
    config.offset_min = declare_parameter<double>("adaptation.offset_min", -0.050);
    config.offset_max = declare_parameter<double>("adaptation.offset_max", 0.050);
    config.offset_step_max =
        declare_parameter<double>("adaptation.offset_step_max", 0.00025);
    config.speed_threshold =
        declare_parameter<double>("adaptation.speed_threshold", 0.001);
    config.position_window_range_max =
        declare_parameter<double>("adaptation.position_window_range_max", 0.001);
    config.pitch_rate_threshold =
        declare_parameter<double>("adaptation.pitch_rate_threshold", 0.25);
    config.height_rate_threshold =
        declare_parameter<double>("adaptation.height_rate_threshold", 0.01);
    config.torque_threshold =
        declare_parameter<double>("adaptation.torque_threshold", 0.25);
    config.torque_ratio_threshold =
        declare_parameter<double>("adaptation.torque_ratio_threshold", 0.75);
    config.pitch_error_threshold =
        declare_parameter<double>("adaptation.pitch_error_threshold", 0.12);
    return config;
  }

  void adaptive_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "reset_position")
    {
      target_x_ = x_;
      reset_adaptation("position reference changed");
    }
    else if (msg->data == "toggle_adaptation")
    {
      if (experiment_mode_ != ExperimentMode::Adaptive)
      {
        RCLCPP_WARN(
            get_logger(), "T is ignored in experiment mode '%s'.",
            experiment_mode_name(experiment_mode_));
        return;
      }
      adaptation_enabled_ = !adaptation_enabled_;
      adaptation_updates_paused_ = false;
      reset_adaptation(adaptation_enabled_ ? "adaptation enabled" : "adaptation disabled");
      RCLCPP_INFO(
          get_logger(), "Adaptive compensation %s; estimate reset to zero.",
          adaptation_enabled_ ? "enabled" : "disabled");
    }
    else if (msg->data == "toggle_adaptation_hold")
    {
      if (experiment_mode_ != ExperimentMode::Adaptive || !adaptation_enabled_)
      {
        RCLCPP_WARN(
            get_logger(), "Cannot hold estimator unless adaptive mode is active and enabled.");
      }
      else
      {
        adaptation_updates_paused_ = !adaptation_updates_paused_;
        RCLCPP_INFO(
            get_logger(), "Adaptive estimator updates %s; compensation is retained.",
            adaptation_updates_paused_ ? "paused" : "resumed");
      }
    }
    else if (msg->data == "reset_adaptation")
    {
      reset_adaptation("manual reset");
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Unknown adaptive LQR command: %s", msg->data.c_str());
    }
  }

  void robot_mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "emergency")
    {
      control_enabled_ = false;
      publish_wheel_torque(0.0);
      RCLCPP_WARN(get_logger(), "Adaptive GS-LQR emergency stop.");
    }
    else if (msg->data == "balance")
    {
      target_x_ = x_;
      reset_adaptation("controller resumed");
      adaptation_updates_paused_ = false;
      control_enabled_ = true;
    }
  }

  void reset_adaptation(const char *reason)
  {
    estimator_.reset();
    RCLCPP_INFO(get_logger(), "Adaptive estimate reset: %s.", reason);
  }

  bool adaptive_compensation_active() const
  {
    return experiment_mode_ == ExperimentMode::Adaptive && adaptation_enabled_;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion quaternion(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w);
    double roll = 0.0;
    double unused_pitch = 0.0;
    double unused_yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, unused_pitch, unused_yaw);

    // The robot pitches about the URDF X axis. Forward lean is positive here.
    pitch_ = -roll;
    const double raw_pitch_rate = -msg->angular_velocity.x;
    if (!pitch_rate_filter_initialized_)
    {
      pitch_rate_ = raw_pitch_rate;
      pitch_rate_filter_initialized_ = true;
    }
    else
    {
      pitch_rate_ = low_pass_filter(raw_pitch_rate, pitch_rate_, 0.10);
    }
    imu_received_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool found_004 = false;
    bool found_007 = false;
    for (std::size_t index = 0; index < msg->name.size(); ++index)
    {
      if (msg->name[index] == "link_004_joint")
      {
        if (index < msg->position.size())
        {
          wheel_004_position_ = msg->position[index];
        }
        if (index < msg->velocity.size())
        {
          wheel_004_velocity_ = msg->velocity[index];
        }
        found_004 = true;
      }
      else if (msg->name[index] == "link_007_joint")
      {
        if (index < msg->position.size())
        {
          wheel_007_position_ = msg->position[index];
        }
        if (index < msg->velocity.size())
        {
          wheel_007_velocity_ = msg->velocity[index];
        }
        found_007 = true;
      }
    }

    if (!(found_004 && found_007))
    {
      return;
    }
    if (!wheel_origin_set_)
    {
      wheel_004_origin_ = wheel_004_position_;
      wheel_007_origin_ = wheel_007_position_;
      wheel_origin_set_ = true;
      target_x_ = 0.0;
    }

    const double raw_x_dot = -wheel_radius_ * 0.5 *
                             (wheel_004_velocity_ + wheel_007_velocity_);
    if (!x_dot_filter_initialized_)
    {
      x_dot_ = raw_x_dot;
      x_dot_filter_initialized_ = true;
    }
    else
    {
      x_dot_ = low_pass_filter(raw_x_dot, x_dot_, 0.05);
    }

    x_ = -wheel_radius_ * 0.5 *
         ((wheel_004_position_ - wheel_004_origin_) +
          (wheel_007_position_ - wheel_007_origin_));
  }

  void interpolate_schedule(
      double height, LQRGain &gain, double &nominal_pitch,
      double &nominal_com_y, double &nominal_com_z) const
  {
    if (height <= gain_table_.front().height)
    {
      assign_gain(
          gain_table_.front(), gain, nominal_pitch, nominal_com_y, nominal_com_z);
      return;
    }
    if (height >= gain_table_.back().height)
    {
      assign_gain(
          gain_table_.back(), gain, nominal_pitch, nominal_com_y, nominal_com_z);
      return;
    }

    for (std::size_t index = 0; index + 1 < gain_table_.size(); ++index)
    {
      const auto &low = gain_table_[index];
      const auto &high = gain_table_[index + 1];
      if (height < low.height || height > high.height)
      {
        continue;
      }
      const double ratio = (height - low.height) / (high.height - low.height);
      gain = {
          lerp(low.k_x, high.k_x, ratio),
          lerp(low.k_x_dot, high.k_x_dot, ratio),
          lerp(low.k_theta, high.k_theta, ratio),
          lerp(low.k_theta_dot, high.k_theta_dot, ratio)};
      nominal_com_y = lerp(low.y_com, high.y_com, ratio);
      nominal_com_z = lerp(low.z_com, high.z_com, ratio);
      nominal_pitch = -std::atan2(nominal_com_y, nominal_com_z);
      return;
    }
  }

  static void assign_gain(
      const GainPoint &point, LQRGain &gain, double &nominal_pitch,
      double &nominal_com_y, double &nominal_com_z)
  {
    gain = {point.k_x, point.k_x_dot, point.k_theta, point.k_theta_dot};
    nominal_com_y = point.y_com;
    nominal_com_z = point.z_com;
    nominal_pitch = -std::atan2(nominal_com_y, nominal_com_z);
  }

  double update_height(double dt)
  {
    previous_height_ = current_height_;
    const double step = std::max(0.0, leg_transition_speed_) * dt;
    if (current_height_ < target_height_)
    {
      current_height_ = std::min(current_height_ + step, target_height_);
    }
    else if (current_height_ > target_height_)
    {
      current_height_ = std::max(current_height_ - step, target_height_);
    }
    return (current_height_ - previous_height_) / dt;
  }

  void publish_leg_pose()
  {
    const auto solution = kinematics_.inverse_kinematics(base_link_height(), 0.0);
    std_msgs::msg::Float64MultiArray message;
    message.data = {
        solution.theta_hip, solution.theta_knee,
        solution.theta_hip, solution.theta_knee};
    leg_pub_->publish(message);
  }

  void publish_wheel_torque(double torque_each)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = {torque_each, torque_each};
    wheel_effort_pub_->publish(message);
  }

  void publish_adaptation_state()
  {
    std_msgs::msg::Float64 offset_message;
    offset_message.data = estimator_.state().equivalent_com_offset;
    offset_pub_->publish(offset_message);

    std_msgs::msg::Float64 pitch_message;
    pitch_message.data = theta_eq_adaptive_;
    equilibrium_pitch_pub_->publish(pitch_message);

    std_msgs::msg::Bool gate_message;
    gate_message.data = estimator_.state().gate_open;
    gate_pub_->publish(gate_message);
  }

  void control_loop()
  {
    const rclcpp::Time current_time = now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0001 || dt > 0.05)
    {
      dt = 0.005;
    }

    if (!imu_received_ || !wheel_origin_set_ || !control_enabled_)
    {
      // Hold the safe startup pose until the balance feedback signals are
      // available.  Extending the legs while wheel torque is disabled makes
      // the open-loop robot fall before LQR can take over.
      publish_leg_pose();
      publish_wheel_torque(0.0);
      return;
    }

    balance_ready_elapsed_ += dt;
    const double height_rate = balance_ready_elapsed_ >= startup_hold_time_ ? update_height(dt) : 0.0;
    publish_leg_pose();

    interpolate_schedule(
        current_height_, current_gain_, theta_eq_true_,
        true_com_y_, nominal_com_z_);
    used_com_y_ = true_com_y_ + injected_com_y_bias_;
    theta_eq_nominal_ = -std::atan2(used_com_y_, nominal_com_z_);
    const double true_equivalent_offset = -injected_com_y_bias_;
    const double x_error = x_ - target_x_;
    const double previous_theta_error = pitch_ - theta_eq_adaptive_;
    const bool adaptive_active = adaptive_compensation_active();

    bbot_balance_controller::AdaptiveEquilibriumInput estimator_input;
    estimator_input.x_error = x_error;
    estimator_input.x_dot = x_dot_;
    estimator_input.pitch = pitch_;
    estimator_input.pitch_error = previous_theta_error;
    estimator_input.pitch_rate = pitch_rate_;
    estimator_input.height_rate = height_rate;
    estimator_input.torque = last_u_model_;
    estimator_input.torque_ratio = last_u_model_ / total_torque_max_;
    estimator_input.nominal_com_y = used_com_y_;
    estimator_input.nominal_com_z = nominal_com_z_;
    estimator_input.enabled = adaptive_active && !adaptation_updates_paused_;
    estimator_.update(dt, estimator_input);

    if (experiment_mode_ == ExperimentMode::Oracle)
    {
      applied_com_y_offset_ = true_equivalent_offset;
    }
    else if (adaptive_active)
    {
      applied_com_y_offset_ = estimator_.state().equivalent_com_offset;
    }
    else
    {
      applied_com_y_offset_ = 0.0;
    }
    theta_eq_adaptive_ =
        bbot_balance_controller::AdaptiveEquilibriumEstimator::equilibrium_pitch(
            used_com_y_, nominal_com_z_, applied_com_y_offset_);
    const double theta_error = pitch_ - theta_eq_adaptive_;

    if (std::abs(theta_error) > max_pitch_error_)
    {
      control_enabled_ = false;
      publish_wheel_torque(0.0);
      RCLCPP_ERROR(
          get_logger(), "Pitch error %.3f rad exceeds %.3f rad; controller disabled.",
          theta_error, max_pitch_error_);
      return;
    }

    const double feedback =
        current_gain_.k_x * x_error +
        current_gain_.k_x_dot * x_dot_ +
        current_gain_.k_theta * theta_error +
        current_gain_.k_theta_dot * pitch_rate_;
    const double u_raw = -feedback;
    const double u_model = clamp_value(u_raw, -total_torque_max_, total_torque_max_);
    last_u_model_ = u_model;
    const double torque_each = clamp_value(
        -0.5 * u_model, -wheel_torque_max_, wheel_torque_max_);
    publish_wheel_torque(torque_each);
    publish_adaptation_state();
    log_data(
        x_error, theta_error, height_rate, u_raw, u_model, torque_each);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[ADAPT-LQR] mode=%s bias=%+.3fmm Lhip=%.3f Hbase=%.3f xerr=%+.3f "
        "pitch=%+.4f eq_nom=%+.4f "
        "eq_adapt=%+.4f dy_obs=%+.3fmm dy_hat=%+.3fmm enabled=%d paused=%d gate=%d "
        "state=%d target=%+.3fmm capture=%.0f%% obs_range=%.3fmm "
        "accel=%+.4fm/s2 apply_rate=%+.3fmm/s u=%+.2f Nm",
        experiment_mode_name(experiment_mode_), 1000.0 * injected_com_y_bias_,
        current_height_, base_link_height(), x_error, pitch_, theta_eq_nominal_,
        theta_eq_adaptive_,
        1000.0 * estimator_.state().observed_com_offset,
        1000.0 * estimator_.state().equivalent_com_offset,
        adaptive_active ? 1 : 0,
        adaptation_updates_paused_ ? 1 : 0,
        estimator_.state().gate_open ? 1 : 0,
        static_cast<int>(estimator_.state().phase),
        1000.0 * estimator_.state().target_com_offset,
        100.0 * estimator_.state().window_progress,
        1000.0 * estimator_.state().observation_window_range,
        estimator_.state().filtered_x_accel,
        1000.0 * estimator_.state().apply_rate,
        u_model);
  }

  void open_log_file()
  {
    const char *home_directory = std::getenv("HOME");
    const std::string base = home_directory ? home_directory : "/tmp";
    const std::string default_path = base +
                                     "/bbot_ws_new/src/bbot_balance_controller/src/data_logs/adaptive_lqr_log.csv";
    const std::string path = declare_parameter<std::string>("log_path", default_path);
    log_file_.open(path);
    if (!log_file_.is_open())
    {
      RCLCPP_WARN(get_logger(), "Cannot open adaptive LQR log: %s", path.c_str());
      return;
    }
    log_file_ << "time,hip_axle_height,base_link_height,height_rate,x,x_ref,x_error,x_dot,"
                 "pitch,pitch_rate,"
                 "theta_eq_nominal,theta_eq_adaptive,theta_error,experiment_mode,"
                 "injected_com_y_bias,theta_eq_true,com_y_true,com_y_used,nominal_com_z,"
                 "delta_y_true,delta_y_applied,"
                 "delta_y_obs,filtered_delta_y_obs,delta_y_hat,delta_y_step,"
                 "filtered_x_error,filtered_x_dot,correction_error,observation_valid,"
                 "gate_open,adapt_enabled,"
                 "adapt_update_paused,window_progress,window_position_range,cooldown_remaining,"
                 "adapt_state,delta_y_target,delta_y_apply_rate,filtered_x_accel,"
                 "observation_window_range,capture_progress,verify_progress,target_updated,"
                 "u_raw,u_model,tau_each\n";
  }

  void log_data(
      double x_error, double theta_error, double height_rate,
      double u_raw, double u_model, double torque_each)
  {
    if (!log_file_.is_open())
    {
      return;
    }
    const double elapsed = (now() - start_time_).seconds();
    const auto &adaptive = estimator_.state();
    log_file_ << elapsed << ',' << current_height_ << ',' << base_link_height() << ',' << height_rate << ',' << x_ << ',' << target_x_ << ',' << x_error << ',' << x_dot_ << ',' << pitch_ << ',' << pitch_rate_ << ',' << theta_eq_nominal_ << ',' << theta_eq_adaptive_ << ',' << theta_error << ',' << static_cast<int>(experiment_mode_) << ',' << injected_com_y_bias_ << ',' << theta_eq_true_ << ',' << true_com_y_ << ',' << used_com_y_ << ',' << nominal_com_z_ << ',' << -injected_com_y_bias_ << ',' << applied_com_y_offset_ << ',' << adaptive.observed_com_offset << ',' << adaptive.filtered_observed_com_offset << ',' << adaptive.equivalent_com_offset << ',' << adaptive.offset_step << ',' << adaptive.filtered_x_error << ',' << adaptive.filtered_x_dot << ',' << adaptive.correction_error << ',' << (adaptive.observation_valid ? 1 : 0) << ',' << (adaptive.gate_open ? 1 : 0) << ',' << (adaptive_compensation_active() ? 1 : 0) << ',' << (adaptation_updates_paused_ ? 1 : 0) << ',' << adaptive.window_progress << ',' << adaptive.window_position_range << ',' << adaptive.cooldown_remaining << ',' << static_cast<int>(adaptive.phase) << ',' << adaptive.target_com_offset << ',' << adaptive.apply_rate << ',' << adaptive.filtered_x_accel << ',' << adaptive.observation_window_range << ',' << adaptive.window_progress << ',' << adaptive.verify_progress << ',' << (adaptive.target_updated ? 1 : 0) << ',' << u_raw << ',' << u_model << ',' << torque_each << '\n';
  }

  double base_link_height_offset() const
  {
    return base_to_hip_height_ + wheel_radius_;
  }

  double base_link_height() const
  {
    return current_height_ + base_link_height_offset();
  }

  bbot_kinematics::Kinematics kinematics_;
  bbot_balance_controller::AdaptiveEquilibriumEstimator estimator_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr adaptive_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr equilibrium_pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_time_;
  rclcpp::Time start_time_;
  std::ofstream log_file_;
  LQRGain current_gain_;

  bool imu_received_{false};
  bool wheel_origin_set_{false};
  bool control_enabled_{true};
  bool adaptation_enabled_{true};
  bool adaptation_updates_paused_{false};
  bool log_enabled_{true};
  bool pitch_rate_filter_initialized_{false};
  bool x_dot_filter_initialized_{false};
  ExperimentMode experiment_mode_{ExperimentMode::Adaptive};

  double pitch_{0.0};
  double pitch_rate_{0.0};
  double wheel_004_position_{0.0};
  double wheel_007_position_{0.0};
  double wheel_004_velocity_{0.0};
  double wheel_007_velocity_{0.0};
  double wheel_004_origin_{0.0};
  double wheel_007_origin_{0.0};
  double x_{0.0};
  double x_dot_{0.0};
  double target_x_{0.0};
  double target_height_{0.50};
  double current_height_{0.50};
  double previous_height_{0.50};
  double leg_transition_speed_{0.05};
  double height_min_{0.30};
  double height_max_{0.50};
  double startup_height_{0.36};
  double startup_hold_time_{2.0};
  double balance_ready_elapsed_{0.0};
  double base_to_hip_height_{0.07};
  double wheel_radius_{0.07};
  double wheel_torque_max_{10.0};
  double total_torque_max_{20.0};
  double max_pitch_error_{0.50};
  double injected_com_y_bias_{0.0};
  double theta_eq_nominal_{0.0};
  double theta_eq_adaptive_{0.0};
  double theta_eq_true_{0.0};
  double true_com_y_{0.0};
  double used_com_y_{0.0};
  double nominal_com_z_{0.0};
  double applied_com_y_offset_{0.0};
  double last_u_model_{0.0};
};

constexpr std::array<GainPoint, 5> AdaptiveLQRBalanceController::gain_table_;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveLQRBalanceController>());
  rclcpp::shutdown();
  return 0;
}
