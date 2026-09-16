#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "bbot_kinematics/kinematics.hpp"

namespace bbot_balance_controller
{

using namespace std::chrono_literals;

namespace
{
double clamp_value(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

double low_pass(double value, double previous, double alpha)
{
  return alpha * value + (1.0 - alpha) * previous;
}

double lerp(double a, double b, double ratio)
{
  return a + (b - a) * ratio;
}
}  // namespace

struct InnerGains
{
  double kp_rate = 0.0;
  double kd_rate = 0.0;
  double kp_theta = 0.0;
  double kd_theta = 0.0;
  double kp_v = 0.0;
  double ki_v = 0.0;
  double kd_v = 0.0;
};

struct GeomPoint
{
  double height;
  double y_com;
  double z_com;
};

class PositionTorqueCascadePIDController : public rclcpp::Node
{
public:
  PositionTorqueCascadePIDController()
  : Node("position_torque_cascade_pid_controller")
  {
    const auto & robot = kinematics_.get_params();
    wheel_radius_ = robot.wheel_radius;
    wheel_torque_max_ = declare_parameter<double>("pid.wheel_torque_max", 10.0);
    total_torque_max_ = declare_parameter<double>("pid.total_torque_max", 20.0);

    height_min_ = declare_parameter<double>("height.hip_axle_min", 0.30);
    height_max_ = declare_parameter<double>("height.hip_axle_max", 0.50);
    base_to_hip_height_ = declare_parameter<double>("height.base_to_hip", 0.07);
    startup_height_ = clamp_value(
      declare_parameter<double>("height.startup_hip_axle", 0.36), height_min_, height_max_);
    target_height_ = clamp_value(
      declare_parameter<double>("target_height", 0.50), height_min_, height_max_);
    current_height_ = startup_height_;
    previous_height_ = current_height_;
    startup_hold_time_ = declare_parameter<double>("height.startup_hold_time", 2.0);
    leg_transition_speed_ = declare_parameter<double>("leg_transition_speed", 0.05);

    max_pitch_error_ = declare_parameter<double>("max_pitch_error", 0.50);
    rate_limit_u_ = declare_parameter<double>("pid.rate_limit_u", 0.0);
    position_v_ref_limit_ = declare_parameter<double>("pid.position.v_ref_limit", 0.40);
    position_integral_limit_ = declare_parameter<double>("pid.position.integral_limit", 2.0);
    delta_theta_limit_ = declare_parameter<double>("pid.delta_theta_limit", 0.055);

    position_kp_ = declare_parameter<double>("pid.position.kp", 0.50);
    position_ki_ = declare_parameter<double>("pid.position.ki", 0.010);
    position_kd_ = declare_parameter<double>("pid.position.kd", 0.30);

    gains_low_.kp_rate = declare_parameter<double>("pid.low.kp_rate", 20.0);
    gains_low_.kd_rate = declare_parameter<double>("pid.low.kd_rate", 0.02);
    gains_low_.kp_theta = declare_parameter<double>("pid.low.kp_theta", 5.5);
    gains_low_.kd_theta = declare_parameter<double>("pid.low.kd_theta", 0.10);
    gains_low_.kp_v = declare_parameter<double>("pid.low.kp_v", 0.08);
    gains_low_.ki_v = declare_parameter<double>("pid.low.ki_v", 0.008);
    gains_low_.kd_v = declare_parameter<double>("pid.low.kd_v", 0.001);
    gains_high_.kp_rate = declare_parameter<double>("pid.high.kp_rate", 22.0);
    gains_high_.kd_rate = declare_parameter<double>("pid.high.kd_rate", 0.025);
    gains_high_.kp_theta = declare_parameter<double>("pid.high.kp_theta", 6.0);
    gains_high_.kd_theta = declare_parameter<double>("pid.high.kd_theta", 0.12);
    gains_high_.kp_v = declare_parameter<double>("pid.high.kp_v", 0.09);
    gains_high_.ki_v = declare_parameter<double>("pid.high.ki_v", 0.008);
    gains_high_.kd_v = declare_parameter<double>("pid.high.kd_v", 0.001);

    log_path_ = declare_parameter<std::string>(
      "log_path",
      "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/position_torque_pid_exploration/position_pid_log.csv");
    log_enabled_ = declare_parameter<bool>("log_enabled", true);

    wheel_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_effort_controller/commands", 10);
    leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/leg_position_controller/commands", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&PositionTorqueCascadePIDController::imu_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PositionTorqueCascadePIDController::joint_state_callback, this, std::placeholders::_1));
    target_height_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/target_height", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        target_height_ = clamp_value(msg->data, height_min_, height_max_);
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/position_torque_cascade_pid/command", 10,
      std::bind(&PositionTorqueCascadePIDController::command_callback, this, std::placeholders::_1));
    legacy_command_sub_ = create_subscription<std_msgs::msg::String>(
      "/adaptive_lqr/command", 10,
      std::bind(&PositionTorqueCascadePIDController::command_callback, this, std::placeholders::_1));
    dist_force_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/disturbance_force_y", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        current_dist_force_ = msg->data;
        last_dist_force_time_ = now();
      });
    robot_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "emergency" || msg->data == "x" || msg->data == "X") {
          control_enabled_ = false;
          publish_wheel_torque(0.0);
          reset_inner_state();
        } else if (msg->data == "balance" || msg->data == "b" || msg->data == "B") {
          control_enabled_ = true;
          reset_inner_state();
        }
      });

    if (log_enabled_) {
      open_log_file();
    }
    start_time_ = now();
    last_time_ = start_time_;
    timer_ = create_wall_timer(5ms, std::bind(&PositionTorqueCascadePIDController::control_loop, this));
    RCLCPP_INFO(
      get_logger(),
      "Independent four-loop PID ready: position -> velocity -> attitude -> rate; "
      "kp=%.4f ki=%.4f kd=%.4f v_ref_limit=%.3f, torque=±%.1f/±%.1f Nm, log=%s",
      position_kp_, position_ki_, position_kd_, position_v_ref_limit_,
      total_torque_max_, wheel_torque_max_, log_path_.c_str());
  }

  ~PositionTorqueCascadePIDController() override
  {
    publish_wheel_torque(0.0);
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

private:
  static constexpr std::array<GeomPoint, 5> geom_table_{
    {{0.3000, -0.0276264, 0.3592154},
     {0.3500, -0.0258066, 0.4016316},
     {0.4000, -0.0234056, 0.4443432},
     {0.4500, -0.0203401, 0.4872441},
     {0.5000, -0.0164269, 0.5302655}}};

  void open_log_file()
  {
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open four-loop PID log: %s", log_path_.c_str());
      return;
    }
    log_file_ << "time,height,base_link_height,p,p_target,delta_p,"
              << "position_target_latched,position_reset_event_count,position_latch_count,"
              << "v,v_ref_raw,v_ref,v_error,p_error,p_p,p_i,p_d,position_saturated,"
              << "position_anti_windup,position_integral,"
              << "pitch,pitch_rate,theta_eq,theta_ref,theta_error,theta_dot_ref,theta_dot_error,"
              << "v_p,v_i,v_d,theta_p,theta_d,rate_p,rate_d,"
              << "u_raw,u_rate_limited,u_clamped,tau_left,tau_right,"
              << "is_saturated,is_rate_limited,control_enabled,force_active,force_value,"
              << "kp_position,ki_position,kd_position,v_ref_limit,"
              << "kp_v,ki_v,kd_v,kp_theta,kd_theta,kp_rate,kd_rate,"
              << "rate_limit_u,total_torque_max,wheel_torque_max\n";
  }

  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "reset_position") {
      ++position_reset_event_count_;
      reset_event_pending_ = true;
      if (!position_target_latched_) {
        position_reset_requested_ = true;
        RCLCPP_INFO(get_logger(), "First reset_position received; waiting for stable balance to latch target");
      } else {
        RCLCPP_INFO(
          get_logger(), "Repeated reset_position ignored; fixed position target remains %.6f m", position_target_);
      }
    } else if (msg->data == "reset_pid") {
      reset_inner_state();
    }
  }

  void reset_inner_state()
  {
    position_integral_ = 0.0;
    v_integral_ = 0.0;
    v_error_prev_ = 0.0;
    rate_error_prev_ = 0.0;
    last_u_clamped_ = 0.0;
    delta_theta_ref_filt_ = 0.0;
    delta_theta_filt_init_ = false;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll = 0.0, pitch_unused = 0.0, yaw_unused = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch_unused, yaw_unused);
    pitch_ = -roll;
    const double raw_rate = -msg->angular_velocity.x;
    if (!pitch_rate_filter_init_) {
      pitch_rate_ = raw_rate;
      pitch_rate_filter_init_ = true;
    } else {
      pitch_rate_ = low_pass(raw_rate, pitch_rate_, 0.10);
    }
    imu_received_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool found_left = false, found_right = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "link_004_joint") {
        if (i < msg->position.size()) wheel_left_pos_ = msg->position[i];
        if (i < msg->velocity.size()) wheel_left_vel_ = msg->velocity[i];
        found_left = true;
      } else if (msg->name[i] == "link_007_joint") {
        if (i < msg->position.size()) wheel_right_pos_ = msg->position[i];
        if (i < msg->velocity.size()) wheel_right_vel_ = msg->velocity[i];
        found_right = true;
      }
    }
    if (!(found_left && found_right)) return;
    if (!wheel_origin_set_) {
      wheel_left_origin_ = wheel_left_pos_;
      wheel_right_origin_ = wheel_right_pos_;
      wheel_origin_set_ = true;
    }
    const double raw_v = -wheel_radius_ * 0.5 * (wheel_left_vel_ + wheel_right_vel_);
    v_ = v_filter_init_ ? low_pass(raw_v, v_, 0.05) : raw_v;
    v_filter_init_ = true;
    p_ = -wheel_radius_ * 0.5 *
      ((wheel_left_pos_ - wheel_left_origin_) + (wheel_right_pos_ - wheel_right_origin_));
  }

  double base_link_height() const
  {
    return current_height_ + base_to_hip_height_ + wheel_radius_;
  }

  void update_height(double dt)
  {
    previous_height_ = current_height_;
    const double step = leg_transition_speed_ * dt;
    if (current_height_ < target_height_) {
      current_height_ = std::min(current_height_ + step, target_height_);
    } else if (current_height_ > target_height_) {
      current_height_ = std::max(current_height_ - step, target_height_);
    }
  }

  void publish_leg_pose()
  {
    const auto solution = kinematics_.inverse_kinematics(base_link_height(), 0.0);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {solution.theta_hip, solution.theta_knee, solution.theta_hip, solution.theta_knee};
    leg_pub_->publish(msg);
  }

  void publish_wheel_torque(double torque_each)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {torque_each, torque_each};
    wheel_effort_pub_->publish(msg);
  }

  double compute_theta_eq(double h) const
  {
    if (h <= geom_table_.front().height) {
      return -std::atan2(geom_table_.front().y_com, geom_table_.front().z_com);
    }
    if (h >= geom_table_.back().height) {
      return -std::atan2(geom_table_.back().y_com, geom_table_.back().z_com);
    }
    for (size_t i = 0; i + 1 < geom_table_.size(); ++i) {
      if (h >= geom_table_[i].height && h <= geom_table_[i + 1].height) {
        const double ratio = (h - geom_table_[i].height) /
          (geom_table_[i + 1].height - geom_table_[i].height);
        const double y = lerp(geom_table_[i].y_com, geom_table_[i + 1].y_com, ratio);
        const double z = lerp(geom_table_[i].z_com, geom_table_[i + 1].z_com, ratio);
        return -std::atan2(y, z);
      }
    }
    return 0.0;
  }

  InnerGains interpolate_gains(double h) const
  {
    const double ratio = clamp_value((h - height_min_) / (height_max_ - height_min_), 0.0, 1.0);
    InnerGains g;
    g.kp_rate = lerp(gains_low_.kp_rate, gains_high_.kp_rate, ratio);
    g.kd_rate = lerp(gains_low_.kd_rate, gains_high_.kd_rate, ratio);
    g.kp_theta = lerp(gains_low_.kp_theta, gains_high_.kp_theta, ratio);
    g.kd_theta = lerp(gains_low_.kd_theta, gains_high_.kd_theta, ratio);
    g.kp_v = lerp(gains_low_.kp_v, gains_high_.kp_v, ratio);
    g.ki_v = lerp(gains_low_.ki_v, gains_high_.ki_v, ratio);
    g.kd_v = lerp(gains_low_.kd_v, gains_high_.kd_v, ratio);
    return g;
  }

  void control_loop()
  {
    const auto current_time = now();
    // The wall timer continues to fire while Gazebo is paused.  Do not run
    // derivative terms repeatedly at an unchanged simulation timestamp; use
    // the same simulation-time gating as the accepted three-loop controller.
    if (last_control_sim_time_.nanoseconds() != 0) {
      const double sim_elapsed = (current_time - last_control_sim_time_).seconds();
      if (sim_elapsed < 0.0) {
        last_control_sim_time_ = current_time;
        last_time_ = current_time;
        reset_inner_state();
        return;
      }
      if (sim_elapsed < 0.0045) {
        return;
      }
    }
    last_control_sim_time_ = current_time;
    double dt = (last_time_.nanoseconds() != 0) ? (current_time - last_time_).seconds() : 0.005;
    last_time_ = current_time;
    if (dt <= 0.0001 || dt > 0.05) dt = 0.005;
    if (!imu_received_ || !wheel_origin_set_ || !control_enabled_) {
      publish_leg_pose();
      publish_wheel_torque(0.0);
      return;
    }

    balance_ready_elapsed_ += dt;
    if (balance_ready_elapsed_ >= startup_hold_time_) update_height(dt);
    publish_leg_pose();
    const InnerGains gains = interpolate_gains(current_height_);
    const double theta_eq = compute_theta_eq(current_height_);
    const bool valid_balance = std::isfinite(p_) &&
      std::abs(pitch_ - theta_eq) <= 0.10 && std::abs(pitch_rate_) <= 0.20 && std::abs(v_) <= 0.05;
    valid_balance_elapsed_ = valid_balance ? valid_balance_elapsed_ + dt : 0.0;

    if (position_reset_requested_ && !position_target_latched_ && valid_balance_elapsed_ >= 0.50) {
      position_target_ = p_;
      position_target_latched_ = true;
      ++position_latch_count_;
      position_integral_ = 0.0;
      RCLCPP_INFO(get_logger(), "Fixed position target latched once: p_target=%.6f m", position_target_);
    }

    const double p_error = position_target_latched_ ? position_target_ - p_ : 0.0;
    const double p_p = position_target_latched_ ? position_kp_ * p_error : 0.0;
    const double p_d = position_target_latched_ ? -position_kd_ * v_ : 0.0;
    const double p_raw_before_i = p_p + p_d + position_ki_ * position_integral_;
    const bool position_saturated_before_i = std::abs(p_raw_before_i) >= position_v_ref_limit_;
    const bool position_anti_windup = position_saturated_before_i &&
      ((p_raw_before_i > 0.0 && p_error > 0.0) || (p_raw_before_i < 0.0 && p_error < 0.0));
    if (position_target_latched_ && !position_anti_windup && position_ki_ > 1e-9) {
      position_integral_ += p_error * dt;
      position_integral_ = clamp_value(position_integral_, -position_integral_limit_, position_integral_limit_);
    }
    const double p_i = position_target_latched_ ? position_ki_ * position_integral_ : 0.0;
    const double v_ref_raw = p_p + p_i + p_d;
    const double v_ref = clamp_value(v_ref_raw, -position_v_ref_limit_, position_v_ref_limit_);
    const bool position_saturated = std::abs(v_ref_raw - v_ref) > 1e-9;

    const double v_error = v_ref - v_;
    const double v_p = gains.kp_v * v_error;
    const double v_d = gains.kd_v * ((v_error - v_error_prev_) / dt);
    const double velocity_pre_i = v_p + gains.ki_v * v_integral_ + v_d;
    const bool velocity_anti_windup = std::abs(velocity_pre_i) >= delta_theta_limit_ &&
      ((velocity_pre_i > 0.0 && v_error > 0.0) || (velocity_pre_i < 0.0 && v_error < 0.0));
    if (!velocity_anti_windup && gains.ki_v > 1e-9) {
      v_integral_ += v_error * dt;
      v_integral_ = clamp_value(v_integral_, -delta_theta_limit_ / gains.ki_v, delta_theta_limit_ / gains.ki_v);
    }
    const double v_i = gains.ki_v * v_integral_;
    const double delta_theta = clamp_value(v_p + v_i + v_d, -delta_theta_limit_, delta_theta_limit_);
    if (!delta_theta_filt_init_) {
      delta_theta_ref_filt_ = delta_theta;
      delta_theta_filt_init_ = true;
    } else {
      delta_theta_ref_filt_ = low_pass(delta_theta, delta_theta_ref_filt_, 0.10);
    }
    const double theta_ref = theta_eq + delta_theta_ref_filt_;
    const double theta_error = theta_ref - pitch_;
    if (std::abs(pitch_ - theta_eq) > max_pitch_error_) {
      control_enabled_ = false;
      publish_wheel_torque(0.0);
      reset_inner_state();
      RCLCPP_ERROR(get_logger(), "Pitch error %.3f rad exceeds %.3f rad; controller disabled", pitch_ - theta_eq, max_pitch_error_);
      return;
    }

    const double theta_p = gains.kp_theta * theta_error;
    const double theta_d = -gains.kd_theta * pitch_rate_;
    const double theta_dot_ref = clamp_value(theta_p + theta_d, -3.2, 3.2);
    const double theta_dot_error = theta_dot_ref - pitch_rate_;
    const double rate_p = gains.kp_rate * theta_dot_error;
    const double rate_d = gains.kd_rate * ((theta_dot_error - rate_error_prev_) / dt);
    const double u_raw = rate_p + rate_d;
    v_error_prev_ = v_error;
    rate_error_prev_ = theta_dot_error;

    double u_rate_limited = u_raw;
    bool is_rate_limited = false;
    if (rate_limit_u_ > 0.0) {
      const double max_du = rate_limit_u_ * dt;
      u_rate_limited = clamp_value(u_raw, last_u_clamped_ - max_du, last_u_clamped_ + max_du);
      is_rate_limited = std::abs(u_rate_limited - u_raw) > 1e-4;
    }
    const double u_clamped = clamp_value(u_rate_limited, -total_torque_max_, total_torque_max_);
    const bool is_saturated = std::abs(u_clamped - u_rate_limited) > 1e-4;
    last_u_clamped_ = u_clamped;
    const double tau_each = clamp_value(0.5 * u_clamped, -wheel_torque_max_, wheel_torque_max_);
    publish_wheel_torque(tau_each);

    bool force_active = false;
    double force_value = 0.0;
    if (last_dist_force_time_.nanoseconds() != 0) {
      const double force_age = (current_time - last_dist_force_time_).seconds();
      if (force_age >= 0.0 && force_age < 0.25 && std::abs(current_dist_force_) > 1e-3) {
        force_active = true;
        force_value = current_dist_force_;
      }
    }

    if (log_file_.is_open()) {
      const double t = (current_time - start_time_).seconds();
      log_file_ << std::fixed << std::setprecision(6)
                << t << ',' << current_height_ << ',' << base_link_height() << ','
                << p_ << ',' << position_target_ << ',' << (p_ - position_target_) << ','
                << (position_target_latched_ ? 1 : 0) << ',' << position_reset_event_count_ << ','
                << position_latch_count_ << ',' << v_ << ',' << v_ref_raw << ',' << v_ref << ','
                << v_error << ',' << p_error << ',' << p_p << ',' << p_i << ',' << p_d << ','
                << (position_saturated ? 1 : 0) << ',' << (position_anti_windup ? 1 : 0) << ','
                << position_integral_ << ',' << pitch_ << ',' << pitch_rate_ << ',' << theta_eq << ','
                << theta_ref << ',' << theta_error << ',' << theta_dot_ref << ',' << theta_dot_error << ','
                << v_p << ',' << v_i << ',' << v_d << ',' << theta_p << ',' << theta_d << ','
                << rate_p << ',' << rate_d << ',' << u_raw << ',' << u_rate_limited << ',' << u_clamped << ','
                << tau_each << ',' << tau_each << ',' << (is_saturated ? 1 : 0) << ','
                << (is_rate_limited ? 1 : 0) << ',' << (control_enabled_ ? 1 : 0) << ','
                << (force_active ? 1 : 0) << ',' << force_value << ','
                << position_kp_ << ',' << position_ki_ << ',' << position_kd_ << ','
                << position_v_ref_limit_ << ',' << gains.kp_v << ',' << gains.ki_v << ',' << gains.kd_v << ','
                << gains.kp_theta << ',' << gains.kd_theta << ',' << gains.kp_rate << ',' << gains.kd_rate << ','
                << rate_limit_u_ << ',' << total_torque_max_ << ',' << wheel_torque_max_ << '\n';
      log_file_.flush();
    }
    reset_event_pending_ = false;
  }

  bbot_kinematics::Kinematics kinematics_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr legacy_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_force_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_dist_force_time_{0, 0, RCL_ROS_TIME};

  double wheel_radius_ = 0.07;
  double wheel_torque_max_ = 10.0;
  double total_torque_max_ = 20.0;
  double height_min_ = 0.30, height_max_ = 0.50;
  double base_to_hip_height_ = 0.07;
  double startup_height_ = 0.36, target_height_ = 0.50;
  double current_height_ = 0.36, previous_height_ = 0.36;
  double startup_hold_time_ = 2.0, leg_transition_speed_ = 0.05;
  double max_pitch_error_ = 0.50, rate_limit_u_ = 0.0;
  double position_v_ref_limit_ = 0.40, position_integral_limit_ = 2.0;
  double delta_theta_limit_ = 0.055;
  double position_kp_ = 0.50, position_ki_ = 0.010, position_kd_ = 0.30;

  bool imu_received_ = false, wheel_origin_set_ = false, control_enabled_ = true;
  bool v_filter_init_ = false, pitch_rate_filter_init_ = false;
  bool position_reset_requested_ = false, position_target_latched_ = false;
  bool reset_event_pending_ = false, delta_theta_filt_init_ = false;
  bool log_enabled_ = true;
  int position_reset_event_count_ = 0, position_latch_count_ = 0;
  double balance_ready_elapsed_ = 0.0, valid_balance_elapsed_ = 0.0;
  double pitch_ = 0.0, pitch_rate_ = 0.0, p_ = 0.0, v_ = 0.0;
  double position_target_ = 0.0, position_integral_ = 0.0;
  double v_integral_ = 0.0, v_error_prev_ = 0.0, rate_error_prev_ = 0.0;
  double delta_theta_ref_filt_ = 0.0, last_u_clamped_ = 0.0;
  double current_dist_force_ = 0.0;
  double wheel_left_pos_ = 0.0, wheel_right_pos_ = 0.0;
  double wheel_left_vel_ = 0.0, wheel_right_vel_ = 0.0;
  double wheel_left_origin_ = 0.0, wheel_right_origin_ = 0.0;
  InnerGains gains_low_, gains_high_;
  std::string log_path_;
  std::ofstream log_file_;
};

}  // namespace bbot_balance_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bbot_balance_controller::PositionTorqueCascadePIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
