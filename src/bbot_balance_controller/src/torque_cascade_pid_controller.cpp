#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

namespace bbot_balance_controller
{

  namespace // 匿名命名空间，仅在此cpp内使用。
  {
    inline double clamp_value(double val, double min_val, double max_val)
    {
      return std::max(min_val, std::min(val, max_val));
    }

    inline double low_pass_filter(double new_val, double old_val, double alpha)
    {
      return alpha * new_val + (1.0 - alpha) * old_val;
    }

    inline double lerp(double a, double b, double ratio)
    {
      return a + (b - a) * ratio;
    }
  } // namespace

  struct CascadePIDGains
  {
    // Velocity loop (outer)
    double kp_v = 0.0;
    double ki_v = 0.0;
    double kd_v = 0.0;

    // Attitude angle loop (mid)
    double kp_theta = 0.0;
    double kd_theta = 0.0;

    // Pitch rate loop (inner)
    double kp_rate = 0.0;
    double kd_rate = 0.0;
  };

  struct NominalGeomPoint
  {
    double height;
    double y_com;
    double z_com;
  };

  class TorqueCascadePIDController : public rclcpp::Node
  {
  public:
    TorqueCascadePIDController()
        : Node("torque_cascade_pid_controller")
    {
      const auto &robot = kinematics_.get_params();
      wheel_radius_ = robot.wheel_radius;
      wheel_torque_max_ = robot.wheel_torque_max;
      total_torque_max_ = 2.0 * wheel_torque_max_;

      // Height parameters (aligned with real robot & adaptive LQR: wheel-axle to hip distance)
      height_min_ = declare_parameter<double>("height.hip_axle_min", 0.30);
      height_max_ = declare_parameter<double>("height.hip_axle_max", 0.50);
      base_to_hip_height_ = declare_parameter<double>("height.base_to_hip", 0.07);
      startup_height_ = declare_parameter<double>("height.startup_hip_axle", 0.36);
      startup_hold_time_ = declare_parameter<double>("height.startup_hold_time", 2.0);
      leg_transition_speed_ = declare_parameter<double>("leg_transition_speed", 0.05);
      target_height_ = declare_parameter<double>("target_height", 0.50);

      target_height_ = clamp_value(target_height_, height_min_, height_max_);
      startup_height_ = clamp_value(startup_height_, height_min_, height_max_);
      current_height_ = startup_height_;
      previous_height_ = current_height_;

      // Safety and limits
      max_pitch_error_ = declare_parameter<double>("max_pitch_error", 0.50);
      rate_limit_u_ = declare_parameter<double>("pid.rate_limit_u", 0.0); // <=0.0 disabled (matches GS-LQR unconstrained slew rate)
      total_torque_max_ = declare_parameter<double>("pid.total_torque_max", 20.0);
      wheel_torque_max_ = declare_parameter<double>("pid.wheel_torque_max", 10.0);
      delta_theta_limit_ = declare_parameter<double>("pid.delta_theta_limit", 0.055);

      // Stage mode for incremental tuning:
      // "stage_a": pitch rate loop only (theta_dot_ref = 0)
      // "stage_b": pitch angle + rate loops (delta_theta_ref = 0)
      // "stage_c" / "normal": full 3-loop cascade (velocity -> attitude -> rate, position strictly observed)
      stage_mode_ = declare_parameter<std::string>("stage_mode", "normal");

      // Position is an observation-only signal in this baseline.  Keep the legacy
      // parameter for launch-file compatibility, but never use a non-zero value
      // in the control law.
      k_x_ = declare_parameter<double>("pid.k_x", 0.0);
      if (std::abs(k_x_) > 1e-12)
      {
        RCLCPP_WARN(
            get_logger(),
            "pid.k_x=%.6f was requested but position feedback is disabled; forcing k_x=0",
            k_x_);
        k_x_ = 0.0;
      }

      // Endpoint PID gains: Low (H = 0.30 m)
      gains_low_.kp_rate = declare_parameter<double>("pid.low.kp_rate", 20.0);
      gains_low_.kd_rate = declare_parameter<double>("pid.low.kd_rate", 0.02);
      gains_low_.kp_theta = declare_parameter<double>("pid.low.kp_theta", 5.5);
      gains_low_.kd_theta = declare_parameter<double>("pid.low.kd_theta", 0.10);
      gains_low_.kp_v = declare_parameter<double>("pid.low.kp_v", 0.08);
      gains_low_.ki_v = declare_parameter<double>("pid.low.ki_v", 0.012);
      gains_low_.kd_v = declare_parameter<double>("pid.low.kd_v", 0.001);

      // Endpoint PID gains: High (H = 0.50 m)
      gains_high_.kp_rate = declare_parameter<double>("pid.high.kp_rate", 22.0);
      gains_high_.kd_rate = declare_parameter<double>("pid.high.kd_rate", 0.025);
      gains_high_.kp_theta = declare_parameter<double>("pid.high.kp_theta", 6.0);
      gains_high_.kd_theta = declare_parameter<double>("pid.high.kd_theta", 0.12);
      gains_high_.kp_v = declare_parameter<double>("pid.high.kp_v", 0.09);
      gains_high_.ki_v = declare_parameter<double>("pid.high.ki_v", 0.012);
      gains_high_.kd_v = declare_parameter<double>("pid.high.kd_v", 0.001);

      // Test excitation step disturbances for systematic multi-stage tuning
      rate_disturbance_step_ = declare_parameter<double>("pid.rate_disturbance_step", 0.0);
      rate_disturbance_start_time_ = declare_parameter<double>("pid.rate_disturbance_start_time", 0.6);
      rate_disturbance_duration_ = declare_parameter<double>("pid.rate_disturbance_duration", 0.2);

      attitude_disturbance_step_ = declare_parameter<double>("pid.attitude_disturbance_step", 0.0);
      disturbance_step_start_time_ = declare_parameter<double>("pid.disturbance_step_start_time", 2.0);
      disturbance_step_duration_ = declare_parameter<double>("pid.disturbance_step_duration", 1.0);

      velocity_disturbance_step_ = declare_parameter<double>("pid.velocity_disturbance_step", 0.0);
      velocity_disturbance_start_time_ = declare_parameter<double>("pid.velocity_disturbance_start_time", 2.5);
      velocity_disturbance_duration_ = declare_parameter<double>("pid.velocity_disturbance_duration", 0.5);

      // Logging
      log_enabled_ = declare_parameter<bool>("log_enabled", true);
      log_path_ = declare_parameter<std::string>(
          "log_path",
          "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/torque_pid_log.csv");

      // Publishers
      wheel_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/wheel_effort_controller/commands", 10);
      leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/leg_position_controller/commands", 10);

      // Subscribers
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, std::bind(&TorqueCascadePIDController::imu_callback, this, std::placeholders::_1));
      joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          std::bind(&TorqueCascadePIDController::joint_state_callback, this, std::placeholders::_1));
      target_height_sub_ = create_subscription<std_msgs::msg::Float64>(
          "/target_height", 10,
          [this](const std_msgs::msg::Float64::SharedPtr msg)
          {
            target_height_ = clamp_value(msg->data, height_min_, height_max_);
          });

      auto handle_cmd = [this](const std::string &cmd)
      {
        if (cmd == "reset_position")
        {
          // This command is retained for manual debugging only.  It may reset
          // the velocity-loop integrator, but it must never change the fixed
          // position origin p_0 used by an acceptance run.
          v_integral_ = 0.0;
          RCLCPP_INFO(
              get_logger(),
              "[Torque-PID] Manual reset_position called: p_0 remains fixed at %.4f m",
              p_0_);
        }
        else if (cmd == "reset_pid")
        {
          reset_pid_state();
          RCLCPP_INFO(get_logger(), "[Torque-PID] Reset all PID state and integrators");
        }
      };

      cmd_sub_ = create_subscription<std_msgs::msg::String>(
          "/torque_cascade_pid/command", 10,
          [handle_cmd](const std_msgs::msg::String::SharedPtr msg)
          { handle_cmd(msg->data); });

      legacy_cmd_sub_ = create_subscription<std_msgs::msg::String>(
          "/adaptive_lqr/command", 10,
          [handle_cmd](const std_msgs::msg::String::SharedPtr msg)
          { handle_cmd(msg->data); });

      dist_force_sub_ = create_subscription<std_msgs::msg::Float64>(
          "/disturbance_force_y", 10,
          [this](const std_msgs::msg::Float64::SharedPtr msg)
          {
            current_dist_force_ = msg->data;
            last_dist_force_time_ = now();
          });

      robot_mode_sub_ = create_subscription<std_msgs::msg::String>(
          "/robot_mode", 10,
          [this](const std_msgs::msg::String::SharedPtr msg)
          {
            if (msg->data == "emergency" || msg->data == "x" || msg->data == "X")
            {
              control_enabled_ = false;
              publish_wheel_torque(0.0);
              reset_pid_state();
              RCLCPP_WARN(get_logger(), "[Torque-PID] Emergency stop engaged");
            }
            else if (msg->data == "balance" || msg->data == "b" || msg->data == "B")
            {
              control_enabled_ = true;
              reset_pid_state();
              RCLCPP_INFO(get_logger(), "[Torque-PID] Balance mode restored");
            }
          });

      if (log_enabled_)
      {
        open_log_file();
      }

      last_time_ = now();
      start_time_ = now();
      timer_ = create_wall_timer(
          5ms, std::bind(&TorqueCascadePIDController::control_loop, this));

      RCLCPP_INFO(
          get_logger(),
          "TorqueCascadePIDController initialized: mode=%s, k_x=%.2f (disabled in 3-loop baseline), "
          "rate_limit=%.1f Nm/s (%s), torque_limits: total=±%.1f Nm, wheel=±%.1f Nm, "
          "hip range=[%.2f, %.2f] m, target=%.2f m, log=%s",
          stage_mode_.c_str(), k_x_, rate_limit_u_,
          (rate_limit_u_ <= 0.0 ? "DISABLED" : "ENABLED"),
          total_torque_max_, wheel_torque_max_,
          height_min_, height_max_,
          target_height_, log_path_.c_str());
    }

    ~TorqueCascadePIDController() override
    {
      publish_wheel_torque(0.0);
      if (log_file_.is_open())
      {
        log_file_.close();
      }
    }

  private:
    static constexpr std::array<NominalGeomPoint, 5> geom_table_{{{0.3000, -0.0276264, 0.3592154},
                                                                  {0.3500, -0.0258066, 0.4016316},
                                                                  {0.4000, -0.0234056, 0.4443432},
                                                                  {0.4500, -0.0203401, 0.4872441},
                                                                  {0.5000, -0.0164269, 0.5302655}}};

    void open_log_file()
    {
      log_file_.open(log_path_, std::ios::out | std::ios::trunc);
      if (!log_file_.is_open())
      {
        RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", log_path_.c_str());
        return;
      }
      // Header matches specification
      log_file_ << "time,height,base_link_height,p,p_0,delta_p,v,v_ref,v_error,"
                << "pitch,pitch_rate,theta_eq,theta_ref,theta_error,theta_dot_ref,theta_dot_error,"
                << "v_p,v_i,v_d,theta_p,theta_d,rate_p,rate_d,"
                << "u_raw,u_rate_limited,u_clamped,tau_left,tau_right,"
                << "is_saturated,is_rate_limited,control_enabled,"
                << "force_active,force_value,"
                << "kp_v,ki_v,kd_v,kp_theta,kd_theta,kp_rate,kd_rate,k_x,"
                << "rate_limit_u,total_torque_max,wheel_torque_max,p_0_latched\n";
    }

    double base_link_height_offset() const
    {
      return base_to_hip_height_ + wheel_radius_;
    }

    double base_link_height() const
    {
      return current_height_ + base_link_height_offset();
    }

    void reset_pid_state()
    {
      v_integral_ = 0.0;
      v_error_prev_ = 0.0;
      theta_error_prev_ = 0.0;
      rate_error_prev_ = 0.0;
      last_u_clamped_ = 0.0;
      delta_theta_ref_filt_ = 0.0;
      delta_theta_filt_init_ = false;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      tf2::Quaternion q(
          msg->orientation.x, msg->orientation.y,
          msg->orientation.z, msg->orientation.w);
      double roll = 0.0, pitch_unused = 0.0, yaw_unused = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch_unused, yaw_unused);

      // Robot pitches about the URDF X-axis. Forward lean is positive.
      pitch_ = -roll;
      const double raw_rate = -msg->angular_velocity.x;

      if (!pitch_rate_filter_init_)
      {
        pitch_rate_ = raw_rate;
        pitch_rate_filter_init_ = true;
      }
      else
      {
        pitch_rate_ = low_pass_filter(raw_rate, pitch_rate_, 0.10);
      }

      last_imu_time_ = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                           ? rclcpp::Time(msg->header.stamp)
                           : now();
      imu_received_ = true;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      bool found_004 = false, found_007 = false;
      for (size_t i = 0; i < msg->name.size(); ++i)
      {
        if (msg->name[i] == "link_004_joint")
        {
          if (i < msg->position.size())
            wheel_004_pos_ = msg->position[i];
          if (i < msg->velocity.size())
            wheel_004_vel_ = msg->velocity[i];
          found_004 = true;
        }
        else if (msg->name[i] == "link_007_joint")
        {
          if (i < msg->position.size())
            wheel_007_pos_ = msg->position[i];
          if (i < msg->velocity.size())
            wheel_007_vel_ = msg->velocity[i];
          found_007 = true;
        }
      }
      if (!(found_004 && found_007))
        return;

      last_joint_time_ = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                             ? rclcpp::Time(msg->header.stamp)
                             : now();

      if (!wheel_origin_set_)
      {
        wheel_004_origin_ = wheel_004_pos_;
        wheel_007_origin_ = wheel_007_pos_;
        wheel_origin_set_ = true;
        p_ = 0.0;
      }

      const double raw_v = -wheel_radius_ * 0.5 * (wheel_004_vel_ + wheel_007_vel_);
      if (!v_filter_init_)
      {
        v_ = raw_v;
        v_filter_init_ = true;
      }
      else
      {
        v_ = low_pass_filter(raw_v, v_, 0.05);
      }

      p_ = -wheel_radius_ * 0.5 * ((wheel_004_pos_ - wheel_004_origin_) + (wheel_007_pos_ - wheel_007_origin_));
    }

    double update_height(double dt)
    {
      previous_height_ = current_height_;
      const double step = leg_transition_speed_ * dt;
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
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {solution.theta_hip, solution.theta_knee,
                  solution.theta_hip, solution.theta_knee};
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
      if (h <= geom_table_.front().height)
      {
        return -std::atan2(geom_table_.front().y_com, geom_table_.front().z_com);
      }
      if (h >= geom_table_.back().height)
      {
        return -std::atan2(geom_table_.back().y_com, geom_table_.back().z_com);
      }
      for (size_t i = 0; i + 1 < geom_table_.size(); ++i)
      {
        const auto &low = geom_table_[i];
        const auto &high = geom_table_[i + 1];
        if (h >= low.height && h <= high.height)
        {
          const double r = (h - low.height) / (high.height - low.height);
          const double y = lerp(low.y_com, high.y_com, r);
          const double z = lerp(low.z_com, high.z_com, r);
          return -std::atan2(y, z);
        }
      }
      return -std::atan2(geom_table_[2].y_com, geom_table_[2].z_com);
    }

    CascadePIDGains interpolate_gains(double h) const
    {
      const double s = clamp_value((h - height_min_) / (height_max_ - height_min_), 0.0, 1.0);
      CascadePIDGains g;
      g.kp_rate = lerp(gains_low_.kp_rate, gains_high_.kp_rate, s);
      g.kd_rate = lerp(gains_low_.kd_rate, gains_high_.kd_rate, s);
      g.kp_theta = lerp(gains_low_.kp_theta, gains_high_.kp_theta, s);
      g.kd_theta = lerp(gains_low_.kd_theta, gains_high_.kd_theta, s);
      g.kp_v = lerp(gains_low_.kp_v, gains_high_.kp_v, s);
      g.ki_v = lerp(gains_low_.ki_v, gains_high_.ki_v, s);
      g.kd_v = lerp(gains_low_.kd_v, gains_high_.kd_v, s);
      return g;
    }

    void control_loop()
    {
      const auto current_time = now();
      if (last_control_sim_time_.nanoseconds() != 0)
      {
        const double sim_elapsed = (current_time - last_control_sim_time_).seconds();
        if (sim_elapsed < 0.0)
        {
          RCLCPP_WARN(get_logger(), "Clock rollback detected (%.4fs). Resetting time baselines.", sim_elapsed);
          last_control_sim_time_ = current_time;
          last_time_ = current_time;
          reset_pid_state();
          return;
        }
        if (sim_elapsed < 0.0045)
        {
          return;
        }
      }

      double raw_dt = last_time_.nanoseconds() != 0 ? (current_time - last_time_).seconds() : 0.005;
      last_time_ = current_time;
      last_control_sim_time_ = current_time;

      double dt = raw_dt;
      if (dt <= 0.0001 || dt > 0.05)
      {
        dt = 0.005;
      }

      if (!imu_received_ || !wheel_origin_set_ || !control_enabled_)
      {
        publish_leg_pose();
        publish_wheel_torque(0.0);
        return;
      }

      balance_ready_elapsed_ += dt;
      if (balance_ready_elapsed_ >= startup_hold_time_)
      {
        update_height(dt);
      }
      publish_leg_pose();

      // 1. Height-scheduled gains and nominal equilibrium angle
      const CascadePIDGains gains = interpolate_gains(current_height_);
      const double theta_eq = compute_theta_eq(current_height_);

      // The fixed observation origin is latched only after the startup hold and
      // only after pitch, pitch rate, and longitudinal velocity have all stayed
      // inside the valid-balance domain continuously.  It is never changed by
      // height commands, force pulses, or mode changes.
      const bool valid_balance =
          std::isfinite(p_) && std::abs(pitch_ - theta_eq) <= 0.10 &&
          std::abs(pitch_rate_) <= 0.20 && std::abs(v_) <= 0.05;
      if (balance_ready_elapsed_ >= startup_hold_time_ && valid_balance)
      {
        valid_balance_elapsed_ += dt;
      }
      else
      {
        valid_balance_elapsed_ = 0.0;
      }
      if (!p_0_latched_ && valid_balance_elapsed_ >= 0.50)
      {
        p_0_ = p_;
        p_0_latched_ = true;
        RCLCPP_INFO(
            get_logger(),
            "[Torque-PID] Fixed origin latched once: p_0 = %.4f m (immutable for this trial)",
            p_0_);
      }

      // 2. Velocity loop: v_ref is fixed at zero for the static baseline.  The
      // optional pulse is an explicit Stage-C excitation, not position feedback.
      double v_ref = 0.0;
      const bool velocity_loop_enabled = (stage_mode_ == "normal" || stage_mode_ == "stage_c");
      if (velocity_loop_enabled && velocity_disturbance_step_ != 0.0)
      {
        const double t_sim = (current_time - start_time_).seconds();
        if (t_sim >= velocity_disturbance_start_time_ &&
            t_sim < (velocity_disturbance_start_time_ + velocity_disturbance_duration_))
        {
          v_ref += velocity_disturbance_step_;
        }
      }
      const double e_v = velocity_loop_enabled ? (v_ref - v_) : 0.0;
      const double v_p = velocity_loop_enabled ? gains.kp_v * e_v : 0.0;
      const double v_d = velocity_loop_enabled ? gains.kd_v * ((e_v - v_error_prev_) / dt) : 0.0;
      v_error_prev_ = e_v;

      // Anti-windup conditional integration
      const double delta_theta_pre = v_p + (gains.ki_v * v_integral_) + v_d;
      bool v_saturated = (delta_theta_pre >= delta_theta_limit_ && e_v > 0.0) ||
                         (delta_theta_pre <= -delta_theta_limit_ && e_v < 0.0);
      if (velocity_loop_enabled && !v_saturated && gains.ki_v > 1e-6)
      {
        v_integral_ += e_v * dt;
        v_integral_ = clamp_value(v_integral_, -delta_theta_limit_ / gains.ki_v, delta_theta_limit_ / gains.ki_v);
      }
      const double v_i = gains.ki_v * v_integral_;

      double delta_theta_ref = 0.0;
      if (velocity_loop_enabled)
      {
        const double raw_delta = clamp_value(v_p + v_i + v_d, -delta_theta_limit_, delta_theta_limit_);
        if (!delta_theta_filt_init_)
        {
          delta_theta_ref_filt_ = raw_delta;
          delta_theta_filt_init_ = true;
        }
        else
        {
          delta_theta_ref_filt_ = low_pass_filter(raw_delta, delta_theta_ref_filt_, 0.10);
        }
        delta_theta_ref = delta_theta_ref_filt_;
      }

      // 3. Attitude angle loop: e_theta = theta_ref - theta
      double theta_ref = theta_eq;
      if (velocity_loop_enabled)
      {
        theta_ref = theta_eq + delta_theta_ref;
      }
      if (stage_mode_ == "stage_b" && attitude_disturbance_step_ != 0.0)
      {
        const double t_sim = (current_time - start_time_).seconds();
        if (t_sim >= disturbance_step_start_time_ &&
            t_sim < (disturbance_step_start_time_ + disturbance_step_duration_))
        {
          theta_ref += attitude_disturbance_step_;
        }
      }
      const double e_theta = theta_ref - pitch_;

      // Safety fall check
      if (std::abs(pitch_ - theta_eq) > max_pitch_error_)
      {
        control_enabled_ = false;
        publish_wheel_torque(0.0);
        reset_pid_state();
        RCLCPP_ERROR(get_logger(), "Pitch error %.3f rad exceeds limit %.3f rad; controller disabled.",
                     pitch_ - theta_eq, max_pitch_error_);
        return;
      }

      const bool attitude_loop_enabled = (stage_mode_ != "stage_a");
      const double theta_p = attitude_loop_enabled ? gains.kp_theta * e_theta : 0.0;
      const double theta_d = attitude_loop_enabled ? -gains.kd_theta * pitch_rate_ : 0.0;
      theta_error_prev_ = e_theta;

      double theta_dot_ref = attitude_loop_enabled ? clamp_value(theta_p + theta_d, -3.2, 3.2) : 0.0;
      if (rate_disturbance_step_ != 0.0)
      {
        const double t_sim = (current_time - start_time_).seconds();
        if (t_sim >= rate_disturbance_start_time_ &&
            t_sim < (rate_disturbance_start_time_ + rate_disturbance_duration_))
        {
          theta_dot_ref += rate_disturbance_step_;
        }
      }

      // 4. Pitch rate loop: e_theta_dot = theta_dot_ref - theta_dot
      const double e_theta_dot = theta_dot_ref - pitch_rate_;
      const double rate_p = gains.kp_rate * e_theta_dot;
      const double rate_d = gains.kd_rate * ((e_theta_dot - rate_error_prev_) / dt);
      rate_error_prev_ = e_theta_dot;

      const double u_raw = rate_p + rate_d;

      // Rate limiting: if rate_limit_u_ > 0, clamp |Delta u| <= rate_limit_u * dt
      double u_rate_limited = u_raw;
      bool is_rate_limited = false;
      if (rate_limit_u_ > 0.0)
      {
        const double max_du = rate_limit_u_ * dt;
        u_rate_limited = clamp_value(u_raw, last_u_clamped_ - max_du, last_u_clamped_ + max_du);
        is_rate_limited = std::abs(u_rate_limited - u_raw) > 1e-4;
      }

      // Total torque clamp: [-total_torque_max_, +total_torque_max_] Nm
      const double u_clamped = clamp_value(u_rate_limited, -total_torque_max_, total_torque_max_);
      const bool is_saturated = std::abs(u_clamped - u_rate_limited) > 1e-4;
      last_u_clamped_ = u_clamped;

      // 5. Actuator torque allocation: tau_left = tau_right = clamp(+0.5 * u, -wheel_torque_max_, wheel_torque_max_)
      // Note on sign convention:
      // When leaning forward (theta > theta_eq), e_theta < 0 -> theta_dot_ref < 0 -> u < 0.
      // In Gazebo, negative wheel effort drives wheels forward to catch the forward lean.
      // Therefore tau_each must have the same sign as u: tau_each = +0.5 * u.
      const double tau_each = clamp_value(0.5 * u_clamped, -wheel_torque_max_, wheel_torque_max_);
      publish_wheel_torque(tau_each);

      // External disturbance force tracking
      bool force_active = false;
      double force_value = 0.0;
      if (last_dist_force_time_.nanoseconds() != 0)
      {
        const double dist_elapsed = (current_time - last_dist_force_time_).seconds();
        if (dist_elapsed >= 0.0 && dist_elapsed < 0.25 && std::abs(current_dist_force_) > 1e-3)
        {
          force_active = true;
          force_value = current_dist_force_;
        }
      }

      // Logging
      if (log_file_.is_open())
      {
        const double t_sim = (current_time - start_time_).seconds();
        const double delta_p = p_ - p_0_;
        log_file_ << std::fixed << std::setprecision(6)
                  << t_sim << ','
                  << current_height_ << ','
                  << base_link_height() << ','
                  << p_ << ','
                  << p_0_ << ','
                  << delta_p << ','
                  << v_ << ','
                  << v_ref << ','
                  << e_v << ','
                  << pitch_ << ','
                  << pitch_rate_ << ','
                  << theta_eq << ','
                  << theta_ref << ','
                  << e_theta << ','
                  << theta_dot_ref << ','
                  << e_theta_dot << ','
                  << v_p << ','
                  << v_i << ','
                  << v_d << ','
                  << theta_p << ','
                  << theta_d << ','
                  << rate_p << ','
                  << rate_d << ','
                  << u_raw << ','
                  << u_rate_limited << ','
                  << u_clamped << ','
                  << tau_each << ','
                  << tau_each << ','
                  << (is_saturated ? 1 : 0) << ','
                  << (is_rate_limited ? 1 : 0) << ','
                  << (control_enabled_ ? 1 : 0) << ','
                  << (force_active ? 1 : 0) << ','
                  << force_value << ','
                  << gains.kp_v << ','
                  << gains.ki_v << ','
                  << gains.kd_v << ','
                  << gains.kp_theta << ','
                  << gains.kd_theta << ','
                  << gains.kp_rate << ','
                  << gains.kd_rate << ','
                  << k_x_ << ','
                  << rate_limit_u_ << ','
                  << total_torque_max_ << ','
                  << wheel_torque_max_ << ','
                  << (p_0_latched_ ? 1 : 0) << '\n';
        log_file_.flush();
      }

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[TORQUE-PID] H=%.3f p=%+.3fm p0=%+.3fm dp=%+.3fm v=%+.3f pitch=%+.4f eq=%+.4f "
          "v_ref=%+.3f th_ref=%+.4f u=%+.2fNm tau=%+.2fNm sat=%d rate_lim=%d force=%+.1fN",
          current_height_, p_, p_0_, (p_ - p_0_), v_, pitch_, theta_eq,
          v_ref, theta_ref, u_clamped, tau_each, is_saturated ? 1 : 0, is_rate_limited ? 1 : 0, force_value);
    }

    // Member variables
    bbot_kinematics::Kinematics kinematics_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_effort_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_force_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr legacy_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_control_sim_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_dist_force_time_{0, 0, RCL_ROS_TIME};

    bool imu_received_ = false;
    bool wheel_origin_set_ = false;
    bool control_enabled_ = true;
    bool v_filter_init_ = false;
    bool pitch_rate_filter_init_ = false;
    bool p_0_latched_ = false;

    double wheel_radius_ = 0.07;
    double wheel_torque_max_ = 10.0;
    double total_torque_max_ = 20.0;

    double height_min_ = 0.30;
    double height_max_ = 0.50;
    double base_to_hip_height_ = 0.07;
    double startup_height_ = 0.36;
    double startup_hold_time_ = 2.0;
    double leg_transition_speed_ = 0.05;
    double target_height_ = 0.50;
    double current_height_ = 0.36;
    double previous_height_ = 0.36;
    double balance_ready_elapsed_ = 0.0;
    double valid_balance_elapsed_ = 0.0;

    double max_pitch_error_ = 0.50;
    double rate_limit_u_ = 0.0; // 0.0 disables rate limiting
    double last_u_clamped_ = 0.0;
    double delta_theta_limit_ = 0.055;

    std::string stage_mode_ = "normal";

    // State variables
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double p_ = 0.0;
    double p_0_ = 0.0;
    double v_ = 0.0;
    double current_dist_force_ = 0.0;
    double rate_disturbance_step_ = 0.0;
    double rate_disturbance_start_time_ = 0.6;
    double rate_disturbance_duration_ = 0.2;
    double attitude_disturbance_step_ = 0.0;
    double disturbance_step_start_time_ = 2.0;
    double disturbance_step_duration_ = 1.0;
    double velocity_disturbance_step_ = 0.0;
    double velocity_disturbance_start_time_ = 2.5;
    double velocity_disturbance_duration_ = 0.5;

    double wheel_004_pos_ = 0.0;
    double wheel_007_pos_ = 0.0;
    double wheel_004_vel_ = 0.0;
    double wheel_007_vel_ = 0.0;
    double wheel_004_origin_ = 0.0;
    double wheel_007_origin_ = 0.0;

    // PID state
    double v_integral_ = 0.0;
    double v_error_prev_ = 0.0;
    double theta_error_prev_ = 0.0;
    double rate_error_prev_ = 0.0;
    double delta_theta_ref_filt_ = 0.0;
    bool delta_theta_filt_init_ = false;

    // Gains
    CascadePIDGains gains_low_;
    CascadePIDGains gains_high_;
    double k_x_ = 0.0;

    // Logging
    bool log_enabled_ = true;
    std::string log_path_;
    std::ofstream log_file_;
  };

} // namespace bbot_balance_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bbot_balance_controller::TorqueCascadePIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
