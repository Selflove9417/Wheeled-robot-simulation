#include "bbot_torque_control/height_controller.hpp"
#include <algorithm>
#include <cmath>

namespace bbot_torque_control
{

HeightController::HeightController(const rclcpp::NodeOptions & options)
: Node("height_controller", options)
{
  // ======== 声明参数 ========
  this->declare_parameter("pub_rate_hz", 100);
  this->declare_parameter("gravity_feedforward_gain", 1.0);
  this->declare_parameter("use_feedforward", true);
  this->declare_parameter("enable_control", true);
  this->declare_parameter("transition_duration", 1.5);

  // PD 参数
  this->declare_parameter("hip_pid.P", 80.0);
  this->declare_parameter("hip_pid.I", 2.0);
  this->declare_parameter("hip_pid.D", 4.0);
  this->declare_parameter("knee_pid.P", 80.0);
  this->declare_parameter("knee_pid.I", 2.0);
  this->declare_parameter("knee_pid.D", 4.0);

  // 默认目标高度
  this->declare_parameter("default_height", 0.55);

  pub_rate_hz_ = this->get_parameter("pub_rate_hz").as_int();
  gravity_feedforward_gain_ = this->get_parameter("gravity_feedforward_gain").as_double();
  use_feedforward_ = this->get_parameter("use_feedforward").as_bool();
  enable_control_ = this->get_parameter("enable_control").as_bool();
  transition_duration_ = this->get_parameter("transition_duration").as_double();
  target_height_ = this->get_parameter("default_height").as_double();
  current_height_ = target_height_;

  // 配置 PID
  hip_pid_.P = this->get_parameter("hip_pid.P").as_double();
  hip_pid_.I = this->get_parameter("hip_pid.I").as_double();
  hip_pid_.D = this->get_parameter("hip_pid.D").as_double();
  knee_pid_.P = this->get_parameter("knee_pid.P").as_double();
  knee_pid_.I = this->get_parameter("knee_pid.I").as_double();
  knee_pid_.D = this->get_parameter("knee_pid.D").as_double();

  hip_pid_.output_limit = 120.0;  // Nm, 髋关节力矩上限
  knee_pid_.output_limit = 120.0; // Nm
  hip_pid_.integral_limit = 40.0;
  knee_pid_.integral_limit = 40.0;

  // ======== 订阅 ========
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    std::bind(&HeightController::imu_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&HeightController::joint_state_callback, this, std::placeholders::_1));

  target_height_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bbot/height_controller/target_height", 10,
    std::bind(&HeightController::target_height_callback, this, std::placeholders::_1));

  // ======== 发布 ========
  effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/leg_effort_controller/commands", 10);

  debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/bbot/height_controller/debug", 10);

  // ======== 控制循环 ========
  auto period = std::chrono::milliseconds(1000 / pub_rate_hz_);
  timer_ = this->create_wall_timer(period, std::bind(&HeightController::control_loop, this));

  RCLCPP_INFO(this->get_logger(),
    "高度控制器已启动 [%d Hz], 默认高度: %.3f m, 前馈: %s",
    pub_rate_hz_, target_height_,
    use_feedforward_ ? "启用" : "禁用");
}

void HeightController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  double sin_pitch = 2.0 * (qw * qy - qx * qz);
  sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
  body_pitch_ = std::asin(sin_pitch);

  imu_received_ = true;
}

void HeightController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  hip_angle_left_ = get_joint_position(*msg, "link_002_joint", 0.0);
  knee_angle_left_ = get_joint_position(*msg, "link_003_joint", 0.0);
  hip_angle_right_ = get_joint_position(*msg, "link_005_joint", 0.0);
  knee_angle_right_ = get_joint_position(*msg, "link_006_joint", 0.0);

  joint_state_received_ = true;
}

void HeightController::target_height_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  double new_target = msg->data;
  // 钳位到机械范围
  const auto & p = kinematics_.get_params();
  new_target = bbot_kinematics::clamp(new_target, p.L_MIN, p.L_MAX);

  if (std::abs(new_target - target_height_) > 0.001) {
    RCLCPP_INFO(this->get_logger(),
      "收到新目标高度: %.3f -> %.3f m (过渡时间: %.1f s)",
      target_height_, new_target, transition_duration_);

    height_transition_start_ = current_height_;
    transition_start_time_ = this->now().seconds();
    transition_active_ = true;
    target_height_ = new_target;
  }
}

double HeightController::get_joint_position(
  const sensor_msgs::msg::JointState & js,
  const std::string & name, double default_val) const
{
  for (size_t i = 0; i < js.name.size(); i++) {
    if (js.name[i] == name) {
      return js.position[i];
    }
  }
  return default_val;
}

double HeightController::smooth_height_transition(double current_time)
{
  if (!transition_active_) {
    return target_height_;
  }

  double elapsed = current_time - transition_start_time_;

  if (elapsed >= transition_duration_) {
    transition_active_ = false;
    current_height_ = target_height_;
    return target_height_;
  }

  // 余弦缓动: 0->1 的光滑过渡
  double raw_progress = elapsed / transition_duration_;
  double smooth_progress = 0.5 * (1.0 - std::cos(raw_progress * M_PI));

  double h = height_transition_start_ +
    smooth_progress * (target_height_ - height_transition_start_);

  current_height_ = h;
  return h;
}

void HeightController::control_loop()
{
  if (!imu_received_ || !joint_state_received_) {
    return;
  }

  double now = this->now().seconds();
  double dt = (pub_rate_hz_ > 0) ? 1.0 / pub_rate_hz_ : 0.01;

  // ======== 1. 计算当前状态 ========
  double com_height = kinematics_.calculate_com_height(
    body_pitch_, hip_angle_left_, knee_angle_left_);

  // 目标高度 (带平滑过渡)
  double desired_height = smooth_height_transition(now);

  // ======== 2. 逆运动学: 目标关节角度 ========
  bbot_kinematics::IKSolution ik = kinematics_.inverse_kinematics(
    desired_height, 0.0);  // body_pitch=0 for upright

  // 注意: 这里需要将 IK 解适配到 URDF 的实际关节约定
  // IK 返回 theta_hip (大腿相对机身) 和 theta_knee (小腿相对大腿)
  // URDF 中：
  //   left_hip_joint: hip_angle (大腿相对于机身)
  //   left_knee_joint: knee_angle (小腿相对于大腿)
  double target_hip = ik.theta_hip;
  double target_knee = ik.theta_knee;

  // ======== 3. 重力前馈力矩 ========
  double ff_hip = 0.0, ff_knee = 0.0;
  if (use_feedforward_) {
    bbot_kinematics::JointTorques gravity = kinematics_.compute_gravity_torques(
      0.0, target_hip, target_knee);  // 在目标姿态下计算
    ff_hip = gravity.hip_torque * gravity_feedforward_gain_;
    ff_knee = gravity.knee_torque * gravity_feedforward_gain_;
  }

  // ======== 4. PD 反馈力矩 (位置误差 -> 力矩) ========
  double hip_error = target_hip - hip_angle_left_;
  double knee_error = target_knee - knee_angle_left_;
  double pd_hip = hip_pid_.update(hip_error, dt);
  double pd_knee = knee_pid_.update(knee_error, dt);

  // ======== 5. 总力矩 = 前馈 + 反馈 ========
  cmd_hip_torque_ = ff_hip + pd_hip;
  cmd_knee_torque_ = ff_knee + pd_knee;

  // 力矩限制
  const auto & p = kinematics_.get_params();
  cmd_hip_torque_ = bbot_kinematics::clamp(cmd_hip_torque_, -p.hip_torque_max, p.hip_torque_max);
  cmd_knee_torque_ = bbot_kinematics::clamp(cmd_knee_torque_, -p.knee_torque_max, p.knee_torque_max);

  // ======== 6. 发布力矩指令 ========
  if (enable_control_) {
    auto effort_msg = std_msgs::msg::Float64MultiArray();
    // 左腿 (轴 +X) 和右腿 (轴 -X) 物理对称
    effort_msg.data[0] = cmd_hip_torque_;     // link_002_joint (left_hip)
    effort_msg.data[1] = cmd_knee_torque_;    // link_003_joint (left_knee)
    effort_msg.data[2] = -cmd_hip_torque_;    // link_005_joint (right_hip, 负号映射)
    effort_msg.data[3] = -cmd_knee_torque_;   // link_006_joint (right_knee, 负号映射)
    effort_pub_->publish(effort_msg);
  }

  // ======== 7. 调试信息 ========
  auto debug_msg = std_msgs::msg::Float64MultiArray();
  debug_msg.data.resize(8);
  debug_msg.data[0] = com_height;
  debug_msg.data[1] = desired_height;
  debug_msg.data[2] = target_hip;
  debug_msg.data[3] = hip_angle_left_;
  debug_msg.data[4] = target_knee;
  debug_msg.data[5] = knee_angle_left_;
  debug_msg.data[6] = cmd_hip_torque_;
  debug_msg.data[7] = cmd_knee_torque_;
  debug_pub_->publish(debug_msg);

  // 定期打印状态
  static int print_cnt = 0;
  if (print_cnt++ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "高度: %.3f/%.3f m | 力矩: hip=%.1f knee=%.1f Nm | "
      "关节: hip=%.2f/%.2f knee=%.2f/%.2f rad | 过渡: %s",
      com_height, desired_height,
      cmd_hip_torque_, cmd_knee_torque_,
      hip_angle_left_, target_hip, knee_angle_left_, target_knee,
      transition_active_ ? "进行中" : "完成");
  }
}

}  // namespace bbot_torque_control

// 独立可执行文件入口
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bbot_torque_control::HeightController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
