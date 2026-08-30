#ifndef BBOT_TORQUE_CONTROL__TORQUE_MONITOR_HPP_
#define BBOT_TORQUE_CONTROL__TORQUE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <vector>
#include <string>

#include "bbot_kinematics/kinematics.hpp"

namespace bbot_torque_control
{

/// @brief 力矩监控节点
/// 订阅关节状态和IMU，实时计算并记录：
/// 1. 当前质心高度
/// 2. 重力补偿力矩（髋、膝）
/// 3. 轮毂平衡力矩估算
///
/// 发布话题:
///   /bbot/torque_monitor/gravity_torques (Float64MultiArray)
///     [0]=hip_torque, [1]=knee_torque, [2]=wheel_torque
///   /bbot/torque_monitor/com_height (Float64)
///
/// 数据记录到文件: torque_log_<timestamp>.csv
class TorqueMonitor : public rclcpp::Node
{
public:
  explicit TorqueMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TorqueMonitor() override;

private:
  // 订阅回调
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 主循环 (定时器)
  void control_loop();

  // 日志记录
  void write_log_header();
  void write_log_row(double t, double com_h,
                     double hip_t, double knee_t, double wheel_t,
                     double hip_angle, double knee_angle, double pitch);

  // 从 joint_states 中提取指定关节的角度
  double get_joint_position(
    const sensor_msgs::msg::JointState & js,
    const std::string & name, double default_val = 0.0) const;

  // ======== 订阅者 ========
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ======== 发布者 ========
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr com_state_pub_;

  // ======== 定时器 ========
  rclcpp::TimerBase::SharedPtr timer_;

  // ======== 运动学求解器 ========
  bbot_kinematics::Kinematics kinematics_;

  // ======== 最新传感器数据 ========
  double body_pitch_ = 0.0;
  double body_pitch_rate_ = 0.0;
  double hip_angle_ = 0.0;    // left_hip
  double knee_angle_ = 0.0;   // left_knee
  bool imu_received_ = false;
  bool joint_state_received_ = false;

  // ======== 数据记录 ========
  std::ofstream log_file_;
  std::string log_path_;
  bool log_enabled_ = true;
  double log_interval_ = 0.05;  // 20Hz 记录
  double last_log_time_ = -1.0;

  // ======== 参数 ========
  int pub_rate_hz_ = 100;  // 发布频率
  std::string imu_topic_ = "/imu";
  std::string joint_state_topic_ = "/joint_states";
};

}  // namespace bbot_torque_control

#endif  // BBOT_TORQUE_CONTROL__TORQUE_MONITOR_HPP_
