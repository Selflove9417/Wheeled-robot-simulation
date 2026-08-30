#ifndef BBOT_TORQUE_CONTROL__HEIGHT_CONTROLLER_HPP_
#define BBOT_TORQUE_CONTROL__HEIGHT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <vector>
#include <string>
#include <cmath>

#include "bbot_kinematics/kinematics.hpp"

namespace bbot_torque_control
{

/// @brief 简易PID控制器 (用于节点内部)
struct SimplePID
{
  double P, I, D;
  double integral = 0.0;
  double prev_error = 0.0;
  double output_limit = 1.0;
  double integral_limit = 1.0;

  double update(double error, double dt)
  {
    // 梯形积分 (抗积分饱和)
    integral += 0.5 * (error + prev_error) * dt;
    integral = std::max(-integral_limit, std::min(integral_limit, integral));

    // 微分
    double derivative = (dt > 1e-6) ? (error - prev_error) / dt : 0.0;

    // PID 输出
    double output = P * error + I * integral + D * derivative;
    output = std::max(-output_limit, std::min(output_limit, output));

    prev_error = error;
    return output;
  }

  void reset()
  {
    integral = 0.0;
    prev_error = 0.0;
  }
};

/// @brief 高度控制器节点
///
/// 控制策略:
///   1. 接收目标高度指令 (支持平滑过渡)
///   2. 通过 IK 计算目标关节角度
///   3. 计算重力前馈力矩 (通过雅可比转置)
///   4. 叠加 PD 位置反馈力矩
///   5. 发布关节力矩指令
///
/// 发布话题:
///   /leg_effort_controller/commands (Float64MultiArray)
///     [0]=left_hip, [1]=left_knee, [2]=right_hip, [3]=right_knee
///
/// 订阅话题:
///   /bbot/height_controller/target_height (Float64)  - 目标质心高度
///   /joint_states (JointState)
///   /imu (Imu)
class HeightController : public rclcpp::Node
{
public:
  explicit HeightController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 订阅回调
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void target_height_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // 主控制循环
  void control_loop();

  // 高度插值: 使用余弦缓动实现平滑过渡
  double smooth_height_transition(double current_time);

  // 从 joint_states 中提取指定关节的角度
  double get_joint_position(
    const sensor_msgs::msg::JointState & js,
    const std::string & name, double default_val = 0.0) const;

  // ======== 订阅者 ========
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;

  // ======== 发布者 ========
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

  // ======== 定时器 ========
  rclcpp::TimerBase::SharedPtr timer_;

  // ======== 运动学求解器 ========
  bbot_kinematics::Kinematics kinematics_;

  // ======== PID 控制器 ========
  SimplePID hip_pid_;
  SimplePID knee_pid_;

  // ======== 最新传感器数据 ========
  double body_pitch_ = 0.0;
  double hip_angle_left_ = 0.0;
  double knee_angle_left_ = 0.0;
  double hip_angle_right_ = 0.0;
  double knee_angle_right_ = 0.0;
  bool imu_received_ = false;
  bool joint_state_received_ = false;

  // ======== 高度控制状态 ========
  double current_height_ = 0.55;        // 当前质心高度 [m]
  double target_height_ = 0.55;         // 目标质心高度 [m]
  double height_transition_start_ = 0.55;  // 过渡起始高度
  double transition_start_time_ = 0.0;  // 过渡开始时间
  double transition_duration_ = 1.5;    // 过渡持续时间 [s]
  bool transition_active_ = false;      // 过渡是否活跃

  // ======== 关节力矩 (最后计算值，用于调试) ========
  double cmd_hip_torque_ = 0.0;
  double cmd_knee_torque_ = 0.0;

  // ======== 控制参数 ========
  int pub_rate_hz_ = 100;
  double gravity_feedforward_gain_ = 1.0;  // 重力前馈增益 (1.0 = 完全补偿)
  bool use_feedforward_ = true;            // 是否启用前馈
  bool enable_control_ = true;             // 总使能开关
};

}  // namespace bbot_torque_control

#endif  // BBOT_TORQUE_CONTROL__HEIGHT_CONTROLLER_HPP_
