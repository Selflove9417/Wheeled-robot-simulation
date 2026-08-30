#ifndef BBOT_KINEMATICS__KINEMATICS_HPP_
#define BBOT_KINEMATICS__KINEMATICS_HPP_

#include "bbot_kinematics/robot_params.hpp"

namespace bbot_kinematics
{

/// @brief 运动学求解器类
/// 提供正运动学(FK)、逆运动学(IK)、雅可比矩阵、重力矩等计算
class Kinematics
{
public:
  /// @brief 构造函数
  /// @param params 机器人物理参数，如果不提供则使用默认值
  explicit Kinematics(const RobotParams & params = RobotParams());

  /// @brief 更新物理参数
  void set_params(const RobotParams & params) { params_ = params; }

  /// @brief 获取当前参数
  const RobotParams & get_params() const { return params_; }

  // ======================== 正运动学 (FK) ========================

  /// @brief 计算实时质心高度 (CoM height)
  /// @param body_pitch  机身俯仰角 [rad] (IMU 测得)
  /// @param hip_angle   髋关节角 [rad] (大腿相对于机身)
  /// @param knee_angle  膝关节角 [rad] (小腿相对于大腿)
  /// @return 质心相对于轮轴的高度 [m]
  double calculate_com_height(
    double body_pitch,
    double hip_angle,
    double knee_angle) const;

  /// @brief 完整正运动学：计算各连杆位置和质心位置
  /// @param body_pitch  机身俯仰角 [rad]
  /// @param hip_angle   髋关节角 [rad]
  /// @param knee_angle  膝关节角 [rad]
  /// @param[out] z_knee  膝关节 z 坐标
  /// @param[out] z_hip   髋关节 z 坐标
  /// @param[out] z_body  机身质心 z 坐标
  /// @param[out] z_c1    小腿质心 z 坐标
  /// @param[out] z_c2    大腿质心 z 坐标
  /// @param[out] x_com   质心 x 坐标
  /// @param[out] z_com   质心 z 坐标
  void forward_kinematics(
    double body_pitch, double hip_angle, double knee_angle,
    double & z_knee, double & z_hip, double & z_body,
    double & z_c1, double & z_c2,
    double & x_com, double & z_com) const;

  // ======================== 逆运动学 (IK) ========================

  /// @brief 逆运动学求解
  /// 给定目标质心高度和机身俯仰角，计算所需的关节角度
  /// @param target_z    目标质心高度 [m]
  /// @param body_pitch  机身俯仰角 [rad]
  /// @return 逆解结果 (hip, knee, ankle 角度)
  IKSolution inverse_kinematics(double target_z, double body_pitch) const;

  // ======================== 雅可比矩阵 ========================

  /// @brief 计算质心雅可比矩阵 (解析法)
  /// J 将关节速度映射到质心速度: [vx_com, vz_com]^T = J * [dθ_hip, dθ_knee, dθ_body]^T
  /// @param body_pitch  机身俯仰角 [rad]
  /// @param hip_angle   髋关节角 [rad]
  /// @param knee_angle  膝关节角 [rad]
  /// @return 2x3 雅可比矩阵
  Jacobian2D compute_jacobian(
    double body_pitch,
    double hip_angle,
    double knee_angle) const;

  // ======================== 重力矩计算 ========================

  /// @brief 计算重力补偿力矩 (静态平衡时关节所需的力矩)
  /// 使用虚功原理: τ_g = J^T * [0, M_total * g]^T
  /// @param body_pitch  机身俯仰角 [rad]
  /// @param hip_angle   髋关节角 [rad]
  /// @param knee_angle  膝关节角 [rad]
  /// @return 髋关节和膝关节的重力补偿力矩 [Nm]
  JointTorques compute_gravity_torques(
    double body_pitch,
    double hip_angle,
    double knee_angle) const;

  /// @brief 计算在给定质心高度下所需的重力补偿力矩
  /// 先通过 IK 求解关节角，再计算重力矩
  /// @param target_height 目标质心高度 [m]
  /// @param body_pitch    机身俯仰角 [rad] (默认 0 = 直立)
  /// @return 各关节的重力补偿力矩 [Nm]
  JointTorques compute_gravity_torques_at_height(
    double target_height,
    double body_pitch = 0.0) const;

  /// @brief 遍历高度范围，计算每个高度下的重力补偿力矩
  /// @param body_pitch  机身俯仰角 [rad]
  /// @param num_points  采样点数
  /// @param[out] heights     高度数组 [m]
  /// @param[out] hip_torques 髋关节力矩数组 [Nm]
  /// @param[out] knee_torques 膝关节力矩数组 [Nm]
  void compute_torque_vs_height(
    double body_pitch,
    int num_points,
    double * heights,
    double * hip_torques,
    double * knee_torques) const;

  // ======================== 轮毂力矩估算 ========================

  /// @brief 估算轮毂电机平衡力矩
  /// 使用倒立摆模型: τ_wheel ≈ M_total * g * com_height * sin(body_pitch)
  /// @param com_height  当前质心高度 [m]
  /// @param body_pitch  机身俯仰角 [rad]
  /// @return 所需的轮毂力矩 [Nm]
  double estimate_balance_torque(double com_height, double body_pitch) const;

  // ======================== 正运动学：获取各连杆绝对角度 ========================

  /// @brief 计算各连杆的绝对角度 (相对于竖直方向)
  /// @param body_pitch  机身俯仰角 [rad]
  /// @param hip_angle   髋关节角 [rad]
  /// @param knee_angle  膝关节角 [rad]
  /// @param[out] theta_body   机身绝对角 [rad]
  /// @param[out] theta_thigh  大腿绝对角 [rad]
  /// @param[out] theta_shank  小腿绝对角 [rad]
  void compute_absolute_angles(
    double body_pitch, double hip_angle, double knee_angle,
    double & theta_body, double & theta_thigh, double & theta_shank) const;

private:
  RobotParams params_;
};

}  // namespace bbot_kinematics

#endif  // BBOT_KINEMATICS__KINEMATICS_HPP_
