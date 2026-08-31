#ifndef BBOT_KINEMATICS__ROBOT_PARAMS_HPP_
#define BBOT_KINEMATICS__ROBOT_PARAMS_HPP_

#include <cmath>

namespace bbot_kinematics
{

/// @brief 机器人物理参数结构体
/// 所有长度单位: 米(m), 质量单位: 千克(kg)
struct RobotParams
{
  // ======================== 几何参数 ========================
  double wheel_radius = 0.07;      // 轮子半径 R
  double l1 = 0.34325;             // 小腿长度 LEG_L1 (shank)
  double l2 = 0.3000;              // 大腿长度 LEG_L2 (thigh)
  double l3 = 0.1240;              // 髋关节到机身质心的距离 LEG_L3
  double x1c = 0.1435;             // 小腿质心位置
  double x2c = 0.1385;             // 大腿质心位置
  double wheel_separation = 0.364; // 左右轮距

  // ======================== 质量参数 ========================
  // 注意: 使用双腿总质量 (平面模型等效), 与 URDF 一致
  double m0 = 4.00;    // 双轮总质量 (2 × 2.00kg)
  double m1 = 1.60;    // 双腿小腿总质量 (2 × 0.80kg)
  double m2 = 2.40;    // 双腿大腿总质量 (2 × 1.20kg)
  double m3 = 14.00;   // 机身质量 (body)
  double M_total;      // 总质量 (在构造时计算 = 22.0kg)

  // ======================== 质心高度范围 ========================
  double L_MIN = 0.25;  // 最低质心高度 (蹲下)
  double L_MAX = 0.45;  // 最高质心高度 (站直)

  // ======================== 关节限制 ========================
  double hip_limit_lower = -1.57;   // rad
  double hip_limit_upper = 1.57;
  double knee_limit_lower = -1.57;  // rad
  double knee_limit_upper = 1.57;

  // ======================== 力矩限制 ========================
  double wheel_torque_max = 10.0;    // Nm, 轮毂电机最大扭矩
  double hip_torque_max = 75.0;     // Nm, 髋关节电机最大扭矩
  double knee_torque_max = 60.0;    // Nm, 膝关节电机最大扭矩

  // ======================== 重力加速度 ========================
  double g = 9.81;  // m/s^2

  /// @brief 构造函数，自动计算总质量
  RobotParams()
  {
    M_total = m0 + m1 + m2 + m3;
  }
};

/// @brief 逆运动学求解结果
///
/// 运动学链: 轮轴 ─[l1:小腿]─→ 膝 ─[l2:大腿]─→ 髋 ─[l3]─→ 机身质心
/// 绝对角度 (从竖直向上=0):
///   θ_shank = body_pitch - θ_hip - θ_knee   (小腿绝对角)
///   θ_thigh = body_pitch - θ_hip            (大腿绝对角)
/// 闭合约束: body_pitch = θ_shank + θ_knee + θ_hip
///
/// URDF 约定 (关节轴均为 Y):
///   hip_urdf  = -θ_hip   (大腿相对于机身, 正=前倾)
///   knee_urdf = -θ_knee  (小腿相对于大腿, 正=前倾)
struct IKSolution
{
  double theta_shank; // θ₁: 小腿绝对角 (轮轴→膝, 竖直=0)
  double theta_knee;  // θ₂: 膝关节角 (大腿与小腿之间的夹角)
  double theta_hip;   // θ₃: 髋关节角 (机身与大腿之间的夹角, 由闭合约束反推)

  /// @brief 获取关节角的文字描述
  double hip_angle() const { return theta_hip; }
  double knee_angle() const { return theta_knee; }
};

/// @brief 关节力矩结构体
struct JointTorques
{
  double hip_torque;    // Nm, 髋关节力矩
  double knee_torque;   // Nm, 膝关节力矩
  double wheel_torque;  // Nm, 轮毂电机力矩 (平衡所需)
};

/// @brief 2D 雅可比矩阵 (2x3)
/// J = [Jx_hip,  Jx_knee,  Jx_body]
///     [Jz_hip,  Jz_knee,  Jz_body]
struct Jacobian2D
{
  double Jx_hip, Jx_knee, Jx_body;
  double Jz_hip, Jz_knee, Jz_body;
};

/// @brief 线性插值工具函数
inline double lerp(double start, double end, double ratio)
{
  return start + ratio * (end - start);
}

/// @brief 钳位函数
inline double clamp(double value, double min_val, double max_val)
{
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

/// @brief 将高度比例映射到 [0, 1] 范围
inline double height_ratio(double current_l, const RobotParams & p)
{
  return clamp((current_l - p.L_MIN) / (p.L_MAX - p.L_MIN), 0.0, 1.0);
}

}  // namespace bbot_kinematics

#endif  // BBOT_KINEMATICS__ROBOT_PARAMS_HPP_
