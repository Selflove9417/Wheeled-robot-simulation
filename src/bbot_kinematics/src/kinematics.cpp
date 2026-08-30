#include "bbot_kinematics/kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace bbot_kinematics
{

Kinematics::Kinematics(const RobotParams & params)
: params_(params)
{
}

// ======================== 正运动学 (FK) ========================

void Kinematics::compute_absolute_angles(
  double body_pitch, double hip_angle, double knee_angle,
  double & theta_body, double & theta_thigh, double & theta_shank) const
{
  double phi1_0 = std::atan2(-0.29348091, 0.06220095);
  double phi2_0 = std::atan2(0.28210870, 0.19553796);

  theta_body = body_pitch;
  theta_thigh = phi1_0 + hip_angle - body_pitch;
  theta_shank = phi2_0 + hip_angle + knee_angle - body_pitch;
}

double Kinematics::calculate_com_height(
  double body_pitch,
  double hip_angle,
  double knee_angle) const
{
  double z_knee, z_hip, z_body, z_c1, z_c2, x_com, z_com;
  forward_kinematics(body_pitch, hip_angle, knee_angle,
                     z_knee, z_hip, z_body, z_c1, z_c2, x_com, z_com);
  return z_com;
}

void Kinematics::forward_kinematics(
  double body_pitch, double hip_angle, double knee_angle,
  double & z_knee, double & z_hip, double & z_body,
  double & z_c1, double & z_c2,
  double & x_com, double & z_com) const
{
  const auto & p = params_;

  double phi1_0 = std::atan2(-0.29348091, 0.06220095);
  double phi2_0 = std::atan2(0.28210870, 0.19553796);

  double phi1 = phi1_0 + hip_angle - body_pitch;
  double phi2 = phi2_0 + hip_angle + knee_angle - body_pitch;

  double dz_knee = -p.l2 * std::cos(phi1);
  double dz_wheel = dz_knee - p.l1 * std::cos(phi2);

  // 以轮轴中心为 z=0
  z_hip = -dz_wheel;
  z_knee = z_hip + dz_knee;
  z_body = z_hip + p.l3 * std::cos(body_pitch);

  z_c1 = z_knee - p.x1c * std::cos(phi2);
  z_c2 = z_hip - p.x2c * std::cos(phi1);

  x_com = 0.0;
  // 离地机身高度
  z_com = z_hip + 0.07 + p.wheel_radius;
}

// ======================== 逆运动学 (IK) ========================

IKSolution Kinematics::inverse_kinematics(double target_z, double body_pitch) const
{
  const auto & p = params_;

  // target_z: 机身离地高度 [m]
  // 髋关节到机身原点高度偏移: 0.07m, 轮子半径: p.wheel_radius = 0.075m
  // 髋关节到轮轴距离在竖直方向的投影: dZ_down = target_z - (0.07 + p.wheel_radius)
  double dZ_down = target_z - (0.07 + p.wheel_radius);
  if (dZ_down < 0.10) dZ_down = 0.10;
  if (dZ_down > 0.60) dZ_down = 0.60;

  double target_dY = -0.01137221; // 真实 CAD 轮轴相对髋关节 Y 向偏置 (0.11363 - 0.1250)
  double d_sq = target_dY * target_dY + dZ_down * dZ_down;
  double d = std::sqrt(d_sq);

  double l_thigh = p.l2;  // 0.3000m
  double l_shank = p.l1;  // 0.34325m

  // 余弦定理求解大腿与小腿夹角 (内角 gamma)
  double cos_gamma = (l_thigh * l_thigh + l_shank * l_shank - d_sq) / (2.0 * l_thigh * l_shank);
  cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
  double gamma = std::acos(cos_gamma);

  double theta_d = std::atan2(target_dY, dZ_down);

  double cos_psi = (l_thigh * l_thigh + d_sq - l_shank * l_shank) / (2.0 * l_thigh * d);
  cos_psi = std::max(-1.0, std::min(1.0, cos_psi));
  double psi = std::acos(cos_psi);

  double phi1 = theta_d - psi;
  double phi2 = phi1 + (M_PI - gamma);

  double phi1_0 = std::atan2(-0.29348091, 0.06220095);
  double phi2_0 = std::atan2(0.28210870, 0.19553796);

  double q_hip = phi1 - phi1_0 + body_pitch;
  double q_knee = (phi2 - phi1) - (phi2_0 - phi1_0);

  IKSolution sol;
  sol.theta_hip = q_hip;
  sol.theta_knee = q_knee;
  sol.theta_shank = phi2;

  return sol;
}

// ======================== 雅可比矩阵 ========================

Jacobian2D Kinematics::compute_jacobian(
  double body_pitch,
  double hip_angle,
  double knee_angle) const
{
  const auto & p = params_;

  // 1. 绝对角度
  double theta_body = body_pitch;
  double theta_thigh = theta_body - hip_angle;
  double theta_shank = theta_thigh - knee_angle;

  // 2. 等效质量-长度
  double a1 = p.m1 * p.x1c + (p.m2 + p.m3) * p.l1;  // 小腿等效
  double b1 = p.m2 * p.x2c + p.m3 * p.l2;           // 大腿等效
  double c1_m = p.m3 * p.l3;                         // 机身等效

  // 3. 解析雅可比矩阵 (除以 M_total)
  // J = [∂x/∂θ_hip, ∂x/∂θ_knee, ∂x/∂θ_body]
  //     [∂z/∂θ_hip, ∂z/∂θ_knee, ∂z/∂θ_body]
  //
  // 推导:
  //   M * x_com = a1*sin(θ_shank) + b1*sin(θ_thigh) + c1_m*sin(θ_body)
  //   M * z_com = a1*cos(θ_shank) + b1*cos(θ_thigh) + c1_m*cos(θ_body)
  //
  //   ∂/∂θ_hip:  ∂θ_shank/∂θ_hip=-1, ∂θ_thigh/∂θ_hip=-1
  //   ∂/∂θ_knee: ∂θ_shank/∂θ_knee=-1, ∂θ_thigh/∂θ_knee=0
  //   ∂/∂θ_body: ∂θ_shank/∂θ_body=1,  ∂θ_thigh/∂θ_body=1, ∂θ_body/∂θ_body=1

  double sin_s = std::sin(theta_shank);
  double cos_s = std::cos(theta_shank);
  double sin_t = std::sin(theta_thigh);
  double cos_t = std::cos(theta_thigh);
  double sin_b = std::sin(theta_body);
  double cos_b = std::cos(theta_body);

  double inv_M = 1.0 / p.M_total;

  Jacobian2D J;

  // ∂x_com/∂θ_hip = (-a1*cos_s - b1*cos_t) / M
  J.Jx_hip = (-a1 * cos_s - b1 * cos_t) * inv_M;

  // ∂x_com/∂θ_knee = (-a1*cos_s) / M
  J.Jx_knee = (-a1 * cos_s) * inv_M;

  // ∂x_com/∂θ_body = (a1*cos_s + b1*cos_t + c1_m*cos_b) / M
  J.Jx_body = (a1 * cos_s + b1 * cos_t + c1_m * cos_b) * inv_M;

  // ∂z_com/∂θ_hip = (a1*sin_s + b1*sin_t) / M
  J.Jz_hip = (a1 * sin_s + b1 * sin_t) * inv_M;

  // ∂z_com/∂θ_knee = (a1*sin_s) / M
  J.Jz_knee = (a1 * sin_s) * inv_M;

  // ∂z_com/∂θ_body = (-a1*sin_s - b1*sin_t - c1_m*sin_b) / M
  J.Jz_body = (-a1 * sin_s - b1 * sin_t - c1_m * sin_b) * inv_M;

  return J;
}

// ======================== 重力矩计算 ========================

JointTorques Kinematics::compute_gravity_torques(
  double body_pitch,
  double hip_angle,
  double knee_angle) const
{
  const auto & p = params_;

  // 计算雅可比矩阵
  Jacobian2D J = compute_jacobian(body_pitch, hip_angle, knee_angle);

  // 重力在质心处的作用力 (世界坐标系，z 向上为正)
  // F_g = [0, -M_total * g]
  // τ = J^T * F_g = [∂z/∂θ * (-M_total*g)]
  //
  // τ_hip  = Jz_hip * (-M_total * g) = -M_total * g * Jz_hip
  // τ_knee = Jz_knee * (-M_total * g) = -M_total * g * Jz_knee
  // τ_body = Jz_body * (-M_total * g)  <-- 这是轮毂力矩的参考

  double Mg = p.M_total * p.g;

  JointTorques torques;
  torques.hip_torque = -Mg * J.Jz_hip;
  torques.knee_torque = -Mg * J.Jz_knee;
  torques.wheel_torque = -Mg * J.Jz_body;  // 轮毂参考力矩

  return torques;
}

JointTorques Kinematics::compute_gravity_torques_at_height(
  double target_height,
  double body_pitch) const
{
  // 先通过 IK 求解关节角
  IKSolution ik = inverse_kinematics(target_height, body_pitch);

  // 再计算重力矩
  return compute_gravity_torques(body_pitch, ik.theta_hip, ik.theta_knee);
}

void Kinematics::compute_torque_vs_height(
  double body_pitch,
  int num_points,
  double * heights,
  double * hip_torques,
  double * knee_torques) const
{
  const auto & p = params_;
  double h_min = p.L_MIN;
  double h_max = p.L_MAX;

  for (int i = 0; i < num_points; i++) {
    double ratio = static_cast<double>(i) / static_cast<double>(num_points - 1);
    double h = h_min + ratio * (h_max - h_min);

    JointTorques t = compute_gravity_torques_at_height(h, body_pitch);

    heights[i] = h;
    hip_torques[i] = t.hip_torque;
    knee_torques[i] = t.knee_torque;
  }
}

// ======================== 轮毂力矩估算 ========================

double Kinematics::estimate_balance_torque(
  double com_height, double body_pitch) const
{
  // 倒立摆模型: τ = M * g * h * sin(θ)
  return params_.M_total * params_.g * com_height * std::sin(body_pitch);
}

}  // namespace bbot_kinematics
