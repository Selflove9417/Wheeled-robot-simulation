#pragma once

// 已部署 LQR 增益表使用的线性轮腿平衡模型。
//
// 本文件是 src/bbot_balance_controller/scripts/verify_lqr_model_and_sweep.py 中
// design_matrices() / discretize() / dare_gain() 的 C++ 移植；正是该脚本验证了
// 编译进 lqr_gain_scheduled_controller.cpp 与
// adaptive_lqr_balance_controller.cpp 的增益表。移植它（而非另造模型）保证了
// MPC 与 LQR 控制器使用同一组 A、B 和同一状态排序：
//
//   X = [x_error, x_dot, pitch_error, pitch_rate]^T
//   dot(X) = A X + B u,  u = 两轮总驱动转矩，+u 使机器人前进
//
// 质量、轮半径与重力取自 bbot_kinematics::RobotParams，使模型跟随 URDF 推导
// 的参数块而非一份拷贝。下方连杆坐标系向量是 Python 参考实现使用的精确
// URDF 关节系偏移（它们不在 RobotParams 中）。

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include "bbot_kinematics/robot_params.hpp"

namespace bbot_balance_controller
{
namespace lqr_plant
{

// 平面矢状链：轮轴 --[l1 小腿]--> 膝 --[l2 大腿]--> 髋。
// 来源：verify_lqr_model_and_sweep.py:37（CAD 中轮轴偏移在髋之前）。
constexpr double kHipYOffsetAheadOfAxle = 0.01137221;

// URDF 中关于轮/关节轴的 ixx 值。
// 来源：verify_lqr_model_and_sweep.py:43-46。
constexpr double kWheelInertia = 0.006481;
constexpr double kBodyInertia = 0.159013 * (9.5 / 14.0);
constexpr double kThighInertia = 0.017921;
constexpr double kShankInertia = 0.013130;

// 对应连杆系下精确的 URDF 二维向量 (y, z)。
// 来源：verify_lqr_model_and_sweep.py:55-59。
struct Vec2
{
  double y;
  double z;
};

constexpr Vec2 kKneeToHip{-0.29348091, -0.06220095};
constexpr Vec2 kThighCom{-0.13690699, -0.02116697};
constexpr Vec2 kAxleToKnee{0.28210870, -0.19553796};
constexpr Vec2 kShankCom{0.11538205, -0.08532288};
constexpr Vec2 kBodyCom{0.13261282 - 0.125, 0.05396677 + 0.07};

struct SuspendedBody
{
  double y_com = 0.0;      // 悬挂质心在轮轴前方 [m]
  double z_com = 0.0;      // 悬挂质心在轮轴上方 [m]
  double length = 0.0;     // 摆长 |com| [m]
  double inertia = 0.0;    // 悬挂部分绕自身质心的转动惯量 [kg m^2]
  double mass = 0.0;       // 悬挂质量（机身 + 大腿 + 小腿）[kg]
  double effective_mass = 0.0;  // 悬挂质量 + 车轮 + 折算车轮转动惯量 [kg]
  double m2 = 0.0;         // inertia + mass * length^2
  double m3 = 0.0;         // mass * length
  double gravitational = 0.0;  // mass * g * length
};

inline Vec2 rotate(double angle, const Vec2 &v)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return Vec2{c * v.y - s * v.z, s * v.y + c * v.z};
}

/// 给定髋高于轮轴的目标高度求关节角。对应 Python 的 ik_leg()，其两连杆解与
/// bbot_kinematics::Kinematics::inverse_kinematics() 相同，区别是本函数直接
/// 接收髋-轴高度而非 base_link 高度。
inline void leg_angles(double hip_axle_height, const bbot_kinematics::RobotParams &p,
  double &q_hip, double &q_knee)
{
  const double down = hip_axle_height;
  const double target_dy = -kHipYOffsetAheadOfAxle;
  const double d_sq = target_dy * target_dy + down * down;
  const double l_thigh = p.l2;
  const double l_shank = p.l1;

  double cos_gamma = (l_thigh * l_thigh + l_shank * l_shank - d_sq) / (2.0 * l_thigh * l_shank);
  cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
  const double gamma = std::acos(cos_gamma);
  const double theta_d = std::atan2(target_dy, down);

  double cos_psi = (l_thigh * l_thigh + d_sq - l_shank * l_shank) / (2.0 * l_thigh * std::sqrt(d_sq));
  cos_psi = std::max(-1.0, std::min(1.0, cos_psi));
  const double psi = std::acos(cos_psi);

  const double phi1 = theta_d - psi;
  const double phi2 = phi1 + (M_PI - gamma);
  const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
  const double phi2_0 = std::atan2(0.28210870, 0.19553796);

  q_hip = phi1 - phi1_0;
  q_knee = (phi2 - phi1) - (phi2_0 - phi1_0);
}

inline SuspendedBody suspended_body(double hip_axle_height, const bbot_kinematics::RobotParams &p)
{
  double q_hip = 0.0;
  double q_knee = 0.0;
  leg_angles(hip_axle_height, p, q_hip, q_knee);

  // knee = -R_shank * axle_to_knee, hip = knee - R_thigh * knee_to_hip
  const Vec2 r_axle_knee = rotate(q_hip + q_knee, kAxleToKnee);
  const Vec2 knee{-r_axle_knee.y, -r_axle_knee.z};
  const Vec2 r_knee_hip = rotate(q_hip, kKneeToHip);
  const Vec2 hip{knee.y - r_knee_hip.y, knee.z - r_knee_hip.z};

  const Vec2 r_shank_com = rotate(q_hip + q_knee, kShankCom);
  const Vec2 p_shank{knee.y + r_shank_com.y, knee.z + r_shank_com.z};
  const Vec2 r_thigh_com = rotate(q_hip, kThighCom);
  const Vec2 p_thigh{hip.y + r_thigh_com.y, hip.z + r_thigh_com.z};
  const Vec2 p_body{hip.y + kBodyCom.y, hip.z + kBodyCom.z};

  const double m_body = p.m3;
  const double m_thigh = p.m2 * 0.5;
  const double m_shank = p.m1 * 0.5;
  const double mass = m_body + 2.0 * m_thigh + 2.0 * m_shank;

  SuspendedBody body;
  body.y_com = (m_body * p_body.y + 2.0 * m_thigh * p_thigh.y + 2.0 * m_shank * p_shank.y) / mass;
  body.z_com = (m_body * p_body.z + 2.0 * m_thigh * p_thigh.z + 2.0 * m_shank * p_shank.z) / mass;
  body.length = std::hypot(body.y_com, body.z_com);
  body.mass = mass;

  auto distance_sq = [&body](const Vec2 &point)
  {
    const double dy = point.y - body.y_com;
    const double dz = point.z - body.z_com;
    return dy * dy + dz * dz;
  };
  body.inertia = kBodyInertia + m_body * distance_sq(p_body) +
    2.0 * (kThighInertia + m_thigh * distance_sq(p_thigh)) +
    2.0 * (kShankInertia + m_shank * distance_sq(p_shank));

  // 车轮贡献：车轮平动质量加上两个轮毂电机折算的转动惯量 (J / R^2)。
  const double m_wheel = p.m0 * 0.5;
  body.effective_mass = mass + 2.0 * m_wheel + 2.0 * kWheelInertia / (p.wheel_radius * p.wheel_radius);
  body.m2 = body.inertia + mass * body.length * body.length;
  body.m3 = mass * body.length;
  body.gravitational = mass * p.g * body.length;
  return body;
}

/// 标称平衡俯仰角（IMU 约定：前倾为正）。表达式同
/// adaptive_lqr_balance_controller.cpp:603。
inline double equilibrium_pitch(const SuspendedBody &body)
{
  return -std::atan2(body.y_com, body.z_com);
}

/// 在悬挂平衡点处线性化的连续时间矩阵。
inline void continuous_matrices(double hip_axle_height, const bbot_kinematics::RobotParams &p,
  Eigen::Matrix4d &A, Eigen::Vector4d &B, SuspendedBody *body_out = nullptr)
{
  const SuspendedBody body = suspended_body(hip_axle_height, p);
  if (body_out != nullptr)
  {
    *body_out = body;
  }
  const double determinant = body.effective_mass * body.m2 - body.m3 * body.m3;

  A.setZero();
  A(0, 1) = 1.0;
  A(1, 2) = -body.m3 * body.gravitational / determinant;
  A(2, 3) = 1.0;
  A(3, 2) = body.effective_mass * body.gravitational / determinant;

  B.setZero();
  B(1) = (body.m2 / p.wheel_radius + body.m3) / determinant;
  B(3) = -(body.effective_mass + body.m3 / p.wheel_radius) / determinant;
}

/// 用缩放平方法（scaling and squaring）的 Taylor 级数计算 expm()，再做精确的
/// 零阶保持离散化
///
///   exp([A B] * ts) = [Ad, Bd; 0, I]
///
/// 这是 Python 参考实现使用的恒等式（对增广矩阵取 scipy.linalg.expm）。
inline void matrix_exponential(const Eigen::MatrixXd &input, Eigen::MatrixXd &output)
{
  const int size = static_cast<int>(input.rows());
  double norm = input.cwiseAbs().colwise().sum().maxCoeff();
  if (!std::isfinite(norm) || norm <= 0.0)
  {
    norm = 1.0;
  }

  int scaling = 0;
  while (norm / std::pow(2.0, scaling) > 0.5 && scaling < 30)
  {
    ++scaling;
  }

  Eigen::MatrixXd term = Eigen::MatrixXd::Identity(size, size);
  Eigen::MatrixXd series = term;
  const Eigen::MatrixXd scaled = input / std::pow(2.0, scaling);
  for (int order = 1; order < 24; ++order)
  {
    term = term * scaled / static_cast<double>(order);
    series += term;
    if (term.cwiseAbs().maxCoeff() < 1.0e-18)
    {
      break;
    }
  }

  for (int square = 0; square < scaling; ++square)
  {
    series = series * series;
  }
  output = series;
}

inline void zoh_discretize(double ts, const Eigen::Matrix4d &A, const Eigen::Vector4d &B,
  Eigen::Matrix4d &Ad, Eigen::Vector4d &Bd)
{
  Eigen::MatrixXd augmented(5, 5);
  augmented.setZero();
  augmented.topLeftCorner(4, 4) = A;
  augmented.block(0, 4, 4, 1) = B;

  Eigen::MatrixXd exponential;
  matrix_exponential(augmented * ts, exponential);
  Ad = exponential.topLeftCorner(4, 4);
  Bd = exponential.col(4).head(4);
}

/// 闭环矩阵的谱半径，用于在策略喂给 Riccati 迭代之前确认它是稳定的。
inline double spectral_radius(const Eigen::Matrix4d &matrix)
{
  Eigen::EigenSolver<Eigen::Matrix4d> solver(matrix, false);
  if (solver.info() != Eigen::Success)
  {
    return std::numeric_limits<double>::infinity();
  }
  double radius = 0.0;
  for (int index = 0; index < matrix.rows(); ++index)
  {
    radius = std::max(radius, std::abs(solver.eigenvalues()(index)));
  }
  return radius;
}

/// 对稳定的 A 求解离散 Lyapunov 方程 P = A' P A + C。
///
/// 按元素以列主序堆叠 vec(P)[row + 4 * column]，得到 (I - M) vec(P) = vec(C)，
/// 其中 M[p + 4q, i + 4j] = at[p,i] * at[q,j]（at = A'）。不用 Eigen 的
/// KroneckerProduct：它在 unsupported 模块里，且按行主序索引分块。
inline bool solve_discrete_lyapunov(const Eigen::Matrix4d &a, const Eigen::Matrix4d &c,
  Eigen::Matrix4d &p)
{
  const Eigen::Matrix4d at = a.transpose();
  Eigen::MatrixXd system = Eigen::MatrixXd::Zero(16, 16);
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      const int output_index = row + 4 * column;
      system(output_index, output_index) = 1.0;
      for (int i = 0; i < 4; ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          system(output_index, i + 4 * j) -= at(row, i) * at(column, j);
        }
      }
    }
  }

  Eigen::VectorXd right(16);
  for (int column = 0; column < 4; ++column)
  {
    right.segment<4>(4 * column) = c.col(column);
  }

  Eigen::FullPivLU<Eigen::MatrixXd> lu(system);
  if (!lu.isInvertible())
  {
    return false;
  }
  const Eigen::VectorXd solved = lu.solve(right);
  if (!solved.allFinite())
  {
    return false;
  }
  for (int column = 0; column < 4; ++column)
  {
    p.col(column) = solved.segment<4>(4 * column);
  }
  return true;
}

/// 用 Kleinman 策略迭代求离散代数 Riccati 解。
///
///   P_{i+1} = Lyapunov(Ad - Bd K_i,  Q + K_i' R K_i)
///   K_{i+1} = (R + Bd' P_{i+1} Bd)^-1 Bd' P_{i+1} Ad
///
/// 在本对象上实测过普通值迭代 P <- Q + Ad' P Ad - ...：它缓慢爬向解约 1000
/// 轮后数值发散（已部署增益的闭环谱半径为 0.9993，残差每轮只缩小 1.4e-3，
/// 舍入误差远早于容差达成就占了上风）。策略迭代只需 3-6 轮。
///
/// k_initial 必须已使 (Ad - Bd k_initial) 稳定；它取自已部署增益表，该表已经
/// 本仓库的多速率扫描验证为稳定（verify_lqr_model_and_sweep.py:run_sweep）。
inline bool solve_dare(const Eigen::Matrix4d &Ad, const Eigen::Vector4d &Bd,
  const Eigen::Matrix4d &Q, double R, const Eigen::RowVector4d &k_initial, Eigen::Matrix4d &P,
  Eigen::RowVector4d &K, int max_iterations, int &iterations, double &residual)
{
  residual = std::numeric_limits<double>::infinity();
  const double radius = spectral_radius(Ad - Bd * k_initial);
  if (!(radius < 1.0) || !(R > 0.0))
  {
    iterations = 0;
    return false;
  }

  Eigen::RowVector4d policy = k_initial;
  for (int iteration = 1; iteration <= max_iterations; ++iteration)
  {
    Eigen::Matrix4d cost = Q;
    cost += (policy.transpose() * policy) * R;
    Eigen::Matrix4d current_p;
    if (!solve_discrete_lyapunov(Ad - Bd * policy, cost, current_p))
    {
      iterations = iteration;
      return false;
    }
    const Eigen::Vector4d right_hand_side = Bd.transpose() * current_p * Ad;
    const double denominator = R + Bd.transpose() * current_p * Bd;
    Eigen::RowVector4d next_policy = right_hand_side.transpose() / denominator;

    const double step = (next_policy - policy).cwiseAbs().maxCoeff();
    const double scale = std::max(1.0, next_policy.cwiseAbs().maxCoeff());
    policy = next_policy;
    P = current_p;
    residual = step;
    if (step <= 1.0e-13 * scale)
    {
      // 对称化：对称右端的 Lyapunov 解除舍入误差外是对称的，而 MPC 代价把
      // P 当作二次型使用。
      P = 0.5 * (P + P.transpose());
      K = policy;
      iterations = iteration;
      return true;
    }
  }
  K = policy;
  iterations = max_iterations;
  return false;
}

/// u = -K X，符号约定与已部署增益表一致。
inline Eigen::RowVector4d lqr_gain(const Eigen::Matrix4d &Ad, const Eigen::Vector4d &Bd,
  const Eigen::Matrix4d &P, double R)
{
  const double denominator = R + Bd.transpose() * P * Bd;
  return (Bd.transpose() * P * Ad) / denominator;
}

}  // namespace lqr_plant
}  // namespace bbot_balance_controller
