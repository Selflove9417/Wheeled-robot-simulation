#pragma once

// 轮腿倒立摆被控对象线性模型与 DARE 求解器。
//
// 1. 状态与输入定义：
//    X = [x_error, x_dot, pitch_error, pitch_rate]^T
//    dot(X) = A X + B u
//    u: 两轮总驱动转矩（+u 驱动机器人向前加速）
//
// 2. 核心功能（看三件事）：
//    - continuous_matrices(): 根据连杆几何与动力学推导连续时间矩阵 A (4x4) 和 B (4x1)
//    - zoh_discretize(): 零阶保持 (ZOH) 精确离散化，生成 5 ms 控制周期的 (Ad, Bd)
//    - solve_dare(): 求解离散 Riccati 方程，计算终端代价 P 与 LQR 增益 K
//      * 核心设计动机：MPC 必须包含终端代价 P，否则有限时域闭环不稳定（谱半径 1.014 > 1）

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

    // 机构几何偏移：轮轴相对髋关节在 Y 方向的前向偏置 [m]
    constexpr double kHipYOffsetAheadOfAxle = 0.01137221;

    // 各连杆绕旋转轴的转动惯量 [kg m^2]
    constexpr double kWheelInertia = 0.006481;
    constexpr double kBodyInertia = 0.159013 * (9.5 / 14.0);
    constexpr double kThighInertia = 0.017921;
    constexpr double kShankInertia = 0.013130;

    // 各连杆在局部坐标系下的质心偏置与关节相对向量 (y, z)
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

    // 悬挂体等效单摆模型参数（机身+腿部等效为倒立摆，车轮折算平动与转动惯量）
    struct SuspendedBody
    {
      double y_com = 0.0;          // 悬挂质心相对轮轴的前向距离 [m]
      double z_com = 0.0;          // 悬挂质心相对轮轴的竖直距离 [m]
      double length = 0.0;         // 等效摆长 sqrt(y^2 + z^2) [m]
      double inertia = 0.0;        // 悬挂体绕自身质心的转动惯量 [kg m^2]
      double mass = 0.0;           // 悬挂体总质量（机身 + 大腿 + 小腿）[kg]
      double effective_mass = 0.0; // 有效平动质量（悬挂质量 + 车轮质量 + 轮电机惯量折算 m_wheel + J/R^2）[kg]
      double m2 = 0.0;             // 绕轴总转动惯量：inertia + mass * length^2
      double m3 = 0.0;             // 质心一阶质量矩：mass * length
      double gravitational = 0.0;  // 重力刚度项：mass * g * length
    };

    inline Vec2 rotate(double angle, const Vec2 &v)
    {
      const double c = std::cos(angle);
      const double s = std::sin(angle);
      return Vec2{c * v.y - s * v.z, s * v.y + c * v.z};
    }

    /// 逆运动学：根据给定的髋-轴垂直高度计算髋关节与膝关节目标角度。
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

    /// 正运动学与动力学聚合：根据当前腿高推导悬挂体综合质心位置、等效摆长与惯量。
    inline SuspendedBody suspended_body(double hip_axle_height, const bbot_kinematics::RobotParams &p)
    {
      double q_hip = 0.0;
      double q_knee = 0.0;
      leg_angles(hip_axle_height, p, q_hip, q_knee);

      // 计算各连杆在轮轴坐标系下的空间位置
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

      // 平行轴定理累加各连杆绕悬挂体质心的转动惯量
      auto distance_sq = [&body](const Vec2 &point)
      {
        const double dy = point.y - body.y_com;
        const double dz = point.z - body.z_com;
        return dy * dy + dz * dz;
      };
      body.inertia = kBodyInertia + m_body * distance_sq(p_body) +
                     2.0 * (kThighInertia + m_thigh * distance_sq(p_thigh)) +
                     2.0 * (kShankInertia + m_shank * distance_sq(p_shank));

      // 车轮贡献：车轮平动质量加上两个轮毂电机折算的转动惯量 (J / R^2)
      const double m_wheel = p.m0 * 0.5;
      body.effective_mass = mass + 2.0 * m_wheel + 2.0 * kWheelInertia / (p.wheel_radius * p.wheel_radius);
      body.m2 = body.inertia + mass * body.length * body.length;
      body.m3 = mass * body.length;
      body.gravitational = mass * p.g * body.length;
      return body;
    }

    /// 标称平衡俯仰角：使悬挂体质心处于轮轴正上方（-atan2(y_com, z_com)，IMU 前倾为正）。
    inline double equilibrium_pitch(const SuspendedBody &body)
    {
      return -std::atan2(body.y_com, body.z_com);
    }

    /// 在平衡点处线性化的连续时间状态空间矩阵：
    ///   d/dt [x_e, x_dot, pitch_e, pitch_dot]^T = A * X + B * u
    /// 其中 u 为双轮总驱动力矩（+u 前向加速）。
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

    /// 采用缩放平方法（Scaling and Squaring）与 Taylor 级数计算矩阵指数 expm()。
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

    /// 零阶保持 (ZOH) 精确离散化：将连续系统 (A, B) 转换为周期 ts 的离散系统 (Ad, Bd)。
    /// 利用矩阵指数恒等式：exp([A, B; 0, 0] * ts) = [Ad, Bd; 0, I]。
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

    /// 计算矩阵谱半径 rho = max(|lambda_i|)，用于验证闭环系统 (Ad - Bd * K) 是否渐近稳定 (rho < 1)。
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

    /// 求解离散 Lyapunov 方程：P = A' P A + C
    /// 将 4x4 矩阵方程按列展平为 16x16 线性方程组 (I - A' \otimes A') vec(P) = vec(C) 进行 LU 求解。
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

    /// 采用 Kleinman 策略迭代求解离散时间代数 Riccati 方程 (DARE)，计算终端代价 P 与 LQR 增益 K：
    ///   1. 代价评估（解 Lyapunov 方程）：P_{k+1} = Lyapunov(Ad - Bd K_k, Q + K_k' R K_k)
    ///   2. 策略更新：K_{k+1} = (R + Bd' P_{k+1} Bd)^-1 Bd' P_{k+1} Ad
    ///
    /// 设计重点：
    /// - 为什么必须求 P：P 作为有限时域 MPC 的终端代价；若不加 P，有限时域闭环不稳定（谱半径 1.014 > 1）。
    /// - 为什么用策略迭代：普通值迭代收敛极慢（需 ~1000 轮）且易数值发散；策略迭代 3~6 轮即可达到 1e-13 容差。
    /// - k_initial：稳定化初始策略（由高度调度表插值提供）。
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
          // 对称化：消除 Lyapunov 数值舍入误差，保证二次型 P 严格对称
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

    /// 根据 Riccati 解 P 计算离散 LQR 状态反馈增益 K，控制律形式为 u = -K X。
    inline Eigen::RowVector4d lqr_gain(const Eigen::Matrix4d &Ad, const Eigen::Vector4d &Bd,
                                       const Eigen::Matrix4d &P, double R)
    {
      const double denominator = R + Bd.transpose() * P * Bd;
      return (Bd.transpose() * P * Ad) / denominator;
    }

  } // namespace lqr_plant
} // namespace bbot_balance_controller
