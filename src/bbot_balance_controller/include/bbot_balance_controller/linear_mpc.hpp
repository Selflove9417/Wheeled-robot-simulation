#pragma once

// 轮腿平衡定高度线性模型预测控制 (Linear MPC)。
//
// 1. 系统状态与输入定义：
//    X = [x_error, x_dot_error, pitch_error, pitch_rate]^T
//    u: 两轮总驱动力矩（+u 前进加速；实车映射为单轮力矩 tau_each = -0.5 * u）
//
// 2. 滚动时域优化问题 (QP)：
//    min_{U, s+, s-}  sum_{i=1..N} e_i^T Q e_i + e_N^T P e_N + sum_{j=0..N-1} R u_j^2
//                     + W (s+ + s-) + w2 (s+^2 + s-^2)
//    s.t.  e_i = Ad^i e0 + sum_{j<i} Ad^(i-1-j) Bd u_j      （系统动力学预测方程）
//          |u_j| <= u_bound                                   （执行器力矩硬约束）
//          e_theta(i) - s+ <= theta_limit                     （俯仰角软约束带，正向共享松弛）
//         -e_theta(i) - s- <= theta_limit                     （俯仰角软约束带，负向共享松弛）
//          s+ >= 0, s- >= 0
//
// 3. 核心机制（看四件事）：
//    - 预测矩阵提升 (Dense Lifting)：
//      通过 psi_ (自由响应) 和 theta_ (输入响应矩阵) 消去状态变量，将 N 步时域“抬升”为一个稠密 QP。
//    - 俯仰带共享松弛对 (s+, s-)：
//      倒立摆受扰一旦越出带宽，硬约束 QP 将无解；而无解的控制器无法输出力矩。
//      大惩罚权重 W 确保带宽内松弛严格为 0（等价硬约束），超限时如实上报并继续输出稳定力矩。
//    - 四级降级链 (Fallback Chain)：
//      kSolved (完整 QP 求解) -> kBandDropped (剔除俯仰带仅保执行器约束) -> kLqrGain (退化为饱和 LQR) -> kZeroTorque (零转矩保护)。
//    - 两个关键细节：
//      1) 约束行 1-范数归一化：消除力矩约束 (~1) 与预测俯仰系数 (~1e-5) 量级失衡，防止 KKT 矩阵病态与迭代爬行；
//      2) build_feasible_start(): 抬升松弛变量构造可行起点，确保 Active-Set 求解器永远从可行点出发。

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "bbot_balance_controller/dense_active_set_qp.hpp"

namespace bbot_balance_controller
{

enum class MpcFallbackStage : int
{
  kSolved = 0,       ///< QP 收敛，俯仰带约束保留在问题中。
  kBandDropped = 1,  ///< 去掉俯仰带约束行后 QP 收敛。
  kLqrGain = 2,      ///< QP 不可用：退化为单步 LQR u = -K e，再做限幅。
  kZeroTorque = 3,   ///< 状态非有限或未配置：输出零转矩。
};

inline const char *mpc_fallback_name(MpcFallbackStage stage)
{
  switch (stage)
  {
    case MpcFallbackStage::kSolved: return "solved";
    case MpcFallbackStage::kBandDropped: return "band_dropped";
    case MpcFallbackStage::kLqrGain: return "lqr_gain";
    case MpcFallbackStage::kZeroTorque: return "zero_torque";
  }
  return "unknown";
}

struct LinearMpcConfig
{
  int horizon = 20;
  double total_torque_limit = 20.0;   // Nm，模型输入的界
  double wheel_torque_limit = 10.0;   // Nm，单轮界（|u| <= 2 * 此值）
  double theta_error_limit = 0.20;    // rad，俯仰误差带的半宽
  double theta_slack_weight = 1.0e5;  // 每弧度带宽违反的罚系数
  double theta_slack_curvature = 1.0; // 保证松弛项 Hessian 正定
  bool use_theta_band = true;
  int max_qp_iterations = 300;
};

struct LinearMpcDiagnostics
{
  double u_applied = 0.0;        ///< 实际下发的第一步控制量（模型单位）
  double u_qp_raw = 0.0;         ///< 最终限幅前的 QP 第一步控制量
  double lqr_u = 0.0;            ///< 单步 LQR 参考输入 u = -K e
  double tau_each = 0.0;         ///< 变号均分后的单轮 effort
  double theta_slack_upper = 0.0;
  double theta_slack_lower = 0.0;
  double theta_predicted_max = 0.0;
  double theta_predicted_min = 0.0;
  double objective = 0.0;
  int theta_band_rows = 0;
  QpStatus initial_qp_status = QpStatus::kInvalidInput;
  int initial_qp_iterations = 0;
  double initial_primal_residual = 0.0;
  double initial_dual_residual = 0.0;
  double initial_stationarity = 0.0;
  int working_set_size = 0;
  int qp_iterations = 0;
  double qp_solve_time_us = 0.0;
  double qp_primal_residual = 0.0;
  double qp_dual_residual = 0.0;
  double qp_stationarity = 0.0;
  double qp_complementarity = 0.0;
  bool total_torque_saturated = false;
  bool wheel_torque_saturated = false;
  bool theta_band_binding = false;
  bool qp_warm_start_used = false;
  QpStatus qp_status = QpStatus::kInvalidInput;
  MpcFallbackStage fallback_stage = MpcFallbackStage::kZeroTorque;
};

class LinearMpc
{
public:
  static constexpr int kMaxHorizon = 40;

  int horizon() const { return horizon_; }
  bool configured() const { return configured_; }
  double torque_limit() const { return u_bound_; }
  const Eigen::RowVector4d &first_move_gain() const { return first_move_gain_; }
  const Eigen::VectorXd &plan() const { return plan_; }

  bool configure(const Eigen::Matrix4d &Ad, const Eigen::Vector4d &Bd, const Eigen::Matrix4d &Q,
    double R, const Eigen::Matrix4d &P, const Eigen::RowVector4d &lqr_gain,
    const LinearMpcConfig &config)
  {
    configured_ = false;
    const int n = config.horizon;
    if (n < 1 || n > kMaxHorizon || !(R > 0.0) || !(config.total_torque_limit > 0.0) ||
      !(config.wheel_torque_limit > 0.0) || !Ad.allFinite() || !Bd.allFinite() ||
      !Q.allFinite() || !P.allFinite())
    {
      return false;
    }

    horizon_ = n;
    config_ = config;
    lqr_gain_ = lqr_gain;
    // 两个执行器限制作用于同一标量输入，有效界取二者中较紧的一个
    u_bound_ = std::min(config.total_torque_limit, 2.0 * config.wheel_torque_limit);

    // 1. 构造状态预测提升矩阵：e = Psi * e0 + Theta * U
    //    - psi_ (4N x 4): 自由响应矩阵，由 Ad 的各次幂堆叠而成
    //    - theta_ (4N x N): 输入响应矩阵（下三角分块），第 (i, j) 块为 Ad^(i-1-j) * Bd
    std::vector<Eigen::Matrix4d> powers(n + 1);
    powers[0] = Eigen::Matrix4d::Identity();
    for (int step = 1; step <= n; ++step)
    {
      powers[step] = powers[step - 1] * Ad;
    }
    psi_.resize(4 * n, 4);
    theta_.resize(4 * n, n);
    for (int i = 1; i <= n; ++i)
    {
      psi_.block(4 * (i - 1), 0, 4, 4) = powers[i];
      for (int j = 0; j < i; ++j)
      {
        theta_.block(4 * (i - 1), j, 4, 1) = powers[i - 1 - j] * Bd;
      }
      for (int j = i; j < n; ++j)
      {
        theta_.block(4 * (i - 1), j, 4, 1).setZero();
      }
    }

    // 2. 状态加权块对角阵 Q_block 与终端代价 P
    block_cost_.resize(4 * n, 4 * n);
    block_cost_.setZero();
    for (int i = 1; i <= n; ++i)
    {
      block_cost_.block(4 * (i - 1), 4 * (i - 1), 4, 4) = Q;
    }
    block_cost_.bottomRightCorner(4, 4) += P;

    // 3. 二次型 Hessian 矩阵：H = 2 * (Theta^T * Q_block * Theta + R_block) + 2 * curvature * I
    const int variables = n + 2;
    hessian_.resize(variables, variables);
    hessian_.setZero();
    hessian_.topLeftCorner(n, n) = 2.0 * (theta_.transpose() * block_cost_ * theta_);
    for (int j = 0; j < n; ++j)
    {
      hessian_(j, j) += 2.0 * R;
    }
    const double curvature = std::max(1.0e-9, config.theta_slack_curvature);
    hessian_(n, n) = 2.0 * curvature;
    hessian_(n + 1, n + 1) = 2.0 * curvature;

    // 验证理论第一步无约束增益 u(0) = -first_move_gain * e0（在时域收敛时贴合 DARE 增益 K）
    const Eigen::MatrixXd coupling = theta_.transpose() * block_cost_ * psi_;
    const Eigen::MatrixXd inverted = hessian_.topLeftCorner(n, n).inverse();
    first_move_gain_.setZero();
    for (int basis = 0; basis < 4; ++basis)
    {
      Eigen::Vector4d unit = Eigen::Vector4d::Zero();
      unit(basis) = 1.0;
      first_move_gain_(basis) = -(inverted * (2.0 * coupling * unit))(0);
    }

    const int rows = 2 * n + (config_.use_theta_band ? 2 * n : 0) + 2;
    constraints_.resize(rows, variables);
    constraints_.setZero();
    bounds_.resize(rows);
    bounds_.setZero();
    gradient_.resize(variables);
    gradient_.setZero();
    gradient_box_.resize(variables);
    gradient_box_.setZero();
    warm_.resize(variables);
    warm_.setZero();
    feasible_start_.resize(variables);
    feasible_start_.setZero();
    warm_box_.resize(variables);
    warm_box_.setZero();
    feasible_box_.resize(variables);
    feasible_box_.setZero();
    plan_.resize(n);
    plan_.setZero();
    box_rows_ = 2 * n;
    band_start_row_ = 2 * n;

    // 4. 构造力矩箱约束：-u_bound <= u_j <= u_bound
    int row = 0;
    for (int j = 0; j < n; ++j)
    {
      constraints_.row(row).setZero();
      constraints_(row, j) = 1.0;
      bounds_(row) = u_bound_;
      ++row;
      constraints_.row(row).setZero();
      constraints_(row, j) = -1.0;
      bounds_(row) = u_bound_;
      ++row;
    }
    band_row_scale_.assign(static_cast<std::size_t>(n), 1.0);
    if (config_.use_theta_band)
    {
      // 5. 构造俯仰软约束带，按 1-范数归一化均衡：
      //    消除力矩约束 (~1) 与后段俯仰响应 (~1e-5) 量级失衡，防止 KKT 矩阵病态导致工作集迭代爬行
      for (int i = 1; i <= n; ++i)
      {
        const Eigen::RowVectorXd band = theta_.row(4 * (i - 1) + 2);
        const double norm = band.lpNorm<1>();
        const double scale = (norm > 1.0e-15) ? 1.0 / norm : 1.0;
        band_row_scale_[static_cast<std::size_t>(i - 1)] = scale;
        constraints_.row(row).setZero();
        constraints_.block(row, 0, 1, n) = band * scale;
        constraints_(row, n) = -scale;
        bounds_(row) = config_.theta_error_limit * scale;
        ++row;
        constraints_.row(row).setZero();
        constraints_.block(row, 0, 1, n) = -band * scale;
        constraints_(row, n + 1) = -scale;
        bounds_(row) = config_.theta_error_limit * scale;
        ++row;
      }
      constraints_.row(row).setZero();
      constraints_(row, n) = -1.0;
      bounds_(row) = 0.0;
      ++row;
      constraints_.row(row).setZero();
      constraints_(row, n + 1) = -1.0;
      bounds_(row) = 0.0;
      ++row;
    }
    else
    {
      constraints_.row(row).setZero();
      bounds_(row) = 0.0;
      ++row;
      constraints_.row(row).setZero();
      bounds_(row) = 0.0;
      ++row;
    }

    qp_.reset(new DenseActiveSetQp(variables, rows));
    QpTolerances tolerances;
    tolerances.max_iterations = std::max(1, config.max_qp_iterations);
    qp_->set_tolerances(tolerances);
    configured_ = true;
    return true;
  }

  /// 求解一个 MPC 周期。`error_state` 为 [x_error, x_dot_err, pitch_error,
  /// 滚动优化单步求解，按四级降级链输出力矩：
  /// solved -> band_dropped -> lqr_gain -> zero_torque
  double solve(const Eigen::Vector4d &error_state, LinearMpcDiagnostics &out)
  {
    out = LinearMpcDiagnostics();
    out.lqr_u = -lqr_gain_.dot(error_state);

    // 状态异常或未配置时直接输出零转矩 (zero_torque)
    if (!configured_ || !error_state.allFinite())
    {
      out.fallback_stage = MpcFallbackStage::kZeroTorque;
      out.qp_status = QpStatus::kInvalidInput;
      warm_.setZero();
      plan_.setZero();
      return 0.0;
    }

    const int n = horizon_;
    free_response_ = psi_ * error_state;
    gradient_.setZero();
    gradient_.head(n) = 2.0 * theta_.transpose() * block_cost_ * free_response_;
    if (config_.use_theta_band)
    {
      gradient_(n) = config_.theta_slack_weight;
      gradient_(n + 1) = config_.theta_slack_weight;
    }

    // 依据自由响应平移俯仰带上下界
    if (config_.use_theta_band)
    {
      for (int i = 1; i <= n; ++i)
      {
        const double offset = free_response_(4 * (i - 1) + 2);
        const double scale = band_row_scale_[static_cast<std::size_t>(i - 1)];
        bounds_(band_start_row_ + 2 * (i - 1)) = (config_.theta_error_limit - offset) * scale;
        bounds_(band_start_row_ + 2 * (i - 1) + 1) = (config_.theta_error_limit + offset) * scale;
      }
    }

    build_feasible_start();

    // 阶段 1：求解包含俯仰带的完整 QP (solved)
    QpStatus status = qp_->solve(hessian_, gradient_, constraints_, bounds_, &warm_,
      &feasible_start_);
    out.initial_qp_status = status;
    out.initial_qp_iterations = qp_->diagnostics().iterations;
    out.initial_primal_residual = qp_->diagnostics().primal_residual;
    out.initial_dual_residual = qp_->diagnostics().dual_residual;
    out.initial_stationarity = qp_->diagnostics().stationarity;
    if (status == QpStatus::kConverged)
    {
      out.fallback_stage = MpcFallbackStage::kSolved;
    }
    else
    {
      // 阶段 2（降级）：去掉俯仰带，仅保留转矩限幅再次求解 (band_dropped)
      gradient_box_.setZero();
      gradient_box_.head(n) = gradient_.head(n);
      warm_box_.setZero();
      warm_box_.head(n) = warm_.head(n).cwiseMin(u_bound_).cwiseMax(-u_bound_);
      feasible_box_.setZero();
      status = qp_->solve(hessian_, gradient_box_, constraints_.topRows(box_rows_),
        bounds_.head(box_rows_), &warm_box_, &feasible_box_);
      out.fallback_stage = (status == QpStatus::kConverged) ? MpcFallbackStage::kBandDropped :
        MpcFallbackStage::kLqrGain;
    }
    copy_solver_report(status, out);

    // 阶段 3（降级）：QP 均未收敛时退化为单步 LQR 并限幅 (lqr_gain)
    double applied = 0.0;
    if (out.fallback_stage == MpcFallbackStage::kLqrGain)
    {
      applied = clamp_value(out.lqr_u);
      plan_.setZero();
      warm_.setZero();
      out.theta_predicted_max = 0.0;
      out.theta_predicted_min = 0.0;
    }
    else
    {
      const Eigen::VectorXd solution = qp_->solution();
      out.u_qp_raw = solution(0);
      plan_ = solution.head(n);
      out.theta_slack_upper = (config_.use_theta_band &&
        out.fallback_stage == MpcFallbackStage::kSolved) ? solution(n) : 0.0;
      out.theta_slack_lower = (config_.use_theta_band &&
        out.fallback_stage == MpcFallbackStage::kSolved) ? solution(n + 1) : 0.0;
      applied = clamp_value(out.u_qp_raw);
      report_band(out);
      warm_.head(n - 1) = plan_.segment(1, n - 1);
      warm_(n - 1) = plan_(n - 1);
      warm_(n) = out.theta_slack_upper;
      warm_(n + 1) = out.theta_slack_lower;
    }
    if (!std::isfinite(applied))
    {
      applied = 0.0;
      out.fallback_stage = MpcFallbackStage::kZeroTorque;
    }

    out.u_applied = applied;
    out.total_torque_saturated = (std::abs(applied) >= u_bound_ - 1.0e-6);
    out.tau_each = -0.5 * applied;
    out.wheel_torque_saturated =
      (std::abs(out.tau_each) >= config_.wheel_torque_limit - 1.0e-6);
    out.tau_each = std::max(-config_.wheel_torque_limit,
      std::min(config_.wheel_torque_limit, out.tau_each));
    return out.u_applied;
  }

private:
  double clamp_value(double value) const
  {
    return std::max(-u_bound_, std::min(u_bound_, value));
  }

  /// 构造严格可行的初始迭代点：
  /// 力矩计划限制在 [-u_bound, u_bound]，抬升松弛变量覆盖预测的最大俯仰误差，
  /// 保证 QP 求解器永远从可行点启动。
  void build_feasible_start()
  {
    const int n = horizon_;
    feasible_start_.setZero();
    feasible_start_.head(n) = warm_.head(n).cwiseMin(u_bound_).cwiseMax(-u_bound_);
    double upper_need = 0.0;
    double lower_need = 0.0;
    if (config_.use_theta_band)
    {
      for (int i = 1; i <= n; ++i)
      {
        const double predicted =
          theta_.row(4 * (i - 1) + 2).dot(feasible_start_.head(n)) + free_response_(4 * (i - 1) + 2);
        upper_need = std::max(upper_need, predicted - config_.theta_error_limit);
        lower_need = std::max(lower_need, -predicted - config_.theta_error_limit);
      }
    }
    feasible_start_(n) = std::max(0.0, upper_need);
    feasible_start_(n + 1) = std::max(0.0, lower_need);
  }

  void report_band(LinearMpcDiagnostics &out)
  {
    const int n = horizon_;
    double maximum = 0.0;
    double minimum = 0.0;
    bool binding = false;
    for (int i = 1; i <= n; ++i)
    {
      const double predicted =
        theta_.row(4 * (i - 1) + 2).dot(plan_) + free_response_(4 * (i - 1) + 2);
      maximum = (i == 1) ? predicted : std::max(maximum, predicted);
      minimum = (i == 1) ? predicted : std::min(minimum, predicted);
      if (std::abs(predicted) >= config_.theta_error_limit - 1.0e-4)
      {
        binding = true;
      }
    }
    out.theta_predicted_max = maximum;
    out.theta_predicted_min = minimum;
    out.theta_band_binding = binding || (out.theta_slack_upper > 1.0e-9) ||
      (out.theta_slack_lower > 1.0e-9);
  }

  void copy_solver_report(QpStatus status, LinearMpcDiagnostics &out)
  {
    const QpDiagnostics &diagnostics = qp_->diagnostics();
    out.qp_status = status;
    out.qp_iterations = diagnostics.iterations;
    out.qp_solve_time_us = diagnostics.solve_time_us;
    out.qp_primal_residual = diagnostics.primal_residual;
    out.qp_dual_residual = diagnostics.dual_residual;
    out.qp_stationarity = diagnostics.stationarity;
    out.qp_complementarity = diagnostics.complementarity;
    out.working_set_size = diagnostics.working_set_size;
    out.qp_warm_start_used = diagnostics.warm_start_used;
    out.objective = diagnostics.objective;
    out.theta_band_rows = config_.use_theta_band ? 2 * horizon_ : 0;
  }

  std::vector<double> band_row_scale_;  // 1 / ||theta 行||_1，约束行均衡化
  int horizon_ = 0;
  int box_rows_ = 0;
  int band_start_row_ = 0;
  double u_bound_ = 20.0;
  bool configured_ = false;
  LinearMpcConfig config_;
  Eigen::RowVector4d lqr_gain_ = Eigen::RowVector4d::Zero();
  Eigen::RowVector4d first_move_gain_ = Eigen::RowVector4d::Zero();
  Eigen::MatrixXd psi_;
  Eigen::MatrixXd theta_;
  Eigen::MatrixXd block_cost_;
  Eigen::MatrixXd hessian_;
  Eigen::VectorXd free_response_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd gradient_box_;
  Eigen::VectorXd warm_;
  Eigen::VectorXd feasible_start_;
  Eigen::VectorXd warm_box_;
  Eigen::VectorXd feasible_box_;
  Eigen::MatrixXd constraints_;
  Eigen::VectorXd bounds_;
  Eigen::VectorXd plan_;
  std::unique_ptr<DenseActiveSetQp> qp_;
};

}  // namespace bbot_balance_controller
