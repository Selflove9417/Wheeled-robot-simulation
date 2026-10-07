#pragma once

// 稠密严格凸二次规划 (QP) 求解器（标准 Active-Set 有效集法）：
//
// 1. 求解标准二次规划问题：
//      minimize    0.5 * x^T H x + g^T x
//      subject to  A x <= b                  (A 为 m x n，行向量为 a_i^T)
//
// 2. Active-Set 核心三步循环（solve_reduced -> 检查乘子 -> 检查步长）：
//    - 步骤 1【解等式约束子问题】：将工作集 (working set) 约束视为等式，解 KKT 系统求搜索方向 p 与乘子 mu
//    - 步骤 2【检查乘子（释放约束）】：若步长 p ≈ 0，检查乘子符号：
//        * 若 mu >= 0，已满足全部 KKT 条件，算法收敛退出；
//        * 若存在 mu_i < 0，释放阻碍下降的最负约束（结合 Maritz 防循环规则暂时禁用该行）。
//    - 步骤 3【检查步长（阻断约束）】：沿方向 p 寻找第一个触碰的外部约束（阻断约束 blocking）：
//        * 计算最大可行步长 alpha in (0, 1] 并更新 x <- x + alpha * p；
//        * 若 alpha < 1，将阻断约束加入工作集。
//
// 3. 适用前提与保证：
//    - H 必须对称正定（使用 LLT 分解严格验证）；
//    - 必须从可行点出发（依次探测 warm start、feasible start、零点，不可行直接报错拒绝）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

namespace bbot_balance_controller
{

  enum class QpStatus : int
  {
    kInvalidInput = 0,
    kConverged = 1,
    kMaxIterations = 2,
    kNumericalFailure = 3,
    kNotPositiveDefinite = 4,
    kNoFeasibleStart = 5,
  };

  inline const char *qp_status_name(QpStatus status)
  {
    switch (status)
    {
    case QpStatus::kConverged:
      return "converged";
    case QpStatus::kMaxIterations:
      return "max_iterations";
    case QpStatus::kNumericalFailure:
      return "numerical_failure";
    case QpStatus::kNotPositiveDefinite:
      return "not_positive_definite";
    case QpStatus::kNoFeasibleStart:
      return "no_feasible_start";
    case QpStatus::kInvalidInput:
      return "invalid_input";
    }
    return "unknown";
  }

  struct QpTolerances
  {
    double feasibility = 1.0e-8;          // 比例化：违反量 <= feasibility * 行尺度
    double absolute_feasibility = 1.0e-9; // 下限，以行的单位计
    double dual = 1.0e-6;                 // |lambda| 尺度：lambda_i >= -dual * scale
    double stationarity = 1.0e-6;         // 相对于 ||g||_inf 与 ||H x||_inf
    double step = 1.0e-10;                // "无移动"的绝对下限
    double step_relative = 1.0e-6;        // ||p||_inf <= step_relative * max(1,||x||_inf)
    double direction = 1.0e-12;           // a_i'p 超过此值才可能阻断步长
    int max_iterations = 200;
  };

  struct QpDiagnostics
  {
    QpStatus status = QpStatus::kInvalidInput;
    int iterations = 0;
    int working_set_size = 0;
    int variables = 0;
    int constraints = 0;
    int constraint_replacements = 0;
    double primal_residual = 0.0;        // max_i (a_i'x - b_i)，> 0 表示违反
    double primal_violation_ratio = 0.0; // max_i 违反量 / 行尺度，<= 1 即可行
    double dual_residual = 0.0;          // max_i max(0, -lambda_i)
    double stationarity = 0.0;           // ||H x + g + A'lambda||_inf
    double complementarity = 0.0;        // max_i |lambda_i (a_i'x - b_i)|
    double objective = 0.0;
    double solve_time_us = 0.0;
    bool warm_start_used = false;
  };

  class DenseActiveSetQp
  {
  public:
    DenseActiveSetQp(int max_variables, int max_constraints)
        : max_variables_(std::max(1, max_variables)),
          max_constraints_(std::max(1, max_constraints))
    {
      const int kkt_size = 2 * max_variables_ + 2;
      kkt_.resize(kkt_size, kkt_size);
      kkt_.setZero();
      rhs_.resize(kkt_size);
      rhs_.setZero();
      solved_.resize(kkt_size);
      h_sym_.resize(max_variables_, max_variables_);
      h_sym_.setZero();
      x_.resize(max_variables_);
      x_.setZero();
      gradient_.resize(max_variables_);
      gradient_.setZero();
      step_.resize(max_variables_);
      step_.setZero();
      lambda_.resize(max_constraints_);
      lambda_.setZero();
      working_lambda_.resize(max_variables_);
      working_lambda_.setZero();
      residual_.resize(max_variables_);
      residual_.setZero();
    }

    void set_tolerances(const QpTolerances &tolerances) { tolerances_ = tolerances; }
    const QpTolerances &tolerances() const { return tolerances_; }

    const Eigen::VectorXd &solution() const { return x_; }
    const Eigen::VectorXd &multipliers() const { return lambda_; }
    const QpDiagnostics &diagnostics() const { return diagnostics_; }
    const std::vector<int> &last_working_set() const { return working_; }

    QpStatus solve(
        const Eigen::MatrixXd &H,
        const Eigen::VectorXd &g,
        const Eigen::MatrixXd &A,
        const Eigen::VectorXd &b,
        const Eigen::VectorXd *x_warm = nullptr,
        const Eigen::VectorXd *x_feasible_start = nullptr)
    {
      const auto clock_start = std::chrono::steady_clock::now();
      diagnostics_ = QpDiagnostics();
      lambda_.setZero();

      const int n = static_cast<int>(H.rows());
      const int m = static_cast<int>(A.rows());
      diagnostics_.variables = n;
      diagnostics_.constraints = m;

      if (n <= 0 || m < 0 || n > max_variables_ || m > max_constraints_ ||
          H.cols() < n || g.size() < n || A.cols() < n || b.size() < m ||
          !H.topLeftCorner(n, n).allFinite() || !g.head(n).allFinite() ||
          !A.leftCols(n).topRows(m).allFinite() || !b.head(m).allFinite())
      {
        x_.setZero();
        return report(QpStatus::kInvalidInput, clock_start);
      }

      h_sym_.topLeftCorner(n, n) = 0.5 * (H.topLeftCorner(n, n) + H.topLeftCorner(n, n).transpose());
      Eigen::LLT<Eigen::MatrixXd> llt(h_sym_.topLeftCorner(n, n));
      if (llt.info() != Eigen::Success)
      {
        x_.head(n).setZero();
        return report(QpStatus::kNotPositiveDefinite, clock_start);
      }

      // 寻找可行初始点：优先热启动 x_warm，其次备选点 x_feasible_start，最后尝试零向量
      if (x_warm != nullptr && x_warm->size() == n && x_warm->allFinite() &&
          max_violation(*x_warm, A, b, n, m) <= 1.0)
      {
        x_.head(n) = *x_warm;
        diagnostics_.warm_start_used = true;
      }
      else if (x_feasible_start != nullptr && x_feasible_start->size() == n &&
               x_feasible_start->allFinite() &&
               max_violation(*x_feasible_start, A, b, n, m) <= 1.0)
      {
        x_.head(n) = *x_feasible_start;
      }
      else
      {
        x_.head(n).setZero();
        if (max_violation(x_.head(n), A, b, n, m) > 1.0)
        {
          return report(QpStatus::kNoFeasibleStart, clock_start);
        }
      }

      banned_.assign(static_cast<std::size_t>(m), 0);
      constraints_row_norm_.resize(static_cast<std::size_t>(m));
      for (int row = 0; row < m; ++row)
      {
        constraints_row_norm_[static_cast<std::size_t>(row)] = A.row(row).head(n).lpNorm<1>();
      }

      // 1. 将当前贴边约束加入初始工作集（热启动跳过迭代的关键）
      build_initial_working_set(A, b, n, m);

      // 2. 执行 Active-Set 主循环（三步循环：解子问题 -> 检查乘子 -> 检查步长）
      run_active_set(H, g, A, b, n, m);

      // 3. 校验最终 KKT 残差（原始可行性、对偶可行性、平稳性、互补松弛性）
      compute_residuals(A, b, g, n, m);
      const QpStatus verified = classify(m);
      if (verified == QpStatus::kConverged)
      {
        // 解的收敛性最终由 KKT 残差检验严格保证
        diagnostics_.status = QpStatus::kConverged;
      }
      else if (diagnostics_.status == QpStatus::kConverged)
      {
        diagnostics_.status = verified;
      }
      diagnostics_.solve_time_us = elapsed_us(clock_start);
      diagnostics_.working_set_size = static_cast<int>(working_.size());
      return diagnostics_.status;
    }

  private:
    /// Floating-point resolution of a_i'x - b_i for row `row` at the current x.
    double row_tolerance(int row, int n, const Eigen::VectorXd &b) const
    {
      const double row_scale = 1.0 + std::abs(b(row)) +
                               constraints_row_norm_[static_cast<std::size_t>(row)] * x_.head(n).cwiseAbs().maxCoeff();
      return std::max(tolerances_.absolute_feasibility, tolerances_.feasibility * row_scale);
    }

    bool negligible_step(int n) const
    {
      const double scale = std::max(1.0, x_.head(n).cwiseAbs().maxCoeff());
      return step_.head(n).cwiseAbs().maxCoeff() <=
             std::max(tolerances_.step, tolerances_.step_relative * scale);
    }

    QpStatus report(QpStatus status, const std::chrono::steady_clock::time_point &start)
    {
      diagnostics_.status = status;
      diagnostics_.solve_time_us = elapsed_us(start);
      return status;
    }

    double max_violation(const Eigen::VectorXd &x, const Eigen::MatrixXd &A,
                         const Eigen::VectorXd &b, int n, int m)
    {
      double worst = -std::numeric_limits<double>::infinity();
      for (int row = 0; row < m; ++row)
      {
        const double scale = 1.0 + std::abs(b(row)) + A.row(row).head(n).lpNorm<1>() * x.cwiseAbs().maxCoeff();
        worst = std::max(worst, (A.row(row).head(n).dot(x) - b(row)) /
                                    (tolerances_.feasibility * scale));
      }
      return worst; // 以行容差为单位，1.0 恰好在限制边界上
    }

    /// 识别初始可行点处已贴边的约束并加入工作集（热启动跳过大部分迭代的关键）。
    void build_initial_working_set(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, int n, int m)
    {
      working_.clear();
      for (int row = 0; row < m && static_cast<int>(working_.size()) < n; ++row)
      {
        if (A.row(row).head(n).dot(x_.head(n)) - b(row) > -row_tolerance(row, n, b))
        {
          add_working_row(row, n);
        }
      }
      std::sort(working_.begin(), working_.end());
    }

    void add_working_row(int row, int n)
    {
      if (static_cast<int>(working_.size()) >= n)
      {
        return;
      }
      working_.push_back(row);
      std::sort(working_.begin(), working_.end());
    }

    /// Active-Set 主循环：
    /// 1. 解等式约束子问题，计算搜索方向与乘子
    /// 2. 步长为零时检查乘子符号：均非负则收敛退出；存在负乘子则释放并按 Maritz 规则防循环
    /// 3. 沿搜索方向计算最大可行步长与阻断约束，更新迭代点并将阻断约束加入工作集
    void run_active_set(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                        const Eigen::MatrixXd &A, const Eigen::VectorXd &b, int n, int m)
    {
      diagnostics_.status = QpStatus::kMaxIterations;
      for (int iteration = 1; iteration <= tolerances_.max_iterations; ++iteration)
      {
        diagnostics_.iterations = iteration;
        gradient_.head(n) = H.topLeftCorner(n, n) * x_.head(n) + g.head(n);

        // 1. 解等式约束子问题
        ReducedSolve reduced = ReducedSolve::kDescent;
        int retries = 0;
        while (true)
        {
          reduced = solve_reduced(A, b, n, m);
          if (reduced != ReducedSolve::kRetryAfterEviction)
          {
            break;
          }
          if (++retries > n + 1)
          {
            diagnostics_.status = QpStatus::kNumericalFailure;
            return;
          }
        }
        if (reduced == ReducedSolve::kNumericalFailure)
        {
          diagnostics_.status = QpStatus::kNumericalFailure;
          return;
        }

        // 2. 步长为零时检查乘子符号（释放约束 / 收敛判定）
        if (reduced == ReducedSolve::kZeroStep)
        {
          const double dual_scale = std::max(1.0, working_lambda_.head(n).cwiseAbs().maxCoeff());
          int worst = -1;
          double worst_value = -tolerances_.dual * dual_scale;
          for (std::size_t slot = 0; slot < working_.size(); ++slot)
          {
            if (working_lambda_(slot) < worst_value)
            {
              worst_value = working_lambda_(slot);
              worst = static_cast<int>(slot);
            }
          }
          if (worst < 0)
          {
            // 所有工作集乘子均非负，且步长为零：已收敛到全局最优解
            diagnostics_.status = QpStatus::kConverged;
            return;
          }
          // 存在负乘子：释放该约束以允许目标函数进一步下降。
          // Maritz 防循环规则：零步长下释放的退化约束暂时加入 banned_，防止同点反复进出陷入死循环。
          diagnostics_.constraint_replacements += 1;
          banned_[static_cast<std::size_t>(working_[worst])] = 1;
          working_.erase(working_.begin() + worst);
          continue;
        }

        // 3. 计算最大可行步长 alpha 与阻断约束 (blocking constraint)
        double alpha = 1.0;
        int blocking = -1;
        for (int row = 0; row < m; ++row)
        {
          if (std::find(working_.begin(), working_.end(), row) != working_.end())
          {
            continue;
          }
          if (banned_[static_cast<std::size_t>(row)])
          {
            continue;
          }
          const double directional = A.row(row).head(n).dot(step_.head(n));
          if (directional <= tolerances_.direction)
          {
            continue;
          }
          const double slack = b(row) - A.row(row).head(n).dot(x_.head(n));
          const double candidate = slack / directional;
          if (candidate < alpha)
          {
            alpha = candidate;
            blocking = row;
          }
        }

        // 更新迭代点
        x_.head(n) += alpha * step_.head(n);
        if (alpha > 0.0)
        {
          // 迭代点实际发生移动后，解除禁用约束（原释放约束在空间新位置可合法阻断）
          std::fill(banned_.begin(), banned_.end(), static_cast<char>(0));
        }
        if (!x_.head(n).allFinite())
        {
          diagnostics_.status = QpStatus::kNumericalFailure;
          return;
        }

        // 若步长受阻 (alpha < 1.0)，将阻断约束收入工作集
        if (blocking >= 0 && alpha < 1.0)
        {
          add_working_row(blocking, n);
        }
      }
      diagnostics_.status = QpStatus::kMaxIterations;
    }

    enum class ReducedSolve
    {
      kDescent,
      kZeroStep,
      kNumericalFailure,
      kRetryAfterEviction,
    };

    /// 解等式约束二次规划子问题：
    ///   minimize    0.5 * p^T H p + (H x + g)^T p
    ///   subject to  a_i^T p = 0  (i \in working_)
    /// 构造 KKT 系统：[H, A_w^T; A_w, 0] [p; mu] = [-grad; 0]
    /// 乘子符号约定：KKT 方程写作 +A' mu，因此需释放的行对应 mu < 0。
    ReducedSolve solve_reduced(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, int n, int m)
    {
      (void)b;
      (void)m;
      const int working_size = static_cast<int>(working_.size());
      const int size = n + working_size;
      if (size > kkt_.rows())
      {
        return ReducedSolve::kNumericalFailure;
      }

      if (working_size == 0)
      {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(h_sym_.topLeftCorner(n, n));
        if (!lu.isInvertible())
        {
          return ReducedSolve::kNumericalFailure;
        }
        step_.head(n) = lu.solve(-gradient_.head(n));
        return negligible_step(n) ? ReducedSolve::kZeroStep : ReducedSolve::kDescent;
      }

      kkt_.setZero();
      kkt_.topLeftCorner(n, n) = h_sym_.topLeftCorner(n, n);
      for (int slot = 0; slot < working_size; ++slot)
      {
        const Eigen::Index column = n + slot;
        kkt_.block(0, column, n, 1) = A.row(working_[slot]).head(n).transpose();
        kkt_.block(column, 0, 1, n) = A.row(working_[slot]).head(n);
      }
      rhs_.head(size).setZero();
      rhs_.head(n) = -gradient_.head(n);
      for (int slot = 0; slot < working_size; ++slot)
      {
        rhs_(n + slot) = 0.0; // 当前点已在工作集边界上 a_i^T x = b_i，因此 a_i^T p = 0
      }

      Eigen::FullPivLU<Eigen::MatrixXd> lu(kkt_.topLeftCorner(size, size));
      while (!lu.isInvertible() || lu.rank() < size)
      {
        // 退化顶点处理：工作集出现线性相关约束致 KKT 矩阵奇异。
        // 淘汰最后加入的约束行，禁用后重新求解 KKT 系统。
        if (working_.empty())
        {
          return ReducedSolve::kNumericalFailure;
        }
        const int dropped = working_.back();
        working_.pop_back();
        banned_[static_cast<std::size_t>(dropped)] = 1;
        diagnostics_.constraint_replacements += 1;
        return ReducedSolve::kRetryAfterEviction;
      }
      solved_.head(size) = lu.solve(rhs_.head(size));
      if (!solved_.head(size).allFinite())
      {
        return ReducedSolve::kNumericalFailure;
      }
      step_.head(n) = solved_.head(n);
      for (int slot = 0; slot < working_size; ++slot)
      {
        working_lambda_(slot) = solved_(n + slot);
      }
      return negligible_step(n) ? ReducedSolve::kZeroStep : ReducedSolve::kDescent;
    }

    void compute_residuals(const Eigen::MatrixXd &A, const Eigen::VectorXd &b,
                           const Eigen::VectorXd &g, int n, int m)
    {
      double primal = 0.0;
      double dual = 0.0;
      double complementarity = 0.0;
      for (int row = 0; row < m; ++row)
      {
        lambda_(row) = 0.0;
      }
      for (std::size_t slot = 0; slot < working_.size(); ++slot)
      {
        lambda_(working_[slot]) = working_lambda_(slot);
      }
      double ratio = 0.0;
      for (int row = 0; row < m; ++row)
      {
        const double violation = A.row(row).head(n).dot(x_.head(n)) - b(row);
        primal = std::max(primal, violation);
        ratio = std::max(ratio, violation / row_tolerance(row, n, b));
        dual = std::max(dual, -lambda_(row));
        complementarity = std::max(complementarity, std::abs(lambda_(row) * violation));
      }
      diagnostics_.primal_violation_ratio = ratio;
      residual_.head(n) = h_sym_.topLeftCorner(n, n) * x_.head(n) + g.head(n);
      stationarity_scale_ = std::max(1.0, g.head(n).cwiseAbs().maxCoeff());
      stationarity_scale_ = std::max(stationarity_scale_,
                                     (h_sym_.topLeftCorner(n, n) * x_.head(n)).cwiseAbs().maxCoeff());
      for (int row = 0; row < m; ++row)
      {
        if (lambda_(row) != 0.0)
        {
          residual_.head(n).noalias() += lambda_(row) * A.row(row).head(n).transpose();
        }
      }
      diagnostics_.primal_residual = primal;
      diagnostics_.dual_residual = dual;
      diagnostics_.complementarity = complementarity;
      diagnostics_.stationarity = residual_.head(n).cwiseAbs().maxCoeff();
      diagnostics_.objective = 0.5 * x_.head(n).dot(h_sym_.topLeftCorner(n, n) * x_.head(n)) +
                               g.head(n).dot(x_.head(n));
      diagnostics_.working_set_size = static_cast<int>(working_.size());
    }

    QpStatus classify(int m)
    {
      if (diagnostics_.primal_violation_ratio > 1.0)
      {
        return QpStatus::kNumericalFailure;
      }
      const double dual_scale = std::max(1.0, lambda_.head(m).cwiseAbs().maxCoeff());
      if (diagnostics_.dual_residual > tolerances_.dual * dual_scale)
      {
        return QpStatus::kNumericalFailure;
      }
      if (diagnostics_.stationarity > tolerances_.stationarity * stationarity_scale_)
      {
        return QpStatus::kNumericalFailure;
      }
      return QpStatus::kConverged;
    }

    static double elapsed_us(const std::chrono::steady_clock::time_point &start)
    {
      const auto delta = std::chrono::steady_clock::now() - start;
      return std::chrono::duration<double, std::micro>(delta).count();
    }

    int max_variables_;
    int max_constraints_;
    QpTolerances tolerances_;
    QpDiagnostics diagnostics_;
    double stationarity_scale_ = 1.0;

    Eigen::MatrixXd kkt_;
    Eigen::MatrixXd h_sym_;
    Eigen::VectorXd rhs_;
    Eigen::VectorXd solved_;
    Eigen::VectorXd x_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd step_;
    Eigen::VectorXd lambda_;
    Eigen::VectorXd working_lambda_;
    Eigen::VectorXd residual_;
    std::vector<int> working_;
    std::vector<char> banned_;
    std::vector<double> constraints_row_norm_;
  };

} // namespace bbot_balance_controller
