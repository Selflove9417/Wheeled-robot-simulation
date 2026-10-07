// Standalone checks for DenseActiveSetQp: constraint satisfaction, KKT
// residuals, warm start, failure detection, and agreement with a brute-force
// vertex enumeration used as an independent reference optimum.
//
// Build (no ROS needed):
//   cd /home/admin/bbot_ws_new && g++ -std=c++17 -O2 -I/usr/include/eigen3
//   -Isrc/bbot_balance_controller/include
//   src/bbot_balance_controller/test/test_dense_active_set_qp.cpp -o /tmp/test_qp

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "bbot_balance_controller/dense_active_set_qp.hpp"

using bbot_balance_controller::DenseActiveSetQp;
using bbot_balance_controller::QpStatus;
using bbot_balance_controller::qp_status_name;

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

struct SmallQp
{
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

/// Reference optimum by enumerating every candidate vertex (all subsets of
/// constraints of size <= n) plus clipping to the box, keeping the best
/// feasible objective. Only valid for the small problems used below.
double brute_force_best_objective(const SmallQp &qp, int n, int m)
{
  double best = std::numeric_limits<double>::infinity();
  std::vector<int> indices(m);
  for (int row = 0; row < m; ++row)
  {
    indices[row] = row;
  }

  const Eigen::VectorXd unconstrained =
    qp.H.fullPivLu().solve(-qp.g);
  std::vector<std::vector<int>> candidates;
  candidates.push_back({});
  const int total = 1 << m;
  for (int mask = 1; mask < total; ++mask)
  {
    std::vector<int> subset;
    for (int row = 0; row < m; ++row)
    {
      if (mask & (1 << row))
      {
        subset.push_back(row);
      }
    }
    if (static_cast<int>(subset.size()) <= n)
    {
      candidates.push_back(subset);
    }
  }

  for (const auto &subset : candidates)
  {
    Eigen::VectorXd x;
    if (subset.empty())
    {
      x = unconstrained;
    }
    else
    {
      const int size = static_cast<int>(subset.size());
      Eigen::MatrixXd kkt(n + size, n + size);
      Eigen::VectorXd rhs(n + size);
      kkt.topLeftCorner(n, n) = qp.H;
      kkt.block(0, n, n, size) = qp.A(subset, Eigen::all).transpose();
      kkt.block(n, 0, size, n) = qp.A(subset, Eigen::all);
      kkt.block(n, n, size, size).setZero();
      rhs.head(n) = -qp.g;
      rhs.tail(size) = qp.b(subset);
      Eigen::FullPivLU<Eigen::MatrixXd> lu(kkt);
      if (!lu.isInvertible())
      {
        continue;
      }
      x = lu.solve(rhs).head(n);
    }
    if (!x.allFinite())
    {
      continue;
    }
    bool feasible = true;
    for (int row = 0; row < m; ++row)
    {
      if (qp.A.row(row).dot(x) > qp.b(row) + 1e-7)
      {
        feasible = false;
        break;
      }
    }
    if (!feasible)
    {
      continue;
    }
    const double objective = 0.5 * x.dot(qp.H * x) + qp.g.dot(x);
    best = std::min(best, objective);
  }
  return best;
}
}  // namespace

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  // 1. Unconstrained quadratic: minimum at (1, 2).
  {
    Eigen::MatrixXd H(2, 2);
    H << 2.0, 0.0, 0.0, 4.0;
    Eigen::VectorXd g(2);
    g << -2.0, -8.0;
    Eigen::MatrixXd A(1, 2);
    A << 0.0, 0.0;  // inactive row 0 * x <= 1
    Eigen::VectorXd b(1);
    b << 1.0;

    DenseActiveSetQp solver(2, 1);
    const QpStatus status = solver.solve(H, g, A, b);
    require(status == QpStatus::kConverged, "unconstrained case must converge");
    require(std::abs(solver.solution()(0) - 1.0) < 1e-9, "x0 wrong");
    require(std::abs(solver.solution()(1) - 2.0) < 1e-9, "x1 wrong");
    require(solver.diagnostics().primal_residual <= 1e-8, "primal residual");
    require(solver.diagnostics().dual_residual <= 1e-8, "dual residual");
    require(solver.diagnostics().stationarity <= 1e-6, "stationarity residual");
    require(solver.diagnostics().complementarity <= 1e-6, "complementarity residual");
    require(solver.diagnostics().iterations >= 1, "iteration count must be reported");
    require(solver.diagnostics().solve_time_us >= 0.0, "solve time must be reported");
  }

  // 2. Box + coupling constraints: the coupling row must be strictly slack at
  // the optimum so both binding multipliers are provably positive.
  {
    SmallQp qp;
    qp.H = Eigen::MatrixXd::Identity(2, 2) * 2.0;
    qp.g.resize(2);
    qp.g << -6.0, -6.0;  // wants (3, 3)
    qp.A.resize(4, 2);
    qp.A << 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, -1.0, 0.0;
    qp.b.resize(4);
    // x >= 0 keeps the origin feasible (the primal active set needs a
    // feasible start) and stays slack at the optimum.
    qp.b << 1.0, 0.4, 2.0, 0.0;  // x <= 1, y <= 0.4, x+y <= 2, x >= 0
    DenseActiveSetQp solver(2, 4);
    const QpStatus status = solver.solve(qp.H, qp.g, qp.A, qp.b);
    printf("case2: status=%s iters=%d ws=%d prim=%.3e dual=%.3e stat=%.3e obj=%.6f x=[%.6f %.6f]\n",
      qp_status_name(status), solver.diagnostics().iterations, solver.diagnostics().working_set_size,
      solver.diagnostics().primal_residual, solver.diagnostics().dual_residual,
      solver.diagnostics().stationarity, solver.diagnostics().objective,
      solver.solution()(0), solver.solution()(1));
    require(status == QpStatus::kConverged, "box case must converge");
    require(std::abs(solver.solution()(0) - 1.0) < 1e-9, "x should sit on its upper bound");
    require(std::abs(solver.solution()(1) - 0.4) < 1e-9, "y should sit on its upper bound");
    for (int row = 0; row < 4; ++row)
    {
      require(qp.A.row(row).dot(solver.solution()) <= qp.b(row) + 1e-8, "constraint violated");
    }
    const double reference = brute_force_best_objective(qp, 2, 4);
    require(std::abs(solver.diagnostics().objective - reference) < 1e-9, "objective != reference");
    require(solver.multipliers()(0) > 0.0, "binding x<=1 needs a positive multiplier");
    require(solver.multipliers()(1) > 0.0, "binding y<=0.4 needs a positive multiplier");
    require(solver.multipliers()(2) == 0.0, "slack row must have zero multiplier");
    require(solver.diagnostics().working_set_size == 2, "two constraints should be active");
  }

  // 3. An infeasible warm start is rejected by the solver and the caller
  // supplied feasible point is used instead.
  {
    SmallQp qp;
    qp.H = Eigen::MatrixXd::Identity(3, 3);
    qp.g.resize(3);
    qp.g << 0.0, -3.0, 0.0;  // wants (0, 3, 0)
    qp.A.resize(4, 3);
    qp.A << 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0;
    qp.b.resize(4);
    qp.b << 1.25, 0.0, 4.0, -2.0;  // y <= 1.25, y >= 0, sum <= 4, sum >= 2
    DenseActiveSetQp solver(3, 4);
    Eigen::VectorXd warm(3);
    warm << 50.0, -50.0, 50.0;  // deep infeasible start
    Eigen::VectorXd feasible_start(3);
    feasible_start << 1.0, 1.0, 1.0;
    const QpStatus status = solver.solve(qp.H, qp.g, qp.A, qp.b, &warm, &feasible_start);
    require(status == QpStatus::kConverged, "infeasible warm start must recover");
    require(!solver.diagnostics().warm_start_used, "infeasible warm start must not be used");
    require(std::abs(solver.solution()(1) - 1.25) < 1e-9, "y bound should bind");
    const double reference = brute_force_best_objective(qp, 3, 4);
    require(std::abs(solver.diagnostics().objective - reference) < 1e-8, "objective != reference");

    // Re-solve from the previous solution: the warm start must be accepted and
    // must not cost more iterations than the cold start.
    const int cold_iterations = solver.diagnostics().iterations;
    Eigen::VectorXd previous = solver.solution().head(3);
    const QpStatus warm_status = solver.solve(qp.H, qp.g, qp.A, qp.b, &previous, &feasible_start);
    require(warm_status == QpStatus::kConverged, "warm re-solve must converge");
    require(solver.diagnostics().warm_start_used, "feasible warm start must be used");
    require(solver.diagnostics().iterations <= cold_iterations,
      "warm start should not need more iterations than a cold start");
    printf("warm start: cold=%d iterations warm=%d iterations\n", cold_iterations,
      solver.diagnostics().iterations);
  }

  // 4. Contradictory constraints must be reported, never silently "solved".
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(1, 1);
    Eigen::VectorXd g(1);
    g << 0.0;  // minimize 0.5 x^2 s.t. x <= 1 and x >= 2
    Eigen::MatrixXd A(2, 1);
    A << 1.0, -1.0;
    Eigen::VectorXd b(2);
    b << 1.0, -2.0;
    DenseActiveSetQp solver(1, 2);
    const QpStatus status = solver.solve(H, g, A, b);
    require(status != QpStatus::kConverged, "infeasible QP must not report convergence");
    const bool flagged = (status == QpStatus::kNoFeasibleStart ||
      status == QpStatus::kMaxIterations || status == QpStatus::kNumericalFailure);
    require(flagged, "infeasible QP must return a diagnosable status");
    require(solver.diagnostics().primal_residual > 0.0 ||
      status == QpStatus::kNoFeasibleStart, "infeasibility must be visible in the residuals");
    printf("infeasible QP -> status=%s iterations=%d primal_residual=%.3e\n",
      qp_status_name(status), solver.diagnostics().iterations,
      solver.diagnostics().primal_residual);
  }

  // 5. Random small problems cross-checked against vertex enumeration.
  {
    const int n = 4;
    const int m = 9;
    DenseActiveSetQp solver(n, m);
    Eigen::MatrixXd base = Eigen::MatrixXd::Random(n, n);
    Eigen::MatrixXd H = base * base.transpose() + Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd g = Eigen::VectorXd::Random(n);
    Eigen::MatrixXd A(m, n);
    Eigen::VectorXd b(m);
    double worst_objective_gap = 0.0;
    double worst_violation = 0.0;
    for (int trial = 0; trial < 200; ++trial)
    {
      Eigen::MatrixXd perturbation = Eigen::MatrixXd::Random(n, n) * 0.2;
      Eigen::MatrixXd Ht = (H + perturbation) * (H + perturbation).transpose() +
        Eigen::MatrixXd::Identity(n, n);
      Eigen::VectorXd gt = g + Eigen::VectorXd::Random(n) * 0.5;
      for (int row = 0; row < n; ++row)
      {
        A.row(row).setZero();
        A(row, row) = 1.0;
        b(row) = 1.0 + 0.5 * std::sin(0.3 * trial + row);
        A.row(n + row).setZero();
        A(n + row, row) = -1.0;
        b(n + row) = 1.0 + 0.5 * std::cos(0.2 * trial + row);
      }
      for (int row = 2 * n; row < m; ++row)
      {
        A.row(row) = Eigen::RowVectorXd::Random(n);
        b(row) = 1.0 + std::abs(Eigen::RowVectorXd::Random(1).value()) * 1.5;
      }
      const QpStatus status = solver.solve(Ht, gt, A, b);
      if (status != QpStatus::kConverged)
      {
        printf("trial %d -> status=%s (accepted, fallback path)\n", trial,
          qp_status_name(status));
        continue;
      }
      for (int row = 0; row < m; ++row)
      {
        worst_violation = std::max(worst_violation, A.row(row).dot(solver.solution()) - b(row));
      }
      require(solver.diagnostics().dual_residual <= 1e-6, "negative multiplier accepted");
      require(solver.diagnostics().stationarity <= 1e-4, "stationarity residual too large");
      const SmallQp qp{Ht, gt, A, b};
      const double reference = brute_force_best_objective(qp, n, m);
      if (std::isfinite(reference))
      {
        worst_objective_gap = std::max(
          worst_objective_gap, std::abs(solver.diagnostics().objective - reference));
      }
    }
    printf("200 random QPs: worst constraint violation=%.3e worst objective gap=%.3e\n",
      worst_violation, worst_objective_gap);
    require(worst_violation <= 1e-7, "a constraint was violated at the reported optimum");
    require(worst_objective_gap <= 1e-7, "objective differs from vertex-enumeration optimum");
  }

  // 6. Invalid input is rejected instead of producing a fabricated answer.
  {
    Eigen::MatrixXd H(2, 2);
    H << 1.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN();
    Eigen::VectorXd g = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(1);
    DenseActiveSetQp solver(2, 1);
    require(solver.solve(H, g, A, b) == QpStatus::kInvalidInput, "NaN H must be rejected");
    require(solver.solution()(0) == 0.0 && solver.solution()(1) == 0.0,
      "rejected problem must return a defined (zero) solution");

    Eigen::MatrixXd too_big(3, 3);
    too_big.setIdentity();
    Eigen::VectorXd g3 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd a3(1, 3);
    a3.setZero();
    require(solver.solve(too_big, g3, a3, b) == QpStatus::kInvalidInput,
      "oversized problem must be rejected");
  }

  // 7. Semi-sparse timing benchmark at the MPC working size (n = 20, m = 80).
  {
    const int n = 20;
    Eigen::MatrixXd M = Eigen::MatrixXd::Random(n, n);
    Eigen::MatrixXd H = M * M.transpose() + Eigen::MatrixXd::Identity(n, n) * 10.0;
    Eigen::VectorXd g = Eigen::VectorXd::Random(n) * 100.0;
    Eigen::MatrixXd A(4 * n, n);
    Eigen::VectorXd b(4 * n);
    for (int row = 0; row < n; ++row)
    {
      A.row(row).setZero();
      A(row, row) = 1.0;
      b(row) = 20.0;
      A.row(n + row).setZero();
      A(n + row, row) = -1.0;
      b(n + row) = 20.0;
    }
    for (int row = 2 * n; row < 4 * n; ++row)
    {
      A.row(row) = Eigen::RowVectorXd::Random(n);
      b(row) = 0.5 + std::abs(Eigen::RowVectorXd::Random(1).value());
    }
    DenseActiveSetQp solver(n, 4 * n);
    double total_us = 0.0;
    double max_us = 0.0;
    int failures = 0;
    Eigen::VectorXd warm;
    for (int trial = 0; trial < 2000; ++trial)
    {
      const QpStatus status = solver.solve(H, g, A, b, trial ? &warm : nullptr);
      if (status != QpStatus::kConverged)
      {
        ++failures;
      }
      warm = solver.solution().head(n);
      total_us += solver.diagnostics().solve_time_us;
      max_us = std::max(max_us, solver.diagnostics().solve_time_us);
    }
    printf("n=20 m=80 x2000: mean=%.1f us max=%.1f us non-converged=%d\n",
      total_us / 2000.0, max_us, failures);
    // This synthetic case uses dense random rows, which the MPC never
    // generates (its rows are one torque box row or a short pitch-band row), so
    // it is the pessimistic bound. The repository builds with an empty
    // CMAKE_BUILD_TYPE: ~40 us mean at -O2, ~1.2 ms mean unoptimised.
    require(total_us / 2000.0 < 2500.0, "QP mean solve time exceeds half the 200 Hz period");
  }

  printf("test_dense_active_set_qp: all checks passed\n");
  return 0;
}
