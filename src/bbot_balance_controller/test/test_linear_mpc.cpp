// 定高度线性 MPC 核心的行为检查：无约束生效时必须复现 LQR 控制律，严格执行
// 器箱约束，保持俯仰误差带（或如实上报所需松弛），按定义好的链条降级，并且
// 足够快以满足部署的 200 Hz 周期。
//
// 参考数值来自 lqr_plant_model.hpp 在 H = 0.40 m 对象上的结果，即已部署 LQR
// 增益表所用的同一模型。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>

#include "bbot_balance_controller/linear_mpc.hpp"
#include "bbot_balance_controller/lqr_plant_model.hpp"
#include "bbot_kinematics/robot_params.hpp"

using bbot_balance_controller::LinearMpc;
using bbot_balance_controller::LinearMpcConfig;
using bbot_balance_controller::LinearMpcDiagnostics;
using bbot_balance_controller::MpcFallbackStage;
using bbot_balance_controller::QpStatus;

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

struct Plant
{
  Eigen::Matrix4d Ad;
  Eigen::Vector4d Bd;
  Eigen::Matrix4d Q;
  Eigen::Matrix4d P;
  Eigen::RowVector4d K;
};

// 部署增益表的 5 个节点（adaptive_lqr_balance_controller.cpp:300-312）。控制器
// 只用它做两件事：theta_eq(H) 的悬挂体质心，以及 Riccati 迭代的 stabilizing
// 种子。这里复制同一张表和同一个插值规则，用于检查整段 0.30~0.50 m 的可解性。
struct TableRow
{
  double height;
  double k_x;
  double k_x_dot;
  double k_theta;
  double k_theta_dot;
  double y_com;
  double z_com;
};

constexpr std::array<TableRow, 5> kTable{{
  {0.3000, -5.622712, -42.666506, -156.508986, -35.846746, -0.0276264, 0.3592154},
  {0.3500, -5.791278, -43.976682, -169.544125, -39.563240, -0.0258066, 0.4016316},
  {0.4000, -5.931603, -45.087129, -181.972969, -43.390225, -0.0234056, 0.4443432},
  {0.4500, -6.052411, -46.061241, -193.948669, -47.349665, -0.0203401, 0.4872441},
  {0.5000, -6.157159, -46.922551, -205.518217, -51.430573, -0.0164269, 0.5302655}}};

TableRow interp_row(double height)
{
  if (height <= kTable.front().height)
  {
    return kTable.front();
  }
  if (height >= kTable.back().height)
  {
    return kTable.back();
  }
  for (std::size_t index = 0; index + 1 < kTable.size(); ++index)
  {
    const TableRow &low = kTable[index];
    const TableRow &high = kTable[index + 1];
    if (height >= low.height && height <= high.height)
    {
      const double ratio = (height - low.height) / (high.height - low.height);
      const auto lerp = [ratio](double a, double b) { return a + (b - a) * ratio; };
      return TableRow{height, lerp(low.k_x, high.k_x), lerp(low.k_x_dot, high.k_x_dot),
        lerp(low.k_theta, high.k_theta), lerp(low.k_theta_dot, high.k_theta_dot),
        lerp(low.y_com, high.y_com), lerp(low.z_com, high.z_com)};
    }
  }
  return kTable[2];
}

Eigen::RowVector4d table_seed(double height)
{
  const TableRow row = interp_row(height);
  Eigen::RowVector4d seed;
  seed << row.k_x, row.k_x_dot, row.k_theta, row.k_theta_dot;
  return seed;
}

/// adaptive 的规则：先按高度插值 y_com / z_com，再取 -atan2，而不是先算各节点
/// 角度再插值角度。
double table_theta_eq(double height)
{
  const TableRow row = interp_row(height);
  return -std::atan2(row.y_com, row.z_com);
}

Plant make_plant(double height, const bbot_kinematics::RobotParams &parameters)
{
  Plant plant;
  Eigen::Matrix4d A;
  Eigen::Vector4d B;
  bbot_balance_controller::lqr_plant::continuous_matrices(height, parameters, A, B);
  bbot_balance_controller::lqr_plant::zoh_discretize(0.005, A, B, plant.Ad, plant.Bd);
  plant.Q = Eigen::Matrix4d::Zero();
  plant.Q(0, 0) = 100.0;
  plant.Q(1, 1) = 5000.0;
  plant.Q(2, 2) = 3000.0;
  plant.Q(3, 3) = 1200.0;
  const Eigen::RowVector4d seed = table_seed(height);
  int iterations = 0;
  double residual = 0.0;
  const bool ok = bbot_balance_controller::lqr_plant::solve_dare(
    plant.Ad, plant.Bd, plant.Q, 1.0, seed, plant.P, plant.K, 50, iterations, residual);
  require(ok, "Riccati solution failed for the test plant");
  return plant;
}

Eigen::Vector4d step(const Plant &plant, const Eigen::Vector4d &state, double u)
{
  return plant.Ad * state + plant.Bd * u;
}

/// 真实模型上预测步 i = 1..N 内的最大绝对俯仰误差。步 0 是测量状态，任何
/// 输入都无法改变，故排除；MPC 带宽约束只针对未来各步。
double rollout_pitch_max(const Plant &plant, const Eigen::Vector4d &state0,
  const Eigen::VectorXd &plan)
{
  Eigen::Vector4d state = state0;
  double maximum = 0.0;
  for (int i = 0; i < plan.size(); ++i)
  {
    state = step(plant, state, plan(i));
    maximum = std::max(maximum, std::abs(state(2)));
  }
  return maximum;
}
}  // namespace

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  bbot_kinematics::RobotParams parameters;
  const Plant plant = make_plant(0.40, parameters);

  LinearMpcConfig base;
  base.horizon = 20;
  base.total_torque_limit = 20.0;   // 2 x wheel_torque_max，与 GS-LQR 相同
  base.wheel_torque_limit = 10.0;
  base.theta_error_limit = 0.20;
  base.use_theta_band = true;

  // 1. 收敛时域必须复现 DARE 控制律，等效第一步增益必须匹配 -K。
  {
    LinearMpc mpc;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, base),
      "configure failed");
    // 残差间隙为 (rho_closed_loop)^(2N)：rho = 0.9993 时 N = 20 为 2.8%，
    // N = 40 为 1.1%，因此时域必须足够长，有限时域控制律才能贴合无穷时域 LQR。
    double worst_gain = 0.0;
    for (int index = 0; index < 4; ++index)
    {
      worst_gain = std::max(worst_gain,
        std::abs(mpc.first_move_gain()(index) + plant.K(index)) / std::abs(plant.K(index)));
    }
    std::printf("N=20 first-move gain vs DARE: worst relative difference = %.4f%%\n",
      worst_gain * 100.0);
    require(worst_gain < 0.03, "MPC first move does not reproduce the LQR gain");

    LinearMpcConfig long_config = base;
    long_config.horizon = 40;
    LinearMpc long_mpc;
    require(long_mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, long_config),
      "configure failed for N=40");
    double long_gap = 0.0;
    for (int index = 0; index < 4; ++index)
    {
      long_gap = std::max(long_gap,
        std::abs(long_mpc.first_move_gain()(index) + plant.K(index)) / std::abs(plant.K(index)));
    }
    std::printf("N=40 first-move gain vs DARE: worst relative difference = %.4f%%\n",
      long_gap * 100.0);
    require(long_gap < worst_gain, "a longer horizon must approach the DARE gain more closely");
    require(long_gap < 0.015, "N=40 should sit within 1.5% of the infinite-horizon gain");

    const Eigen::Vector4d small(0.02, 0.05, 0.01, 0.05);
    LinearMpcDiagnostics diagnostics;
    const double u = mpc.solve(small, diagnostics);
    const double lqr_u = -plant.K.dot(small);
    std::printf("small-signal MPC u=%.6f LQR u=%.6f (stage=%s iters=%d %.1f us)\n", u, lqr_u,
      mpc_fallback_name(diagnostics.fallback_stage), diagnostics.qp_iterations,
      diagnostics.qp_solve_time_us);
    require(diagnostics.fallback_stage == MpcFallbackStage::kSolved, "small signal must solve fully");
    require(std::abs(u - lqr_u) < 0.02 * std::abs(lqr_u) + 1.0e-6,
      "unsaturated MPC input must match the LQR input");
    require(diagnostics.theta_slack_upper == 0.0 && diagnostics.theta_slack_lower == 0.0,
      "slack must stay exactly zero when the band is reachable");
    require(!diagnostics.total_torque_saturated, "small signal must not saturate");
    require(diagnostics.qp_status == QpStatus::kConverged, "status");
    require(diagnostics.qp_primal_residual <= 1.0e-8, "primal residual");
    require(diagnostics.qp_dual_residual <= 1.0e-6, "dual residual");
  }

  // 2. 总转矩与单轮转矩限制被精确执行。
  {
    LinearMpc mpc;
    LinearMpcConfig config = base;
    config.use_theta_band = false;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, config),
      "configure failed");
    require(std::abs(mpc.torque_limit() - 20.0) < 1.0e-12, "bound is min(20, 2*10)");

    const Eigen::Vector4d aggressive(1.0, 0.8, 0.15, 1.0);
    LinearMpcDiagnostics diagnostics;
    const double u = mpc.solve(aggressive, diagnostics);
    std::printf("saturated case: u=%.6f raw=%.6f lqr=%.6f total_sat=%d wheel_sat=%d ws=%d\n", u,
      diagnostics.u_qp_raw, diagnostics.lqr_u,
      static_cast<int>(diagnostics.total_torque_saturated),
      static_cast<int>(diagnostics.wheel_torque_saturated), diagnostics.working_set_size);
    require(std::abs(std::abs(u) - 20.0) < 1.0e-9, "input must sit exactly on the bound");
    require(std::abs(u) < std::abs(diagnostics.lqr_u), "the bound must reduce the input");
    require(std::abs(diagnostics.tau_each) <= 10.0 + 1.0e-12, "wheel effort beyond its limit");
    require(diagnostics.total_torque_saturated, "saturation flag");
    for (int index = 0; index < mpc.plan().size(); ++index)
    {
      require(std::abs(mpc.plan()(index)) <= 20.0 + 1.0e-9, "a plan element broke the box");
    }

    // 更紧的单轮限制必须收紧同一输入。
    LinearMpc restricted;
    LinearMpcConfig tight = config;
    tight.wheel_torque_limit = 5.0;   // -> |u| <= 10
    require(restricted.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, tight),
      "configure failed");
    LinearMpcDiagnostics tight_report;
    const double tight_u = restricted.solve(aggressive, tight_report);
    require(std::abs(std::abs(tight_u) - 10.0) < 1.0e-9, "per-wheel limit must bound the total input");
    require(std::abs(tight_report.tau_each) <= 5.0 + 1.0e-12, "per-wheel limit violated");
  }

  // 3. 俯仰误差带：带宽较宽时不起作用；起作用且执行器有裕量时精确满足；
  //    超出转矩权限时通过松弛如实上报。
  {
    const Eigen::Vector4d state(0.05, 0.10, 0.010, 0.10);

    LinearMpc none;
    LinearMpcConfig without_band = base;
    without_band.use_theta_band = false;
    require(none.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, without_band),
      "configure failed");
    LinearMpcDiagnostics free_report;
    const double u_free = none.solve(state, free_report);
    const double free_peak = rollout_pitch_max(plant, state, none.plan());
    std::printf("no band: u=%.6f rollout pitch peak=%.6f\n", u_free, free_peak);
    require(free_report.fallback_stage == MpcFallbackStage::kSolved, "box-only case must solve");

    // (a) 带宽为无带宽约束峰值的 2 倍时，答案必须完全不变，且不需要松弛。
    LinearMpc wide;
    LinearMpcConfig wide_config = base;
    wide_config.theta_error_limit = 2.0 * free_peak;
    require(wide.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, wide_config),
      "configure failed");
    LinearMpcDiagnostics wide_report;
    const double u_wide = wide.solve(state, wide_report);
    const double wide_peak = rollout_pitch_max(plant, state, wide.plan());
    std::printf("band=%.4f (wide): u=%.6f peak=%.6f slack=%.3e/%.3e\n",
      wide_config.theta_error_limit, u_wide, wide_peak, wide_report.theta_slack_upper,
      wide_report.theta_slack_lower);
    require(std::abs(u_wide - u_free) < 1.0e-6, "a slack band must not alter the unconstrained input");
    require(wide_report.theta_slack_upper == 0.0 && wide_report.theta_slack_lower == 0.0,
      "a reachable band must not need slack");
    require(wide_peak <= wide_config.theta_error_limit + 1.0e-9, "wide band must hold");

    // (b) 带宽取峰值的 60% 时必须起作用：要么恰好满足，要么上报的松弛等于
    //     残余违反量，不能出现其他情况。
    LinearMpc tight;
    LinearMpcConfig tight_config = base;
    tight_config.theta_error_limit = 0.6 * free_peak;
    require(tight.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, tight_config),
      "configure failed");
    LinearMpcDiagnostics tight_report;
    const double u_tight = tight.solve(state, tight_report);
    const double tight_peak = rollout_pitch_max(plant, state, tight.plan());
    const double slack = std::max(tight_report.theta_slack_upper, tight_report.theta_slack_lower);
    std::printf("band=%.4f (binding): u=%.6f peak=%.6f slack=%.6f stage=%s iters=%d\n",
      tight_config.theta_error_limit, u_tight, tight_peak, slack,
      mpc_fallback_name(tight_report.fallback_stage), tight_report.qp_iterations);
    require(tight_report.fallback_stage == MpcFallbackStage::kSolved,
      "the band case must be solved by the QP, not by the fallback");
    require(std::abs(u_tight - u_free) > 1.0e-6, "a binding band must change the input");
    require(tight_peak <= tight_config.theta_error_limit + slack + 1.0e-6,
      "residual pitch violation must be covered by the reported slack");
    if (slack <= 1.0e-9)
    {
      require(tight_peak <= tight_config.theta_error_limit + 1.0e-6,
        "zero slack but the band was exceeded");
    }
    for (int index = 0; index < tight.plan().size(); ++index)
    {
      require(std::abs(tight.plan()(index)) <= 20.0 + 1.0e-9, "box violated in the band case");
    }

    // (c) 已跌出带宽：求解器必须返回有定义、有界的输入，并上报无法消除的
    //     违反量。
    const Eigen::Vector4d fallen(0.0, 0.0, 0.19, 0.6);
    LinearMpcDiagnostics fallen_report;
    const double fallen_u = tight.solve(fallen, fallen_report);
    const double fallen_slack = fallen_report.theta_slack_upper + fallen_report.theta_slack_lower;
    std::printf("unreachable band: stage=%s slack=%.4f rad predicted_max=%.4f first_attempt=%s "
      "iters=%d prim=%.3e dual=%.3e stat=%.3e\n", mpc_fallback_name(fallen_report.fallback_stage),
      fallen_slack, fallen_report.theta_predicted_max,
      bbot_balance_controller::qp_status_name(fallen_report.initial_qp_status),
      fallen_report.initial_qp_iterations, fallen_report.initial_primal_residual,
      fallen_report.initial_dual_residual, fallen_report.initial_stationarity);
    require(fallen_report.fallback_stage == MpcFallbackStage::kSolved,
      "an unreachable band must still solve (soft constraint)");
    require(fallen_slack > 0.0, "the solver must report the violation it could not remove");
    require(std::abs(fallen_u) <= 20.0 + 1.0e-9, "box must still hold with slack");
  }

  // 4. 对任何输入都有定义、有界的输出，包括非有限状态。
  {
    LinearMpc mpc;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, base),
      "configure failed");
    LinearMpcDiagnostics diagnostics;
    const Eigen::Vector4d nan_state(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
    require(mpc.solve(nan_state, diagnostics) == 0.0, "NaN state must give zero torque");
    require(diagnostics.fallback_stage == MpcFallbackStage::kZeroTorque, "zero-torque stage");

    LinearMpc broken;
    LinearMpcConfig unconfigured;
    unconfigured.horizon = 0;
    require(!broken.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, unconfigured),
      "horizon 0 must be rejected");
    LinearMpcDiagnostics broken_report;
    require(broken.solve(Eigen::Vector4d::Zero(), broken_report) == 0.0,
      "unconfigured MPC must output zero");
    require(broken_report.fallback_stage == MpcFallbackStage::kZeroTorque, "zero-torque stage");
  }

  // 5. 降级链条：迭代上限设为 1 时，两级 QP 在饱和问题上都失败，必须由
  //    LQR 步接管，且仍做限幅。
  {
    LinearMpc mpc;
    LinearMpcConfig capped = base;
    capped.max_qp_iterations = 1;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, capped),
      "configure failed");
    const Eigen::Vector4d aggressive(1.0, 0.8, 0.15, 1.0);
    LinearMpcDiagnostics diagnostics;
    const double u = mpc.solve(aggressive, diagnostics);
    std::printf("fallback chain: stage=%s status=%s iters=%d u=%.6f\n",
      mpc_fallback_name(diagnostics.fallback_stage),
      bbot_balance_controller::qp_status_name(diagnostics.qp_status), diagnostics.qp_iterations, u);
    require(diagnostics.fallback_stage != MpcFallbackStage::kSolved,
      "a one-iteration cap cannot solve a binding problem");
    require(std::abs(u) <= 20.0 + 1.0e-9, "fallback output must respect the bound");
    require(std::isfinite(u), "fallback output must be finite");
  }

  // 6. 模型上的闭环：约束记账、快模态收敛，以及部署 5 ms 周期的求解器耗时。
  //
  //    该设计慢位置模态的实测谱半径为每拍 0.99930（时间常数约 7 s），这是
  //    部署增益表的属性而非 MPC 的属性，因此滚动验证只断言单调衰减，
  //    而不断言固定收敛时间。
  {
    LinearMpc mpc;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, base),
      "configure failed");
    Eigen::Vector4d state(0.50, 0.0, 0.10, 0.0);
    double total_us = 0.0;
    double max_us = 0.0;
    int non_solved = 0;
    double worst_pitch = 0.0;
    double worst_torque = 0.0;
    double previous_position = std::abs(state(0));
    double position_growth = 0.0;
    const int ticks = 2000;
    for (int tick = 0; tick < ticks; ++tick)
    {
      LinearMpcDiagnostics diagnostics;
      const double u = mpc.solve(state, diagnostics);
      require(std::isfinite(u), "rollout produced a non-finite input");
      total_us += diagnostics.qp_solve_time_us;
      max_us = std::max(max_us, diagnostics.qp_solve_time_us);
      if (diagnostics.fallback_stage != MpcFallbackStage::kSolved)
      {
        ++non_solved;
      }
      state = step(plant, state, u);
      require(state.allFinite(), "rollout state went non-finite");
      worst_pitch = std::max(worst_pitch, std::abs(state(2)));
      worst_torque = std::max(worst_torque, std::abs(u));
      if (tick > 100)
      {
        position_growth = std::max(0.0, std::abs(state(0)) - previous_position);
        if (position_growth > 1.0e-9 && tick > 400)
        {
          std::printf("position error grew by %.3e m at tick %d\n", position_growth, tick);
        }
      }
      previous_position = std::abs(state(0));
    }
    std::printf("%d-tick rollout: e=[%.5f %.6f %.6f %.6f] pitch_max=%.4f (limit %.2f) "
      "torque_max=%.3f mean=%.1f us max=%.1f us non_solved=%d\n", ticks, state(0), state(1),
      state(2), state(3), worst_pitch, base.theta_error_limit, worst_torque,
      total_us / ticks, max_us, non_solved);
    require(std::abs(state(2)) < 1.0e-3, "the pitch channel must settle");
    require(std::abs(state(3)) < 1.0e-3, "the pitch-rate channel must settle");
    require(std::abs(state(1)) < 0.05, "the velocity error must decay with the slow mode");
    require(std::abs(state(0)) < 0.25, "the slow position mode must keep decaying");
    require(worst_pitch <= base.theta_error_limit + 1.0e-6, "band must hold on the true model");
    require(worst_torque <= 20.0 + 1.0e-9, "box must hold during the rollout");
    // 阈值取 5 ms 周期的一半，且不是 -O2 下的数值：本仓库以空
    // CMAKE_BUILD_TYPE 构建，测试必须在未优化时通过。本机实测：
    // -O2 时约 20 us，未优化时约 1.4 ms。
    require(total_us / ticks < 2500.0, "mean solve time must stay below half the control period");
    require(max_us < 5000.0, "worst solve time must stay inside the control period");
    require(non_solved == 0, "every tick of the rollout should be solved by the QP");
  }

  // 7. 俯仰扰动被抓回，输入始终不超过配置的权限。
  {
    LinearMpc mpc;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, base),
      "configure failed");
    Eigen::Vector4d state = Eigen::Vector4d::Zero();
    state(2) = 0.12;   // 前倾 6.9 度；平衡角已被折算出去
    int ticks_pitch_settled = 0;
    double peak_torque = 0.0;
    double peak_position = 0.0;
    double peak_pitch = 0.0;
    for (int tick = 0; tick < 4000; ++tick)
    {
      LinearMpcDiagnostics diagnostics;
      const double u = mpc.solve(state, diagnostics);
      peak_torque = std::max(peak_torque, std::abs(u));
      state = step(plant, state, u);
      peak_position = std::max(peak_position, std::abs(state(0)));
      peak_pitch = std::max(peak_pitch, std::abs(state(2)));
      if (ticks_pitch_settled == 0 && std::abs(state(2)) < 1.0e-4 && std::abs(state(3)) < 1.0e-4)
      {
        ticks_pitch_settled = tick + 1;
      }
    }
    std::printf("pitch kick 0.12 rad: pitch|below 1e-4 at tick %d (=%.2f s, set by the 7 s slow "
      "mode), peak |u|=%.2f Nm, peak pitch=%.4f, position excursion=%.3f m, final e=[%.5f %.6f "
      "%.6f]\n", ticks_pitch_settled, ticks_pitch_settled * 0.005, peak_torque, peak_pitch,
      peak_position, state(0), state(1), state(2));
    require(ticks_pitch_settled > 0, "the MPC did not catch the pitch disturbance");
    require(peak_pitch < 0.13, "the pitch must not overshoot the applied disturbance");
    require(peak_torque <= 20.0 + 1.0e-9, "disturbance response must respect the bound");
    // 采用衰减比判据而非绝对下限：设计的慢模态（每拍谱半径 0.99930）约需
    // 20 s 才衰减到舍入误差量级，固定的小阈值只会考验时域长度。
    require(std::abs(state(2)) < 0.01 * peak_pitch, "the pitch disturbance must be attenuated");
    require(std::abs(state(0)) < 0.20 * peak_position, "the induced position error must decay");
  }

  // 8. 与 LQR 基线在同一模型、同一初态、同一执行器限幅下的对等性。第一阶段
  //    关注的是模型、符号和方向约定是否正确，因此验收标准是"MPC 紧跟无穷
  //    时域 LQR"，而非"MPC 更优"。
  {
    const Eigen::Vector4d start(0.30, 0.0, 0.08, 0.0);
    Eigen::RowVector4d deployed_gain;
    deployed_gain << -5.931603, -45.087129, -181.972969, -43.390225;

    LinearMpc mpc;
    require(mpc.configure(plant.Ad, plant.Bd, plant.Q, 1.0, plant.P, plant.K, base),
      "configure failed");

    const int ticks = 2000;
    double deviation = 0.0;
    double reference_scale = 0.0;
    double mpc_torque = 0.0;
    double lqr_torque = 0.0;
    Eigen::Vector4d mpc_state = start;
    Eigen::Vector4d lqr_state = start;
    Eigen::Vector4d fixed_state = start;
    for (int tick = 0; tick < ticks; ++tick)
    {
      LinearMpcDiagnostics diagnostics;
      const double u_mpc = mpc.solve(mpc_state, diagnostics);
      double u_lqr = -plant.K.dot(lqr_state);
      u_lqr = std::max(-20.0, std::min(20.0, u_lqr));
      double u_fixed = -deployed_gain.dot(fixed_state);
      u_fixed = std::max(-20.0, std::min(20.0, u_fixed));
      mpc_torque = std::max(mpc_torque, std::abs(u_mpc));
      lqr_torque = std::max(lqr_torque, std::abs(u_lqr));
      deviation = std::max(deviation, (mpc_state - lqr_state).cwiseAbs().maxCoeff());
      reference_scale = std::max(reference_scale, lqr_state.cwiseAbs().maxCoeff());
      mpc_state = step(plant, mpc_state, u_mpc);
      lqr_state = step(plant, lqr_state, u_lqr);
      fixed_state = step(plant, fixed_state, u_fixed);
    }
    std::printf("parity over %.1f s: max |e_MPC - e_LQR| = %.4e (response scale %.3f, ratio %.2f%%), "
      "peak |u| MPC=%.2f LQR=%.2f, final states MPC=[%.4f %.4f %.4f] LQR=[%.4f %.4f %.4f] "
      "fixed=[%.4f %.4f %.4f]\n", ticks * 0.005, deviation, reference_scale,
      100.0 * deviation / reference_scale, mpc_torque, lqr_torque, mpc_state(0), mpc_state(1),
      mpc_state(2), lqr_state(0), lqr_state(1), lqr_state(2), fixed_state(0), fixed_state(1),
      fixed_state(2));
    require(deviation < 0.05 * reference_scale,
      "a horizon-20 MPC with the DARE terminal cost must track the same-weighting LQR");
    require(std::abs(mpc_torque - lqr_torque) < 0.05 * lqr_torque + 1.0e-6,
      "peak torque must be comparable to the same-weighting LQR");
    require(reference_scale > 0.3, "the reference response should start at the applied disturbance");
  }

  // 9. 高度调度适定性：同一组 Q/R/N 下，H 从 0.30 到 0.50 m 的每个高度都必须
  //    (a) 表列插值行是 stabilizing 种子，(b) DARE 有解，(c) 闭环渐近稳定，
  //    (d) 该高度的 MPC 在探测状态上无回退地求解且在执行器权限内。theta_eq 必须
  //    遵守"先插 y_com/z_com 再取 -atan2"的规则；并且每拍重建（冷启动 QP）与
  //    只配置一次（热启动 QP）必须给出同一个第一步输入，否则固定高度的回归无据。
  {
    Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
    q << 100.0, 0.0, 0.0, 0.0, 0.0, 5000.0, 0.0, 0.0, 0.0, 0.0, 3000.0, 0.0, 0.0, 0.0, 0.0, 1200.0;
    const double r_scheduled = 8.0;
    LinearMpcConfig scheduled = base;
    scheduled.max_qp_iterations = 300;

    double worst_seed_radius = 0.0;
    double worst_closed_loop = 0.0;
    int worst_passes = 0;
    for (int index = 0; index <= 40; ++index)
    {
      const double height = 0.30 + 0.005 * static_cast<double>(index);
      Eigen::Matrix4d A;
      Eigen::Vector4d B;
      bbot_balance_controller::lqr_plant::continuous_matrices(height, parameters, A, B);
      Eigen::Matrix4d ad;
      Eigen::Vector4d bd;
      bbot_balance_controller::lqr_plant::zoh_discretize(0.005, A, B, ad, bd);

      const Eigen::RowVector4d seed = table_seed(height);
      const double seed_radius =
        bbot_balance_controller::lqr_plant::spectral_radius(ad - bd * seed);
      require(seed_radius < 1.0, "the interpolated table row must stabilise Ad - Bd*K at H=0.30..0.50");
      worst_seed_radius = std::max(worst_seed_radius, seed_radius);

      Eigen::Matrix4d p;
      Eigen::RowVector4d gain;
      int passes = 0;
      double residual = 0.0;
      const bool solved = bbot_balance_controller::lqr_plant::solve_dare(
        ad, bd, q, r_scheduled, seed, p, gain, 100, passes, residual);
      require(solved, "the scheduled DARE must solve across the whole height range");
      worst_passes = std::max(worst_passes, passes);

      const double closed_loop =
        bbot_balance_controller::lqr_plant::spectral_radius(ad - bd * gain);
      require(closed_loop < 1.0, "the scheduled closed loop must be asymptotically stable");
      worst_closed_loop = std::max(worst_closed_loop, closed_loop);

      LinearMpc mpc;
      require(mpc.configure(ad, bd, q, r_scheduled, p, gain, scheduled),
        "configure must accept every scheduled height");
      const Eigen::Vector4d probe(0.02, 0.15, 0.03, 0.40);
      LinearMpcDiagnostics diagnostics;
      const double u = mpc.solve(probe, diagnostics);
      require(std::isfinite(u), "scheduled output must be finite");
      require(diagnostics.fallback_stage == MpcFallbackStage::kSolved,
        "the scheduled problem must solve without a fallback");
      require(std::abs(u) <= 20.0 + 1.0e-9, "scheduled output must respect the actuator bound");
    }
    std::printf("H=0.30..0.50 (41 点): worst seed |lambda|=%.6f  worst closed-loop |lambda|=%.6f  "
      "max Riccati passes=%d\n", worst_seed_radius, worst_closed_loop, worst_passes);

    // theta_eq 的规则：节点处表列必须与几何一致；中点处"先插值再 atan2"与
    // "先算角度再插值"必须是两个不同的数（否则本检查区分不了两种规则）。
    for (const TableRow &row : kTable)
    {
      const double geometric = -std::atan2(
        bbot_balance_controller::lqr_plant::suspended_body(row.height, parameters).y_com,
        bbot_balance_controller::lqr_plant::suspended_body(row.height, parameters).z_com);
      const double gap = std::abs(geometric - table_theta_eq(row.height));
      std::printf("theta_eq node H=%.4f: table=%.7f geometric=%.7f |gap|=%.2e\n", row.height,
        table_theta_eq(row.height), geometric, gap);
      require(gap < 1.0e-5, "table centroid must reproduce the suspended-body geometry at the nodes");
    }
    {
      const double mid = 0.375;
      const double adopted = table_theta_eq(mid);
      const double wrong_rule = 0.5 * (table_theta_eq(0.35) + table_theta_eq(0.40));
      const double geometric = -std::atan2(
        bbot_balance_controller::lqr_plant::suspended_body(mid, parameters).y_com,
        bbot_balance_controller::lqr_plant::suspended_body(mid, parameters).z_com);
      std::printf("theta_eq H=%.3f: 先插 y/z 再 atan2=%.7f，先算角度再插值=%.7f（差 %.2e），几何=%.7f\n",
        mid, adopted, wrong_rule, std::abs(adopted - wrong_rule), geometric);
      require(std::abs(adopted - wrong_rule) > 1.0e-9,
        "the two theta_eq rules must be distinguishable at mid-span");
      require(std::abs(adopted - geometric) < 5.0e-4,
        "the adopted rule must stay close to the true geometry between nodes");
    }

    // 每拍重建 vs 只配置一次：同一条闭环状态序列上的第一步输入必须一致。
    {
      Eigen::Matrix4d A;
      Eigen::Vector4d B;
      bbot_balance_controller::lqr_plant::continuous_matrices(0.40, parameters, A, B);
      Eigen::Matrix4d ad;
      Eigen::Vector4d bd;
      bbot_balance_controller::lqr_plant::zoh_discretize(0.005, A, B, ad, bd);
      Eigen::Matrix4d p;
      Eigen::RowVector4d gain;
      int passes = 0;
      double residual = 0.0;
      require(bbot_balance_controller::lqr_plant::solve_dare(
          ad, bd, q, r_scheduled, table_seed(0.40), p, gain, 100, passes, residual),
        "Riccati failed at the regression height");

      LinearMpc warm;
      require(warm.configure(ad, bd, q, r_scheduled, p, gain, scheduled), "configure failed");
      LinearMpc rebuilt;
      require(rebuilt.configure(ad, bd, q, r_scheduled, p, gain, scheduled), "configure failed");

      Eigen::Vector4d state(0.0, 0.0, 0.025, 0.0);
      double worst_gap = 0.0;
      for (int tick = 0; tick < 300; ++tick)
      {
        if (tick == 120)
        {
          state += Eigen::Vector4d(0.0, 0.0, 0.05, 0.3);
        }
        LinearMpcDiagnostics first;
        const double u_warm = warm.solve(state, first);
        rebuilt.configure(ad, bd, q, r_scheduled, p, gain, scheduled);
        LinearMpcDiagnostics second;
        const double u_rebuilt = rebuilt.solve(state, second);
        worst_gap = std::max(worst_gap, std::abs(u_warm - u_rebuilt));
        require(second.fallback_stage == first.fallback_stage,
          "rebuilding every cycle must not change the fallback stage");
        state = ad * state + bd * u_warm;
      }
      std::printf("每拍重建 vs 只配置一次 (300 拍): max |u difference| = %.3e Nm\n", worst_gap);
      require(worst_gap < 1.0e-9, "per-cycle rebuild must reproduce the once-configured MPC output");
    }
  }

  std::printf("test_linear_mpc: all checks passed\n");
  return 0;
}
