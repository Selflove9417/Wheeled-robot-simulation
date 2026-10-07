// Height-Scheduled Linear MPC 的离线可行性与耗时基准（不进控制器、不启动 ROS）。
//
// 编译运行（不在 CMake 里，手工一次即可）：
//   g++ -std=c++17 -O2 -I src/bbot_balance_controller/include
//       -I src/bbot_kinematics/include -I /usr/include/eigen3
//       -o /tmp/bench src/bbot_balance_controller/scripts/bench_mpc_height_scheduled.cpp
//   /tmp/bench 8          # 第二个参数是 R，默认 8
// 换 -O0 重编一次就是仓库默认构建（CMAKE_BUILD_TYPE 为空）下的口径。
//
// 四段输出：
//  A. 同一组 Q/R/N 下，H in [0.30, 0.50] 每个高度的 DARE 是否可解、闭环是否稳定、
//     种子增益（表列插值）是否在每个 H 上都 stabilizing。
//  B. 每拍重建 Ad/Bd -> ZOH -> DARE -> prediction/QP 的实测耗时。
//  C. 固定 H=0.40 时，"每拍重建（冷启动）" 与 "只配置一次（热启动）" 的输出差异。
//  D. frozen-scheduling 的准 LTV 闭环仿真（0.40->0.30->0.50->0.40，含模型偏置）。
//     注意 D 的控制器与植物同源，只能证明调度适定性，不能当整机结论。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "bbot_balance_controller/linear_mpc.hpp"
#include "bbot_balance_controller/lqr_plant_model.hpp"
#include "bbot_kinematics/robot_params.hpp"

using namespace bbot_balance_controller;
using bbot_kinematics::RobotParams;

namespace
{
struct GainRow
{
  double height, k_x, k_x_dot, k_theta, k_theta_dot, y_com, z_com;
};

const std::vector<GainRow> kTable = {
  {0.3000, -5.622712, -42.666506, -156.508986, -35.846746, -0.0276264, 0.3592154},
  {0.3500, -5.791278, -43.976682, -169.544125, -39.563240, -0.0258066, 0.4016316},
  {0.4000, -5.931603, -45.087129, -181.972969, -43.390225, -0.0234056, 0.4443432},
  {0.4500, -6.052411, -46.061241, -193.948669, -47.349665, -0.0203401, 0.4872441},
  {0.5000, -6.157159, -46.922551, -205.518217, -51.430573, -0.0164269, 0.5302655}};

GainRow interp_row(double h)
{
  if (h <= kTable.front().height) return kTable.front();
  if (h >= kTable.back().height) return kTable.back();
  for (std::size_t i = 0; i + 1 < kTable.size(); ++i)
  {
    const GainRow &lo = kTable[i];
    const GainRow &hi = kTable[i + 1];
    if (h >= lo.height && h <= hi.height)
    {
      const double r = (h - lo.height) / (hi.height - lo.height);
      auto lerp = [r](double a, double b) { return a + (b - a) * r; };
      return GainRow{h, lerp(lo.k_x, hi.k_x), lerp(lo.k_x_dot, hi.k_x_dot),
        lerp(lo.k_theta, hi.k_theta), lerp(lo.k_theta_dot, hi.k_theta_dot),
        lerp(lo.y_com, hi.y_com), lerp(lo.z_com, hi.z_com)};
    }
  }
  return kTable[2];
}

const double kTs = 0.005;
const int kHorizon = 20;

Eigen::Matrix4d make_q()
{
  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
  q << 100.0, 0.0, 0.0, 0.0, 0.0, 5000.0, 0.0, 0.0, 0.0, 0.0, 3000.0, 0.0, 0.0, 0.0, 0.0, 1200.0;
  return q;
}

LinearMpcConfig make_config()
{
  LinearMpcConfig c;
  c.horizon = kHorizon;
  c.total_torque_limit = 20.0;
  c.wheel_torque_limit = 10.0;
  c.theta_error_limit = 0.20;
  c.theta_slack_weight = 1.0e5;
  c.use_theta_band = true;
  c.max_qp_iterations = 300;
  return c;
}

struct AtHeight
{
  double h;
  double theta_eq;
  Eigen::Matrix4d A;
  Eigen::Vector4d B;
  Eigen::Matrix4d Ad;
  Eigen::Vector4d Bd;
  Eigen::Matrix4d P;
  Eigen::RowVector4d K_seed;
  Eigen::RowVector4d K_dare;
  double rho_seed;
  double rho_cl;
  double slowest_abs;
  int dare_iters;
  bool dare_ok;
  double l1_dist_seed_to_dare;
};

AtHeight build_at(double h, const RobotParams &p, double r_weight)
{
  AtHeight out;
  out.h = h;
  const GainRow row = interp_row(h);
  out.theta_eq = -std::atan2(row.y_com, row.z_com);
  out.K_seed << row.k_x, row.k_x_dot, row.k_theta, row.k_theta_dot;
  lqr_plant::continuous_matrices(h, p, out.A, out.B);
  lqr_plant::zoh_discretize(kTs, out.A, out.B, out.Ad, out.Bd);
  out.rho_seed = lqr_plant::spectral_radius(out.Ad - out.Bd * out.K_seed);
  int iters = 0;
  double resid = 0.0;
  out.dare_ok = lqr_plant::solve_dare(
    out.Ad, out.Bd, make_q(), r_weight, out.K_seed, out.P, out.K_dare, 100, iters, resid);
  out.dare_iters = iters;
  out.rho_cl = lqr_plant::spectral_radius(out.Ad - out.Bd * out.K_dare);
  Eigen::EigenSolver<Eigen::Matrix4d> es(out.Ad - out.Bd * out.K_dare, false);
  out.slowest_abs = 0.0;
  for (int i = 0; i < 4; ++i) out.slowest_abs = std::max(out.slowest_abs, std::abs(es.eigenvalues()(i)));
  out.l1_dist_seed_to_dare = (out.K_dare - out.K_seed).cwiseAbs().maxCoeff();
  return out;
}

double us_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

struct Stage
{
  double mean_us = 0.0;
  double p99_us = 0.0;
  double max_us = 0.0;
};

Stage summarize(std::vector<double> &v)
{
  Stage s;
  if (v.empty()) return s;
  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) sum += x;
  s.mean_us = sum / static_cast<double>(v.size());
  s.p99_us = v[static_cast<std::size_t>(0.99 * (v.size() - 1))];
  s.max_us = v.back();
  return s;
}
}  // namespace

int main(int argc, char **argv)
{
  RobotParams p;
  const double r_weight = (argc > 1) ? std::atof(argv[1]) : 8.0;
  const Eigen::Vector4d probe_state(0.02, 0.15, 0.03, 0.40);

  printf("=== A. 同一 Q=diag(100,5000,3000,1200), R=%g, N=%d 在 H 网格上的可解性 ===\n",
    r_weight, kHorizon);
  printf("%-7s %-10s %-9s %-8s %-9s %-9s %-10s %-9s %-9s %s\n", "H[m]", "theta_eq", "rho_seed",
    "DAREok", "iters", "rho_cl", "slowest|L|", "tau[s]", "maxKdiff", "K_dare");
  bool all_ok = true;
  double worst_seed_rho = 0.0;
  for (int i = 0; i <= 40; ++i)
  {
    const double h = 0.30 + 0.005 * i;
    const AtHeight a = build_at(h, p, r_weight);
    if (!a.dare_ok || !(a.rho_seed < 1.0) || !(a.rho_cl < 1.0)) all_ok = false;
    worst_seed_rho = std::max(worst_seed_rho, a.rho_seed);
    const double tau = (a.slowest_abs < 1.0 && a.slowest_abs > 0.0) ?
      -kTs / std::log(a.slowest_abs) : std::numeric_limits<double>::infinity();
    printf("%-7.3f %-+10.6f %-9.6f %-8s %-9d %-9.6f %-10.6f %-9.3f %-9.4f [%.3f %.3f %.3f %.3f]\n",
      h, a.theta_eq, a.rho_seed, a.dare_ok ? "yes" : "NO", a.dare_iters, a.rho_cl,
      a.slowest_abs, tau, a.l1_dist_seed_to_dare, a.K_dare(0), a.K_dare(1), a.K_dare(2),
      a.K_dare(3));
  }
  printf(">> DARE 全程可解且闭环稳定 = %s，最差种子谱半径 = %.6f\n",
    all_ok ? "YES" : "NO", worst_seed_rho);

  printf("\n=== A2. 五个名义高度上的 MPC 首动增益与 u(state) ===\n");
  printf("%-7s %-30s %-11s %-9s %-9s %-9s\n", "H[m]", "first_move_gain", "u_mpc[Nm]", "stage",
    "qp_it", "qp_us");
  for (double h : {0.30, 0.35, 0.40, 0.45, 0.50})
  {
    const AtHeight a = build_at(h, p, r_weight);
    LinearMpc mpc;
    if (!mpc.configure(a.Ad, a.Bd, make_q(), r_weight, a.P, a.K_dare, make_config()))
    {
      printf("%-7.3f configure FAILED\n", h);
      continue;
    }
    LinearMpcDiagnostics d;
    const Eigen::RowVector4d &fm = mpc.first_move_gain();
    const double u = mpc.solve(probe_state, d);
    printf("%-7.3f [%.3f %.3f %.3f %.3f] %+8.3f %-9s %-9d %8.1f\n", h, fm(0), fm(1),
      fm(2), fm(3), u, d.fallback_stage == MpcFallbackStage::kSolved ? "solved" : "other",
      d.qp_iterations, d.qp_solve_time_us);
  }

  printf("\n=== A3. d(u)/d(H)（同一状态、只换模型）===\n");
  {
    const double h0 = 0.40;
    const double dh = 0.00025;  // 一拍 5 ms 在 0.05 m/s 下的高度变化
    LinearMpc m0;
    LinearMpc m1;
    const AtHeight a0 = build_at(h0, p, r_weight);
    const AtHeight a1 = build_at(h0 + dh, p, r_weight);
    m0.configure(a0.Ad, a0.Bd, make_q(), r_weight, a0.P, a0.K_dare, make_config());
    m1.configure(a1.Ad, a1.Bd, make_q(), r_weight, a1.P, a1.K_dare, make_config());
    LinearMpcDiagnostics d0;
    LinearMpcDiagnostics d1;
    const double u0 = m0.solve(probe_state, d0);
    const double u1 = m1.solve(probe_state, d1);
    printf("u(%.5f)=%.6f Nm, u(%.5f)=%.6f Nm -> du=%.6f Nm/拍 (%.3f Nm/s)\n", h0, u0, h0 + dh, u1,
      u1 - u0, (u1 - u0) / kTs);
    printf("theta_eq(%.5f)=%.6f, theta_eq(%.5f)=%.6f -> dtheta=%.6f rad/拍 (%.4f rad/s)\n", h0,
      a0.theta_eq, h0 + dh, a1.theta_eq, a1.theta_eq - a0.theta_eq,
      (a1.theta_eq - a0.theta_eq) / kTs);
  }

  printf("\n=== B. 每拍重建各阶段耗时（H=0.40，%d 次重复）===\n", 3000);
  {
    const int reps = 3000;
    std::vector<double> t_ab, t_zoh, t_dare, t_conf, t_solve, t_total;
    Eigen::Matrix4d A;
    Eigen::Vector4d B;
    Eigen::Matrix4d Ad;
    Eigen::Vector4d Bd;
    Eigen::Matrix4d P;
    Eigen::RowVector4d K;
    const GainRow row = interp_row(0.40);
    Eigen::RowVector4d seed;
    seed << row.k_x, row.k_x_dot, row.k_theta, row.k_theta_dot;
    LinearMpc mpc;
    for (int i = 0; i < reps; ++i)
    {
      const double h = 0.40 + 0.000001 * (i % 5);
      auto t0 = std::chrono::steady_clock::now();
      lqr_plant::continuous_matrices(h, p, A, B);
      const double tA = us_since(t0);
      auto t1 = std::chrono::steady_clock::now();
      lqr_plant::zoh_discretize(kTs, A, B, Ad, Bd);
      const double tZ = us_since(t1);
      int it = 0;
      double res = 0.0;
      auto t2 = std::chrono::steady_clock::now();
      lqr_plant::solve_dare(Ad, Bd, make_q(), r_weight, seed, P, K, 100, it, res);
      const double tD = us_since(t2);
      auto t3 = std::chrono::steady_clock::now();
      mpc.configure(Ad, Bd, make_q(), r_weight, P, K, make_config());
      const double tC = us_since(t3);
      LinearMpcDiagnostics d;
      auto t4 = std::chrono::steady_clock::now();
      mpc.solve(probe_state, d);
      const double tS = us_since(t4);
      t_ab.push_back(tA);
      t_zoh.push_back(tZ);
      t_dare.push_back(tD);
      t_conf.push_back(tC);
      t_solve.push_back(tS);
      t_total.push_back(tA + tZ + tD + tC);
    }
    const Stage s_ab = summarize(t_ab);
    const Stage s_zoh = summarize(t_zoh);
    const Stage s_dare = summarize(t_dare);
    const Stage s_conf = summarize(t_conf);
    const Stage s_solve = summarize(t_solve);
    const Stage s_total = summarize(t_total);
    printf("A/B 连续模型      : mean %8.1f us  p99 %8.1f  max %8.1f\n", s_ab.mean_us, s_ab.p99_us,
      s_ab.max_us);
    printf("ZOH(expm 5x5)    : mean %8.1f us  p99 %8.1f  max %8.1f\n", s_zoh.mean_us, s_zoh.p99_us,
      s_zoh.max_us);
    printf("DARE(Kleinman)   : mean %8.1f us  p99 %8.1f  max %8.1f\n", s_dare.mean_us,
      s_dare.p99_us, s_dare.max_us);
    printf("预测阵+QP rebuild: mean %8.1f us  p99 %8.1f  max %8.1f\n", s_conf.mean_us,
      s_conf.p99_us, s_conf.max_us);
    printf("--- 每拍重建合计   : mean %8.1f us  p99 %8.1f  max %8.1f  (预算 5000 us)\n",
      s_total.mean_us, s_total.p99_us, s_total.max_us);
    printf("QP solve(热/冷)  : mean %8.1f us  p99 %8.1f  max %8.1f\n", s_solve.mean_us,
      s_solve.p99_us, s_solve.max_us);
    printf("重建+solve       : mean %8.1f us = 预算的 %.1f%%\n", s_total.mean_us + s_solve.mean_us,
      100.0 * (s_total.mean_us + s_solve.mean_us) / 5000.0);
  }

  printf("\n=== C. 固定 H=0.40：每拍重建（冷启动） vs 只配置一次（热启动）===\n");
  {
    const AtHeight a = build_at(0.40, p, r_weight);
    LinearMpc warm;
    warm.configure(a.Ad, a.Bd, make_q(), r_weight, a.P, a.K_dare, make_config());
    LinearMpc cold;
    cold.configure(a.Ad, a.Bd, make_q(), r_weight, a.P, a.K_dare, make_config());

    // 一条带扰动的 400 拍闭环：外部输入 a.Ad/a.Bd 演化，两个求解器同一状态。
    Eigen::Vector4d e = Eigen::Vector4d::Zero();
    e << 0.0, 0.0, 0.025, 0.0;         // 初始 0.025 rad 俯仰偏差
    const Eigen::Vector4d kick(0.0, 0.0, 0.05, 0.3);
    double max_du = 0.0;
    int stage_diff = 0;
    long warm_iters = 0;
    long cold_iters = 0;
    int warm_cap = 0;
    int cold_cap = 0;
    for (int k = 0; k < 400; ++k)
    {
      if (k == 100) e += kick;
      if (k == 250) e -= kick;
      LinearMpcDiagnostics dw;
      LinearMpcDiagnostics dc;
      const double uw = warm.solve(e, dw);
      cold.configure(a.Ad, a.Bd, make_q(), r_weight, a.P, a.K_dare, make_config());
      const double uc = cold.solve(e, dc);
      max_du = std::max(max_du, std::abs(uw - uc));
      stage_diff += (dw.fallback_stage == dc.fallback_stage) ? 0 : 1;
      warm_iters += dw.qp_iterations;
      cold_iters += dc.qp_iterations;
      warm_cap += (dw.qp_status != QpStatus::kConverged) ? 1 : 0;
      cold_cap += (dc.qp_status != QpStatus::kConverged) ? 1 : 0;
      e = a.Ad * e + a.Bd * uw;
    }
    printf("max|u_warm - u_cold| = %.3e Nm，stage 不同拍数 = %d/400\n", max_du, stage_diff);
    printf("QP 平均迭代：热启动 %.1f，冷启动 %.1f；非收敛拍数：热 %d，冷 %d\n",
      warm_iters / 400.0, cold_iters / 400.0, warm_cap, cold_cap);
  }

  printf("\n=== D. frozen-scheduling 准 LTV 闭环仿真（真实 Gazebo 之前的离线判据）===\n");
  printf("场景：H 以 0.05 m/s 走 0.40->0.30->0.50->0.40；控制器每拍按 H_model 重建，\n"
         "      植物按 H_plant 演化（H_plant = H_model + bias，代表模型误差）。\n"
         "      theta_eq(H) 的移动被建模成 pitch 误差的瞬时参考跳变。\n");
  for (double bias : {0.0, 0.01, 0.02})
  {
    double h = 0.40;
    const std::vector<double> waypoints = {0.30, 0.50, 0.40};
    std::size_t wp = 0;
    Eigen::Vector4d e = Eigen::Vector4d::Zero();
    double theta_prev = interp_row(h).y_com, z_prev = interp_row(h).z_com;
    double prev_theta = -std::atan2(theta_prev, z_prev);
    double peak_theta_err = 0.0;
    double peak_u = 0.0;
    double peak_x_err = 0.0;
    long sat_count = 0;
    long cycles = 0;
    int stage_other = 0;
    // 先站 2 s，再走三个阶段各 4 s，最后留 4 s 观察沉降
    const int total = static_cast<int>((2.0 + 3 * 4.0 + 4.0) / kTs);
    const double t_start_move = 2.0;
    for (int k = 0; k < total; ++k)
    {
      const double t = k * kTs;
      if (wp < waypoints.size() && t >= t_start_move)
      {
        const double goal = waypoints[wp];
        const double step = 0.05 * kTs;
        if (std::abs(h - goal) <= step)
        {
          h = goal;
          ++wp;
        }
        else
        {
          h += (goal > h ? step : -step);
        }
      }
      const GainRow row = interp_row(h);
      const double theta_now = -std::atan2(row.y_com, row.z_com);
      e(2) -= (theta_now - prev_theta);      // 参考在机身下面移动
      prev_theta = theta_now;

      const AtHeight model = build_at(h, p, r_weight);
      LinearMpc mpc;
      mpc.configure(model.Ad, model.Bd, make_q(), r_weight, model.P, model.K_dare, make_config());
      LinearMpcDiagnostics d;
      const double u = mpc.solve(e, d);
      ++cycles;
      sat_count += d.total_torque_saturated ? 1 : 0;
      stage_other += d.fallback_stage == MpcFallbackStage::kSolved ? 0 : 1;

      const AtHeight plant = build_at(h + bias, p, r_weight);
      e = plant.Ad * e + plant.Bd * u;

      peak_theta_err = std::max(peak_theta_err, std::abs(e(2)));
      peak_u = std::max(peak_u, std::abs(u));
      peak_x_err = std::max(peak_x_err, std::abs(e(0)));
    }
    printf("bias=%.3f m: 峰值|theta_err|=%.4f rad  峰值|u|=%.2f Nm  饱和 %.1f%%  非 solved 拍 %d  "
           "终端 |theta_err|=%.5f |x_err|=%.4f m\n",
      bias, peak_theta_err, peak_u, 100.0 * sat_count / cycles, stage_other,
      std::abs(e(2)), std::abs(e(0)));
  }
  return 0;
}
