#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace bbot_jump
{

/// @brief 五次多项式轨迹生成器 (C2 连续)
struct QuinticTrajectory
{
    double t0 = 0.0;
    double tf = 0.0;
    double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, a4 = 0.0, a5 = 0.0;

    void init(double t_start, double duration,
              double z0, double v0, double acc0,
              double zf, double vf, double accf)
    {
        t0 = t_start;
        tf = t_start + duration;
        double T = (duration > 1e-4) ? duration : 1e-4;

        a0 = z0;
        a1 = v0 * T;
        a2 = 0.5 * acc0 * T * T;

        double delta_z = zf - (a0 + a1 + a2);
        double delta_v = vf * T - (a1 + 2.0 * a2);
        double delta_a = accf * T * T - 2.0 * a2;

        a3 = 10.0 * delta_z - 4.0 * delta_v + 0.5 * delta_a;
        a4 = -15.0 * delta_z + 7.0 * delta_v - 1.0 * delta_a;
        a5 = 6.0 * delta_z - 3.0 * delta_v + 0.5 * delta_a;
    }

    void evaluate(double t, double & z_out, double & v_out, double & acc_out) const
    {
        double duration = tf - t0;
        if (duration <= 1e-4) {
            z_out = a0;
            v_out = 0.0;
            acc_out = 0.0;
            return;
        }

        double tau = (t - t0) / duration;
        tau = std::clamp(tau, 0.0, 1.0);

        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        z_out = a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
        v_out = (a1 + 2.0 * a2 * tau + 3.0 * a3 * tau2 + 4.0 * a4 * tau3 + 5.0 * a5 * tau4) / duration;
        acc_out = (2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau2 + 20.0 * a5 * tau3) / (duration * duration);
    }

    bool is_finished(double t) const
    {
        return t >= tf;
    }
};

// 有限采样的轨迹准入检查；保留位置裕量，不能只验证端点连续。
inline bool flight_trajectory_admissible(const QuinticTrajectory & traj,
                                         double speed_limit, double acceleration_limit)
{
    for (int i = 0; i <= 256; ++i) {
        double q, v, a;
        traj.evaluate(traj.t0 + (traj.tf - traj.t0) * i / 256.0, q, v, a);
        if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a) ||
            std::abs(q) > 1.45 || std::abs(v) > speed_limit ||
            std::abs(a) > acceleration_limit) return false;
    }
    return true;
}

// Check the complete requested segment without clipping positions or rates.
// The boundary may be an actual initial state or a continuous previous target.
inline bool flight_segment_admissible(
    const std::array<double, 4> & q, const std::array<double, 4> & v,
    const std::array<double, 4> & a, const std::array<double, 4> & end,
    double duration)
{
    // Shorter intervals use the trajectory's stationary fallback and cannot
    // represent the requested endpoint, so they are inadmissible for flight.
    if (!std::isfinite(duration) || duration <= 1e-4) return false;
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i]) || !std::isfinite(v[i]) ||
            !std::isfinite(a[i]) || !std::isfinite(end[i])) return false;
        QuinticTrajectory segment;
        segment.init(0.0, duration, q[i], v[i], a[i], end[i], 0.0, 0.0);
        const double speed_limit = (i % 2 == 0) ? 7.5 : 10.0;
        const double acceleration_limit = (i % 2 == 0) ? 240.0 : 320.0;
        if (!flight_trajectory_admissible(segment, speed_limit, acceleration_limit))
            return false;
    }
    return true;
}

inline bool flight_round_trip_admissible(
    const std::array<double, 4> & q, const std::array<double, 4> & v,
    const std::array<double, 4> & a, const std::array<double, 4> & mid,
    const std::array<double, 4> & end, double tuck_duration,
    double extend_duration, double time_available)
{
    if (!std::isfinite(tuck_duration) || tuck_duration <= 1e-4 ||
        !std::isfinite(extend_duration) || extend_duration <= 1e-4 ||
        !std::isfinite(time_available) || time_available < 0.0 ||
        !std::isfinite(tuck_duration + extend_duration) ||
        time_available < tuck_duration + extend_duration) return false;
    const std::array<double, 4> rest{};
    return flight_segment_admissible(q, v, a, mid, tuck_duration) &&
        flight_segment_admissible(mid, rest, rest, end, extend_duration);
}

struct FlightRoundTripPlan
{
    bool valid = false;
    double tuck_duration = 0.0;
    double extend_duration = 0.0;
};

// Preserve both requested configurations and the existing motion budgets.
// Allocate time to the monotone rest-to-rest deployment first, then search
// bounded tuck durations for the supplied (possibly moving) initial boundary.
inline FlightRoundTripPlan plan_flight_round_trip(
    const std::array<double, 4> & q, const std::array<double, 4> & v,
    const std::array<double, 4> & a, const std::array<double, 4> & mid,
    const std::array<double, 4> & end, double nominal_tuck,
    double nominal_extend, double time_available)
{
    FlightRoundTripPlan result;
    if (!std::isfinite(nominal_tuck) || nominal_tuck <= 1e-4 ||
        !std::isfinite(nominal_extend) || nominal_extend <= 1e-4 ||
        !std::isfinite(time_available) || time_available <= 0.0) return result;

    constexpr double control_period = .005;
    constexpr double flight_timeout = .60;
    constexpr int max_tuck_candidates = 120;
    const double budget = std::min(time_available, flight_timeout);
    double minimum_extend = nominal_extend;
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i]) || !std::isfinite(v[i]) ||
            !std::isfinite(a[i]) || !std::isfinite(mid[i]) ||
            !std::isfinite(end[i]) || std::abs(q[i]) > 1.45 ||
            std::abs(mid[i]) > 1.45 || std::abs(end[i]) > 1.45) return result;
        const double speed_limit = (i % 2 == 0) ? 7.5 : 10.0;
        const double acceleration_limit = (i % 2 == 0) ? 240.0 : 320.0;
        if (std::abs(v[i]) > speed_limit || std::abs(a[i]) > acceleration_limit)
            return result;
        const double delta = std::abs(end[i] - mid[i]);
        // Exact peaks for a zero-velocity, zero-acceleration quintic segment.
        minimum_extend = std::max(minimum_extend, 1.875 * delta / speed_limit);
        minimum_extend = std::max(minimum_extend,
            std::sqrt((10.0 / std::sqrt(3.0)) * delta / acceleration_limit));
    }
    const double extend_duration =
        std::ceil(minimum_extend / control_period) * control_period;
    if (!std::isfinite(extend_duration) || extend_duration > budget) return result;
    const std::array<double, 4> rest{};
    if (!flight_segment_admissible(mid, rest, rest, end, extend_duration)) return result;

    for (int candidate = 0; candidate < max_tuck_candidates; ++candidate) {
        const double tuck_duration = nominal_tuck + control_period * candidate;
        if (tuck_duration + extend_duration > budget) break;
        if (flight_segment_admissible(q, v, a, mid, tuck_duration)) {
            result.valid = true;
            result.tuck_duration = tuck_duration;
            result.extend_duration = extend_duration;
            return result;
        }
    }
    return result;
}

}  // namespace bbot_jump
