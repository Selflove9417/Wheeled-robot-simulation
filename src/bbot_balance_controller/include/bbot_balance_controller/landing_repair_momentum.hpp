#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace bbot_jump
{
// Private diagnostic for the landing-repair experiment. This is a seven-body
// CAD estimate, not a measured torque or an estimate of external contact force.
struct LandingMomentum
{
    bool valid = false;
    double total = 0.0;
    double body_rate_term = 0.0;
    double joint_rate_term = 0.0;
    double wheel_relative_spin_term = 0.0;
    double pitch_inertia = 0.0;
};

inline LandingMomentum landing_momentum(
    const JointVector & q, const JointVector & v,
    const std::array<double, 2> & wheel_relative_rate,
    double pitch_rate, double body_mass)
{
    LandingMomentum out;
    if (!q.allFinite() || !v.allFinite() || !std::isfinite(pitch_rate) ||
        !std::isfinite(body_mass) || body_mass <= 0.0 ||
        !std::isfinite(wheel_relative_rate[0]) ||
        !std::isfinite(wheel_relative_rate[1])) return out;
    const auto mass = flight_pitch_joint_mass_matrix(q, body_mass);
    if (!mass.allFinite()) return out;
    // The production floating model includes wheel orbital inertia but leaves
    // independent axial spin out. Add Iw*(theta_dot+qh_dot+qk_dot+w_dot).
    constexpr double wheel_inertia = 0.006481;
    out.pitch_inertia = mass(0, 0) + 2.0 * wheel_inertia;
    if (out.pitch_inertia <= 1e-9) return out;
    out.body_rate_term = -out.pitch_inertia * pitch_rate;
    out.joint_rate_term = mass.block<1, 4>(0, 1).dot(v) +
                          wheel_inertia * v.sum();
    out.wheel_relative_spin_term = wheel_inertia *
        (wheel_relative_rate[0] + wheel_relative_rate[1]);
    out.total = out.body_rate_term + out.joint_rate_term +
                out.wheel_relative_spin_term;
    out.valid = std::isfinite(out.total);
    return out;
}

inline double landing_pitch_rate_for_momentum(
    const JointVector & q, const JointVector & v,
    const std::array<double, 2> & wheel_relative_rate,
    double total_momentum, double body_mass)
{
    const auto at_rest = landing_momentum(q, v, wheel_relative_rate, 0.0, body_mass);
    if (!at_rest.valid || !std::isfinite(total_momentum))
        return std::numeric_limits<double>::quiet_NaN();
    return (at_rest.joint_rate_term + at_rest.wheel_relative_spin_term -
            total_momentum) / at_rest.pitch_inertia;
}

struct LandingJointSample
{
    double stamp = -1.0;
    JointVector q = JointVector::Zero();
    JointVector v = JointVector::Zero();
    std::array<double, 2> wheel_rate{};
};

class LandingJointHistory
{
public:
    bool push(const LandingJointSample & sample)
    {
        if (!std::isfinite(sample.stamp) || sample.stamp <= 0.0 ||
            !sample.q.allFinite() || !sample.v.allFinite() ||
            !std::isfinite(sample.wheel_rate[0]) ||
            !std::isfinite(sample.wheel_rate[1])) return false;
        if (!history_.empty() && sample.stamp < history_.back().stamp)
            history_.clear(); // Simulation reset must not reuse old samples.
        if (!history_.empty() && std::abs(sample.stamp - history_.back().stamp) < 1e-9)
            history_.back() = sample;
        else history_.push_back(sample);
        while (history_.size() > 64) history_.pop_front();
        return true;
    }

    // No extrapolation, nearest-neighbour pairing, or held velocity pretending
    // to be synchronous: the raw IMU and all six velocities must share a stamp.
    bool exact(double imu_stamp, double now, LandingJointSample & out) const
    {
        if (!std::isfinite(imu_stamp) || !std::isfinite(now) ||
            imu_stamp <= 0.0 || now < imu_stamp || now - imu_stamp > 0.020)
            return false;
        for (auto it = history_.rbegin(); it != history_.rend(); ++it)
            if (std::abs(it->stamp - imu_stamp) <= 1e-9) {
                out = *it;
                return true;
            }
        return false;
    }

private:
    std::deque<LandingJointSample> history_;
};
} // namespace bbot_jump
