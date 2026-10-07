#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include "bbot_balance_controller/flight_joint_pd.hpp"
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"

namespace bbot_jump
{
struct StampedJointKinematics
{
    double stamp{-1.0};
    JointVector q{JointVector::Zero()};
    JointVector qdot{JointVector::Zero()};
    Eigen::Vector2d wheel_rates{Eigen::Vector2d::Zero()};
};

struct GroundMomentumReference
{
    bool valid{false};
    double pitch{0.0};
    double raw_rate{0.0};
    double bounded_rate{0.0};
    double effective_inertia{0.0};
};

// Solve the CAD geometry for a forward COM offset from the wheel axle.  Pitch
// uses the controller's forward-positive convention.  The caller remains
// responsible for a smooth transition into/out of this reference.
inline bool ground_com_forward_pitch_reference(
    const JointVector & q, double body_mass, double target_forward,
    double pitch_limit, double & pitch_reference)
{
    pitch_reference = 0.0;
    if (!q.allFinite() || !std::isfinite(body_mass) || body_mass <= 0.0 ||
        !std::isfinite(target_forward) || target_forward < 0.0 ||
        !std::isfinite(pitch_limit) || pitch_limit <= 0.0)
        return false;
    const std::array<double, 4> q_array{q[0], q[1], q[2], q[3]};
    const auto geometry = centroidal_geometry(q_array, body_mass);
    const Eigen::Vector3d r = geometry.com - geometry.axle;
    const double radius = std::hypot(r.y(), r.z());
    if (!geometry.com.allFinite() || !geometry.axle.allFinite() ||
        !std::isfinite(radius) || radius <= 1e-6 || target_forward > radius)
        return false;
    const double raw = std::asin(target_forward / radius) - std::atan2(r.y(), r.z());
    if (!std::isfinite(raw) || std::abs(raw) > pitch_limit)
        return false;
    pitch_reference = raw;
    return true;
}

inline bool ground_h_momentum_pitch_rate_reference(
    double aligned_pitch_rate, double momentum, double target_momentum,
    double effective_inertia, double rate_limit, GroundMomentumReference & out)
{
    out = {};
    if (!std::isfinite(aligned_pitch_rate) || !std::isfinite(momentum) ||
        !std::isfinite(target_momentum) || target_momentum < 0.0 ||
        !std::isfinite(effective_inertia) || effective_inertia <= 1e-6 ||
        !std::isfinite(rate_limit) || rate_limit <= 0.0)
        return false;
    const double raw = aligned_pitch_rate +
        (momentum - target_momentum) / effective_inertia;
    if (!std::isfinite(raw))
        return false;
    out.valid = true;
    out.raw_rate = raw;
    out.bounded_rate = std::clamp(raw, -rate_limit, rate_limit);
    out.effective_inertia = effective_inertia;
    return true;
}

inline double smooth_reference_blend(double elapsed, double duration)
{
    if (!std::isfinite(elapsed) || !std::isfinite(duration) || duration <= 0.0)
        return 0.0;
    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

inline double slew_reference(double previous, double target, double max_rate, double dt)
{
    if (!std::isfinite(previous) || !std::isfinite(target) ||
        !std::isfinite(max_rate) || max_rate < 0.0 ||
        !std::isfinite(dt) || dt <= 0.0)
        return previous;
    return previous + std::clamp(target - previous, -max_rate * dt, max_rate * dt);
}

struct GovernedGroundPitchReference
{
    bool valid{false};
    double pitch{0.0};
    double rate{0.0};
    double bounded_cad_pitch{0.0};
    double tracking_error{0.0};
};

// Integrate the H-derived rate reference and use the CAD COM target only as a
// bounded attraction. The reference can never lead measured pitch by more
// than max_lead; returned rate is the derivative of the returned angle.
inline GovernedGroundPitchReference govern_ground_pitch_reference(
    double previous_pitch_ref, double measured_pitch, double measured_pitch_rate,
    double cad_pitch,
    double h_rate_target, double previous_rate_ref, double dt,
    double max_lead = 0.08, double max_abs_pitch = 0.45,
    double max_rate = 1.20, double max_rate_slew = 30.0,
    double cad_attraction_gain = 2.0,
    double minimum_pitch = -std::numeric_limits<double>::infinity())
{
    GovernedGroundPitchReference out;
    if (!std::isfinite(previous_pitch_ref) || !std::isfinite(measured_pitch) ||
        !std::isfinite(measured_pitch_rate) || !std::isfinite(cad_pitch) ||
        !std::isfinite(h_rate_target) ||
        !std::isfinite(previous_rate_ref) || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(max_lead) || max_lead <= 0.0 ||
        !std::isfinite(max_abs_pitch) || max_abs_pitch <= 0.0 ||
        !std::isfinite(max_rate) || max_rate <= 0.0 ||
        !std::isfinite(max_rate_slew) || max_rate_slew < 0.0 ||
        !std::isfinite(cad_attraction_gain) || cad_attraction_gain < 0.0 ||
        std::isnan(minimum_pitch) || minimum_pitch == std::numeric_limits<double>::infinity() ||
        (std::isfinite(minimum_pitch) &&
            (minimum_pitch < -max_abs_pitch || minimum_pitch > max_abs_pitch)) ||
        std::abs(measured_pitch) > max_abs_pitch)
        return out;

    out.bounded_cad_pitch = std::clamp(
        cad_pitch, -max_abs_pitch, std::min(max_abs_pitch, measured_pitch + max_lead));
    const double lower = std::isfinite(minimum_pitch)
        ? std::max(-max_abs_pitch, minimum_pitch) : -max_abs_pitch;
    const double upper = std::min(max_abs_pitch, measured_pitch + max_lead);
    if (lower > upper || previous_pitch_ref < lower) return out;
    const double lead_headroom = std::max(0.0, upper - previous_pitch_ref);
    // Discrete one-step stopping envelope: after this update the relative
    // reference rate must still be stoppable before exhausting the lead gap.
    const double accel_step = max_rate_slew * dt;
    const double max_relative_rate =
        std::sqrt(accel_step * accel_step +
                  2.0 * max_rate_slew * lead_headroom) - accel_step;
    const double rate_upper_from_lead = std::min(
        max_rate, measured_pitch_rate + max_relative_rate);
    if (rate_upper_from_lead < -max_rate) return out;

    // Optional lower pitch bound uses the matching discrete stopping envelope.
    // The returned angle is always integrated from the returned rate; we never
    // clamp the angle to the floor after the rate has been selected.
    double rate_lower_from_floor = -max_rate;
    if (std::isfinite(minimum_pitch)) {
        const double floor_headroom = std::max(0.0, previous_pitch_ref - lower);
        const double max_downward_rate = std::sqrt(
            accel_step * accel_step + 2.0 * max_rate_slew * floor_headroom) - accel_step;
        rate_lower_from_floor = -std::min(max_rate, max_downward_rate);
    }
    if (rate_lower_from_floor > rate_upper_from_lead) return out;
    const double rate_request = std::clamp(
        h_rate_target + cad_attraction_gain *
            (out.bounded_cad_pitch - previous_pitch_ref),
        rate_lower_from_floor, rate_upper_from_lead);
    const double rate = slew_reference(
        previous_rate_ref, rate_request, max_rate_slew, dt);
    const double candidate = previous_pitch_ref + rate * dt;
    const double rate_floor = std::max(-max_rate, previous_rate_ref - max_rate_slew * dt);
    const double rate_ceiling = std::min(max_rate, previous_rate_ref + max_rate_slew * dt);
    const double reachable_lower = std::max(lower, previous_pitch_ref + rate_floor * dt);
    const double reachable_upper = std::min(upper, previous_pitch_ref + rate_ceiling * dt);
    if (reachable_lower > reachable_upper) return out;
    const double limited = std::clamp(candidate, reachable_lower, reachable_upper);
    if (!std::isfinite(limited)) return GovernedGroundPitchReference{};
    const double applied_rate = (limited - previous_pitch_ref) / dt;
    if (std::isfinite(minimum_pitch) &&
        (limited < lower - 1e-12 || std::abs(applied_rate) > max_rate + 1e-12 ||
         std::abs(applied_rate - previous_rate_ref) > max_rate_slew * dt + 1e-12))
        return out;
    out.valid = true;
    out.pitch = limited;
    out.rate = applied_rate;
    out.tracking_error = measured_pitch - limited;
    return out;
}

// Keep complete sensor rows so the gyro and all joint rates can be evaluated
// at the same IMU timestamp. There is deliberately no extrapolation.
class JointKinematicsHistory
{
  public:
    bool push(const StampedJointKinematics &sample)
    {
        last_push_invalidated_ = false;
        if (!std::isfinite(sample.stamp) || sample.stamp <= 0.0 ||
            !sample.q.allFinite() || !sample.qdot.allFinite() ||
            !sample.wheel_rates.allFinite())
        {
            samples_.clear();
            last_push_invalidated_ = true;
            return false;
        }
        if (!samples_.empty())
        {
            const auto &last = samples_.back();
            if (sample.stamp < last.stamp)
            {
                samples_.clear();
                last_push_invalidated_ = true;
            }
            else if (sample.stamp == last.stamp)
            {
                const bool same = (sample.q - last.q).norm() <= 1e-12 &&
                    (sample.qdot - last.qdot).norm() <= 1e-12 &&
                    (sample.wheel_rates - last.wheel_rates).norm() <= 1e-12;
                if (!same)
                {
                    samples_.clear();
                    last_push_invalidated_ = true;
                }
                return false;
            }
            else if (sample.stamp - last.stamp > 0.080)
            {
                samples_.clear();
                last_push_invalidated_ = true;
            }
        }
        samples_.push_back(sample);
        while (samples_.size() > 64) samples_.pop_front();
        return true;
    }

    void reset()
    {
        samples_.clear();
        last_push_invalidated_ = true;
    }
    bool last_push_invalidated() const { return last_push_invalidated_; }

    bool exact(double stamp, StampedJointKinematics &out) const
    {
        if (!std::isfinite(stamp) || stamp <= 0.0) return false;
        const auto it = std::lower_bound(samples_.begin(), samples_.end(), stamp,
            [](const StampedJointKinematics &sample, double value) {
                return sample.stamp < value;
            });
        if (it == samples_.end() || it->stamp != stamp) return false;
        out = *it;
        return true;
    }

  private:
    std::deque<StampedJointKinematics> samples_;
    bool last_push_invalidated_{false};
};

class StampedAngularMomentumObserver
{
  public:
    static constexpr double kFreshSeconds = 0.020;

    void reset()
    {
        stamp_ = -1.0;
        value_ = 0.0;
        raw_pitch_rate_ = 0.0;
        valid_ = false;
    }

    // A new estimate is accepted only from a same-stamp IMU/joint row. An
    // already-complete estimate may be held briefly between 100Hz IMU frames.
    bool update(double now, double imu_stamp, double raw_pitch_rate,
                const JointKinematicsHistory &history, double body_mass)
    {
        if (!std::isfinite(now) || !std::isfinite(imu_stamp) ||
            !std::isfinite(raw_pitch_rate) || !std::isfinite(body_mass) ||
            body_mass <= 0.0 || imu_stamp <= 0.0 || imu_stamp > now ||
            now - imu_stamp > kFreshSeconds)
        {
            reset();
            return false;
        }
        if (stamp_ >= 0.0 && imu_stamp < stamp_)
        {
            reset();
            return false;
        }
        if (imu_stamp > stamp_)
        {
            StampedJointKinematics sample;
            if (history.exact(imu_stamp, sample))
            {
                double momentum = 0.0;
                if (!flight_planar_angular_momentum(sample.q, sample.qdot,
                        -raw_pitch_rate, sample.wheel_rates, body_mass, momentum))
                {
                    reset();
                    return false;
                }
                stamp_ = imu_stamp;
                value_ = momentum;
                raw_pitch_rate_ = raw_pitch_rate;
                valid_ = true;
            }
        }
        return valid(now);
    }

    bool valid(double now) const
    {
        return valid_ && std::isfinite(now) && now >= stamp_ &&
            now - stamp_ <= kFreshSeconds;
    }
    double momentum() const { return valid_ ? value_ : 0.0; }
    double raw_pitch_rate() const { return valid_ ? raw_pitch_rate_ : 0.0; }
    double stamp() const { return valid_ ? stamp_ : -1.0; }

  private:
    double stamp_{-1.0};
    double value_{0.0};
    double raw_pitch_rate_{0.0};
    bool valid_{false};
};

struct MomentumFlightBudget
{
    bool valid{false};
    int rejection_code{0}; // 1=state, 2=remaining time, 3=joint path, 4=wheel/H/pitch
    int rejection_detail{0};
    double tuck_duration{0.0};
    double extend_duration{0.0};
    double arrest_duration{0.0};
    double final_pitch{0.0};
    double final_pitch_rate{0.0};
    double max_abs_pitch_rate{0.0};
    double max_pitch_deviation{0.0};
    double max_abs_wheel_rate{0.0};
    double max_wheel_torque{0.0};
    double max_required_hip_torque{0.0};
    double max_required_knee_torque{0.0};
    double h_state_stamp{-1.0};
    double joint_state_stamp{-1.0};
    double com_state_stamp{-1.0};
    double forward_state_stamp{-1.0};
    double first_contact_time{0.0};
    double first_contact_pitch{0.0};
    double first_contact_pitch_rate{0.0};
    double max_wheel_clearance{0.0};
    double ground_handoff_blend_at_contact{0.0};
    double ground_capture_forward_velocity{0.0};
    std::array<double, 4> arrest_end_q{};
    std::array<double, 4> tuck_target_q{};
    std::array<double, 4> landing_target_q{};
};

inline double minimum_wheel_bottom_clearance(
    const CentroidalGeometry & geometry, double pitch, double com_world_z,
    double wheel_radius, double ground_z)
{
    if (!std::isfinite(pitch) || !std::isfinite(com_world_z) ||
        !std::isfinite(wheel_radius) || !std::isfinite(ground_z))
        return std::numeric_limits<double>::quiet_NaN();
    const double com_relative_z = rotate_about_hip(-pitch, geometry.com).z();
    const double left = com_world_z +
        rotate_about_hip(-pitch, geometry.axle_left).z() - com_relative_z - wheel_radius - ground_z;
    const double right = com_world_z +
        rotate_about_hip(-pitch, geometry.axle_right).z() - com_relative_z - wheel_radius - ground_z;
    return std::min(left, right);
}

inline bool fast_centroidal_positions(
    const JointVector & q, double body_mass, Eigen::Vector3d & com,
    Eigen::Vector3d & axle_left, Eigen::Vector3d & axle_right)
{
    com.setZero(); axle_left.setZero(); axle_right.setZero();
    if (!q.allFinite() || !std::isfinite(body_mass) || body_mass <= 0.0) return false;
    com = body_mass * Eigen::Vector3d(0.20001846, 0.13261282, 0.05396677);
    for (int side=0; side<2; ++side) {
        const int h=2*side, k=h+1;
        const Eigen::Vector3d hip(side==0?0.3032:0.0965,0.125,-0.07);
        const Eigen::Vector3d thigh=rotate_about_hip(q[h],
            {side==0?0.06107357:-0.06077357,-0.13690699,-0.02116697});
        const Eigen::Vector3d knee=rotate_about_hip(q[h],
            {side==0?0.072:-0.0667,-0.29348091,-0.06220095});
        const Eigen::Vector3d shank=rotate_about_hip(q[h]+q[k],
            {side==0?-0.01104398:0.00604399,0.11538205,-0.08532288});
        const Eigen::Vector3d wheel=rotate_about_hip(q[h]+q[k],
            {side==0?-0.022:-0.0405,0.28210870,-0.19553796});
        const Eigen::Vector3d wheel_com=hip+knee+wheel+
            Eigen::Vector3d(side==0?0.03825001:0.01925,0.0,0.0);
        com += 1.2*(hip+thigh)+0.8*(hip+knee+shank)+2.0*wheel_com;
        const Eigen::Vector3d axle=hip+knee+wheel;
        if (side==0) axle_left=axle; else axle_right=axle;
    }
    com /= body_mass+8.0;
    return com.allFinite() && axle_left.allFinite() && axle_right.allFinite();
}

inline double fast_minimum_wheel_bottom_clearance(
    const Eigen::Vector3d & com, const Eigen::Vector3d & axle_left,
    const Eigen::Vector3d & axle_right, double pitch, double com_world_z,
    double wheel_radius, double ground_z)
{
    if (!com.allFinite() || !axle_left.allFinite() || !axle_right.allFinite() ||
        !std::isfinite(pitch) || !std::isfinite(com_world_z) ||
        !std::isfinite(wheel_radius) || !std::isfinite(ground_z))
        return std::numeric_limits<double>::quiet_NaN();
    const double c=std::cos(pitch), s=std::sin(pitch);
    const double com_relative_z=c*com.z()-s*com.y();
    const double left_z=c*axle_left.z()-s*axle_left.y();
    const double right_z=c*axle_right.z()-s*axle_right.y();
    return std::min(com_world_z+left_z-com_relative_z-wheel_radius-ground_z,
                    com_world_z+right_z-com_relative_z-wheel_radius-ground_z);
}

inline double fast_minimum_wheel_bottom_clearance_yz(
    const Eigen::Vector2d & com_yz, const Eigen::Vector2d & axle_left_yz,
    const Eigen::Vector2d & axle_right_yz, double pitch, double com_world_z,
    double wheel_radius, double ground_z)
{
    if (!com_yz.allFinite() || !axle_left_yz.allFinite() || !axle_right_yz.allFinite() ||
        !std::isfinite(pitch) || !std::isfinite(com_world_z) ||
        !std::isfinite(wheel_radius) || !std::isfinite(ground_z))
        return std::numeric_limits<double>::quiet_NaN();
    const double c=std::cos(pitch), s=std::sin(pitch);
    const double com_relative_z=c*com_yz.y()-s*com_yz.x();
    const double left_z=c*axle_left_yz.y()-s*axle_left_yz.x();
    const double right_z=c*axle_right_yz.y()-s*axle_right_yz.x();
    return std::min(com_world_z+left_z-com_relative_z-wheel_radius-ground_z,
                    com_world_z+right_z-com_relative_z-wheel_radius-ground_z);
}

inline bool momentum_consistent_wheel_command(
    const JointVector & q, const JointVector & qdot, double total_h,
    double body_mass, double pitch, double measured_pitch_rate,
    double pitch_reference, double pitch_reference_rate, double wheel_radius,
    double max_relative_wheel_rate, double & wheel_linear_command,
    double & desired_pitch_rate)
{
    wheel_linear_command = 0.0;
    desired_pitch_rate = 0.0;
    if (!q.allFinite() || !qdot.allFinite() || !std::isfinite(total_h) ||
        !std::isfinite(body_mass) || body_mass <= 0.0 ||
        !std::isfinite(pitch) || !std::isfinite(measured_pitch_rate) ||
        !std::isfinite(pitch_reference) || !std::isfinite(pitch_reference_rate) ||
        !std::isfinite(wheel_radius) || wheel_radius <= 0.0 ||
        !std::isfinite(max_relative_wheel_rate) || max_relative_wheel_rate <= 0.0)
        return false;
    const double rate_error = measured_pitch_rate - pitch_reference_rate;
    desired_pitch_rate = std::clamp(
        pitch_reference_rate - 4.0 * (pitch - pitch_reference) - rate_error,
        -3.0, 3.0);
    double required_relative_rate = 0.0;
    if (!flight_symmetric_wheel_rate_for_momentum(
            q, qdot, -desired_pitch_rate, total_h, body_mass, required_relative_rate))
        return false;
    required_relative_rate = std::clamp(
        required_relative_rate, -max_relative_wheel_rate, max_relative_wheel_rate);
    wheel_linear_command = wheel_radius * required_relative_rate;
    return std::isfinite(wheel_linear_command);
}

inline bool momentum_consistent_wheel_command(
    const PitchJointMassMatrix & mass, const JointVector & qdot, double total_h,
    double pitch, double measured_pitch_rate, double pitch_reference,
    double pitch_reference_rate, double wheel_radius,
    double max_relative_wheel_rate, double & wheel_linear_command,
    double & desired_pitch_rate)
{
    wheel_linear_command = 0.0;
    desired_pitch_rate = 0.0;
    if (!mass.allFinite() || !qdot.allFinite() || !std::isfinite(total_h) ||
        !std::isfinite(pitch) || !std::isfinite(measured_pitch_rate) ||
        !std::isfinite(pitch_reference) || !std::isfinite(pitch_reference_rate) ||
        !std::isfinite(wheel_radius) || wheel_radius <= 0.0 ||
        !std::isfinite(max_relative_wheel_rate) || max_relative_wheel_rate <= 0.0)
        return false;
    const double rate_error = measured_pitch_rate - pitch_reference_rate;
    desired_pitch_rate = std::clamp(
        pitch_reference_rate - 4.0 * (pitch - pitch_reference) - rate_error,
        -3.0, 3.0);
    double required_relative_rate = 0.0;
    if (!flight_symmetric_wheel_rate_for_momentum(
            mass, qdot, -desired_pitch_rate, total_h, required_relative_rate))
        return false;
    required_relative_rate = std::clamp(
        required_relative_rate, -max_relative_wheel_rate, max_relative_wheel_rate);
    wheel_linear_command = wheel_radius * required_relative_rate;
    return std::isfinite(wheel_linear_command);
}

inline double flight_ground_catch_shank_rate(const JointVector & qdot,
                                             double controller_pitch_rate)
{
    if (!qdot.allFinite() || !std::isfinite(controller_pitch_rate))
        return std::numeric_limits<double>::quiet_NaN();
    return 0.5 * (qdot[0] + qdot[1] + qdot[2] + qdot[3]) -
           controller_pitch_rate;
}

struct FastPlanarMomentumTerms
{
    double effective_inertia{0.0};
    double nonwheel_momentum{0.0};
    Eigen::Vector2d com_yz{Eigen::Vector2d::Zero()};
    Eigen::Vector2d axle_left_yz{Eigen::Vector2d::Zero()};
    Eigen::Vector2d axle_right_yz{Eigen::Vector2d::Zero()};
};

inline bool flight_theta_rate_from_fast_terms(
    const FastPlanarMomentumTerms & terms, const Eigen::Vector2d & wheel_rates,
    double total_h, double & theta_rate)
{
    theta_rate = 0.0;
    constexpr double wheel_inertia = 0.006481;
    if (!std::isfinite(terms.effective_inertia) || terms.effective_inertia <= 1e-9 ||
        !std::isfinite(terms.nonwheel_momentum) || !wheel_rates.allFinite() ||
        !std::isfinite(total_h)) return false;
    const double effective = terms.effective_inertia;
    theta_rate = (total_h - terms.nonwheel_momentum -
        wheel_inertia * wheel_rates.sum()) / effective;
    return std::isfinite(theta_rate);
}

// Direct rigid-body sum equivalent to row 0 of the CAD Schur matrix. It is
// used by the bounded takeoff envelope to avoid repeated 5x5 Schur builds at
// each 5ms integration point. Golden/random tests compare it to the matrix
// implementation before the controller relies on it.
inline bool flight_fast_planar_momentum_terms(
    const JointVector & q, const JointVector & qdot, double body_mass,
    FastPlanarMomentumTerms & terms)
{
    terms = {};
    if (!q.allFinite() || !qdot.allFinite() || !std::isfinite(body_mass) || body_mass <= 0.0)
        return false;
    struct Point { double mass, y, z, vy, vz; };
    std::array<Point, 7> points{};
    const auto rotate_yz = [](double angle, double y, double z, double & ry, double & rz) {
        const double c=std::cos(angle), s=std::sin(angle);
        ry=c*y-s*z; rz=s*y+c*z;
    };
    constexpr double hip_y=0.125, hip_z=-0.07;
    points[0]={body_mass,0.13261282,0.05396677,0.0,0.0};
    double spin_inertia = 0.159013 * body_mass / 14.0;
    double rotor_nonwheel = 0.0;
    for (int side=0; side<2; ++side) {
        const int h=2*side, k=h+1;
        const double qh=q[h], qk=q[k], vh=qdot[h], vk=qdot[k];
        double ty,tz,ky,kz,sy,sz,wy,wz;
        rotate_yz(qh,-0.13690699,-0.02116697,ty,tz);
        rotate_yz(qh,-0.29348091,-0.06220095,ky,kz);
        rotate_yz(qh+qk,0.11538205,-0.08532288,sy,sz);
        rotate_yz(qh+qk,0.28210870,-0.19553796,wy,wz);
        const double axle_y=hip_y+ky+wy, axle_z=hip_z+kz+wz;
        if (side==0) terms.axle_left_yz << axle_y,axle_z;
        else terms.axle_right_yz << axle_y,axle_z;
        const double thigh_vy=-vh*tz, thigh_vz=vh*ty;
        const double knee_y=ky+sy, knee_z=kz+sz;
        const double knee_vy=-vh*kz-(vh+vk)*sz;
        const double knee_vz=vh*ky+(vh+vk)*sy;
        const double wheel_y=ky+wy, wheel_z=kz+wz;
        const double wheel_vy=-vh*kz-(vh+vk)*wz;
        const double wheel_vz=vh*ky+(vh+vk)*wy;
        points[1+3*side]={1.20,hip_y+ty,hip_z+tz,thigh_vy,thigh_vz};
        points[2+3*side]={0.80,hip_y+knee_y,hip_z+knee_z,knee_vy,knee_vz};
        points[3+3*side]={2.00,hip_y+wheel_y,hip_z+wheel_z,wheel_vy,wheel_vz};
        spin_inertia += 0.017921 + 0.013130;
        rotor_nonwheel += 0.017921 * vh + 0.013130 * (vh + vk);
        rotor_nonwheel += 0.006481*(vh+vk);
    }
    double mass_sum=0.0;
    double com_y=0.0,com_z=0.0;
    for (const auto & p : points) { mass_sum+=p.mass; com_y+=p.mass*p.y; com_z+=p.mass*p.z; }
    if (!std::isfinite(mass_sum) || mass_sum <= 0.0) return false;
    com_y /= mass_sum; com_z /= mass_sum;
    terms.com_yz << com_y,com_z;
    constexpr double rotor_inertia=0.006481;
    terms.effective_inertia=spin_inertia+2.0*rotor_inertia;
    for (const auto & p : points) {
        const double ry=p.y-com_y, rz=p.z-com_z;
        terms.effective_inertia += p.mass*(ry*ry+rz*rz);
        terms.nonwheel_momentum += p.mass*(ry*p.vz-rz*p.vy);
    }
    terms.nonwheel_momentum += rotor_nonwheel;
    return std::isfinite(terms.effective_inertia) && terms.effective_inertia > 1e-9 &&
           std::isfinite(terms.nonwheel_momentum);
}

inline bool momentum_consistent_wheel_command(
    const FastPlanarMomentumTerms & terms, double total_h,
    double pitch, double measured_pitch_rate, double pitch_reference,
    double pitch_reference_rate, double wheel_radius,
    double max_relative_wheel_rate, double & wheel_linear_command,
    double & desired_pitch_rate)
{
    wheel_linear_command=0.0;
    desired_pitch_rate=0.0;
    if (!std::isfinite(terms.effective_inertia) || terms.effective_inertia<=1e-9 ||
        !std::isfinite(terms.nonwheel_momentum) || !std::isfinite(total_h) ||
        !std::isfinite(pitch) || !std::isfinite(measured_pitch_rate) ||
        !std::isfinite(pitch_reference) || !std::isfinite(pitch_reference_rate) ||
        !std::isfinite(wheel_radius) || wheel_radius<=0.0 ||
        !std::isfinite(max_relative_wheel_rate) || max_relative_wheel_rate<=0.0)
        return false;
    const double rate_error=measured_pitch_rate-pitch_reference_rate;
    desired_pitch_rate=std::clamp(
        pitch_reference_rate-4.0*(pitch-pitch_reference)-rate_error,-3.0,3.0);
    constexpr double wheel_inertia=0.006481;
    const double relative=(total_h-terms.effective_inertia*(-desired_pitch_rate)-
        terms.nonwheel_momentum)/(2.0*wheel_inertia);
    if (!std::isfinite(relative)) return false;
    wheel_linear_command=wheel_radius*std::clamp(
        relative,-max_relative_wheel_rate,max_relative_wheel_rate);
    return std::isfinite(wheel_linear_command);
}

inline bool certified_arrest_complete(double elapsed, double duration)
{
    return std::isfinite(elapsed) && std::isfinite(duration) && duration > 0.0 &&
           elapsed + 1e-9 >= duration;
}

inline bool certified_tuck_endpoint_matches(
    const std::array<double, 4> & q, const std::array<double, 4> & qdot,
    const std::array<double, 4> & certified_q,
    double q_tolerance = 0.005, double qdot_tolerance = 0.10)
{
    if (!std::isfinite(q_tolerance) || q_tolerance < 0.0 ||
        !std::isfinite(qdot_tolerance) || qdot_tolerance < 0.0)
        return false;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!std::isfinite(q[i]) || !std::isfinite(qdot[i]) ||
            !std::isfinite(certified_q[i]) ||
            std::abs(q[i] - certified_q[i]) > q_tolerance ||
            std::abs(qdot[i]) > qdot_tolerance)
            return false;
    }
    return true;
}

// Bounded, state-dependent nominal flight envelope.  It preserves the actual
// measured total H, integrates the required relative wheel speed through the
// existing phase slew limits, and predicts the first-contact body attitude.
// It is a feasibility filter; the runtime wheel feedback remains active and
// hard limits are still enforced by the actuator path.
inline MomentumFlightBudget plan_momentum_flight_budget(
    const JointVector & q0, const JointVector & v0,
    const Eigen::Vector2d & wheel_rates0, double total_h,
    double pitch0, double landing_pitch, double remaining,
    double com_z0, double com_vz0, double ground_z,
    double arrest_duration,
    const std::array<double, 4> & tuck, const std::array<double, 4> & land,
    double body_mass, double hip_speed_limit, double knee_speed_limit,
    double hip_acceleration_limit, double knee_acceleration_limit,
    double hip_position_limit, double knee_position_limit,
    double wheel_radius = 0.07, double forward_velocity = 0.45,
    double pitch_transition_duration = 0.28, double max_reference_rate = 0.30,
    double h_stamp = -1.0, double q_stamp = -1.0, double com_stamp = -1.0,
    double forward_stamp = -1.0)
{
    MomentumFlightBudget result;
    if (!q0.allFinite() || !v0.allFinite() || !wheel_rates0.allFinite() ||
        !std::isfinite(total_h) || !std::isfinite(pitch0) ||
        !std::isfinite(landing_pitch) || !std::isfinite(remaining) ||
        !std::isfinite(com_z0) || !std::isfinite(com_vz0) ||
        !std::isfinite(ground_z) || !std::isfinite(forward_velocity) ||
        !std::isfinite(pitch_transition_duration) || pitch_transition_duration <= 0.0 ||
        !std::isfinite(max_reference_rate) || max_reference_rate <= 0.0 ||
        !std::isfinite(arrest_duration) || arrest_duration <= 0.0 ||
        !std::isfinite(body_mass) || body_mass <= 0.0 ||
        !std::isfinite(wheel_radius) || wheel_radius <= 0.0)
    {
        result.rejection_code = 1;
        return result;
    }
    if (remaining < arrest_duration + 0.20)
    {
        result.rejection_code = 2;
        return result;
    }
    result.ground_capture_forward_velocity = forward_velocity;
    result.h_state_stamp = h_stamp;
    result.joint_state_stamp = q_stamp;
    result.com_state_stamp = com_stamp;
    result.forward_state_stamp = forward_stamp;
    const std::array<double, 4> q{q0[0], q0[1], q0[2], q0[3]};
    const std::array<double, 4> v{v0[0], v0[1], v0[2], v0[3]};

    // A small deterministic allocation family uses the measured ballistic
    // window.  Longer tuck time is selected only when the full path and H/wheel
    // prediction remain feasible; there is no fixed takeoff-state template.
    // One deterministic state-scaled pair avoids an unbounded candidate search
    // on the controller thread; a failed pair is explicitly rejected.
    constexpr std::array<double, 1> tuck_fraction{0.54};
    constexpr std::array<double, 1> extend_fraction{0.32};
    constexpr double dt = 0.005;
    constexpr double wheel_inertia = 0.006481;
    constexpr double relative_wheel_limit = 28.5714;
    constexpr double wheel_torque_limit = 50.0;
    std::array<double, 4> arrest_end{};
    const std::array<double, 4> arrest_zero{};
    for (std::size_t i = 0; i < 4; ++i)
        arrest_end[i] = q[i] + 0.5 * v[i] * arrest_duration;
    result.arrest_end_q = arrest_end;
    if (!flight_segment_admissible(q, v, arrest_zero, arrest_end, arrest_duration,
            30.0, 30.0, hip_acceleration_limit, knee_acceleration_limit,
            hip_position_limit, knee_position_limit))
    {
        result.rejection_code = 3;
        return result;
    }
    constexpr double kTouchdownReserve = 0.035;
    const double post_arrest = remaining - arrest_duration - kTouchdownReserve;
    if (post_arrest < 0.20)
    {
        result.rejection_code = 2;
        return result;
    }
    for (std::size_t candidate = 0; candidate < tuck_fraction.size(); ++candidate)
    {
        const double tuck_time = std::max(0.10, tuck_fraction[candidate] * post_arrest);
        const double extend_time = std::max(0.10, extend_fraction[candidate] * post_arrest);
        if (tuck_time + extend_time > post_arrest) continue;
        std::array<QuinticTrajectory, 4> tuck_traj, extend_traj;
        std::array<QuinticTrajectory, 4> arrest_traj;
        for (std::size_t i = 0; i < 4; ++i)
        {
            arrest_traj[i].init(0.0, arrest_duration, q[i], v[i], 0.0,
                                arrest_end[i], 0.0, 0.0);
            tuck_traj[i].init(arrest_duration, tuck_time, arrest_end[i], 0.0, 0.0,
                              tuck[i], 0.0, 0.0);
            extend_traj[i].init(arrest_duration + tuck_time, extend_time, tuck[i], 0.0, 0.0,
                                land[i], 0.0, 0.0);
        }
        bool joint_path_ok = true;
        for (std::size_t i = 0; i < 4; ++i)
        {
            const double speed_limit = (i % 2 == 0) ? hip_speed_limit : knee_speed_limit;
            const double accel_limit = (i % 2 == 0) ? hip_acceleration_limit : knee_acceleration_limit;
            const double pos_limit = (i % 2 == 0) ? hip_position_limit : knee_position_limit;
            joint_path_ok = joint_path_ok &&
                flight_trajectory_admissible(arrest_traj[i], 30.0, accel_limit, pos_limit) &&
                flight_trajectory_admissible(tuck_traj[i], speed_limit, accel_limit, pos_limit) &&
                flight_trajectory_admissible(extend_traj[i], speed_limit, accel_limit, pos_limit);
        }
        if (!joint_path_ok)
        {
            result.rejection_code = 3;
            continue;
        }
        double pitch = pitch0;
        Eigen::Vector2d wheel_rates = wheel_rates0;
        double max_rate = 0.0, max_wheel = 0.0, max_torque = 0.0;
        double max_hip_torque = 0.0, max_knee_torque = 0.0;
        double final_controller_rate = 0.0;
        double max_clearance = -std::numeric_limits<double>::infinity();
        double first_contact_time = -1.0;
        double first_contact_pitch = 0.0;
        double first_contact_rate = 0.0;
        double ground_blend = 0.0;
        double blend_at_contact = 0.0;
        FastPlanarMomentumTerms initial_terms;
        if (!flight_fast_planar_momentum_terms(q0, v0, body_mass, initial_terms)) {
            result.rejection_code=4; result.rejection_detail=13; return result;
        }
    const double initial_clearance = fast_minimum_wheel_bottom_clearance_yz(
        initial_terms.com_yz, initial_terms.axle_left_yz, initial_terms.axle_right_yz,
        pitch0, com_z0, wheel_radius, ground_z);
        double previous_clearance = initial_clearance;
        bool airborne_clearance_observed = false;
        double previous_theta_rate = 0.0;
        if (!flight_pitch_rate_for_total_momentum(
                q0, v0, wheel_rates0, total_h, body_mass, previous_theta_rate)) {
            result.rejection_code = 4;
            return result;
        }
        bool feasible = true;
        int detail = 0;
        const double simulation_horizon = remaining + 0.040;
        const int steps = static_cast<int>(std::ceil(simulation_horizon / dt));
        std::array<double, 13> torque_sample_times{};
        const std::array<double, 5> sample_fraction{0.0, 0.25, 0.50, 0.75, 1.0};
        std::size_t torque_sample_count = 0;
        for (double phase_start : {0.0, arrest_duration, arrest_duration + tuck_time}) {
            const double phase_duration = phase_start == 0.0 ? arrest_duration :
                (phase_start == arrest_duration ? tuck_time : extend_time);
            for (std::size_t fraction_index = 0; fraction_index < sample_fraction.size(); ++fraction_index) {
                if (phase_start > 0.0 && fraction_index == 0) continue;
                torque_sample_times[torque_sample_count++] =
                    phase_start + sample_fraction[fraction_index] * phase_duration;
            }
        }
        struct TorqueSample {
            JointVector q{JointVector::Zero()};
            JointVector qdot{JointVector::Zero()};
            JointVector qddot{JointVector::Zero()};
            double theta_rate{0.0};
            Eigen::Vector2d wheel_torque{Eigen::Vector2d::Zero()};
        };
        std::array<TorqueSample, 13> torque_samples{};
        std::size_t torque_sample_index = 0;
        double previous_t = 0.0;
        for (int step = 1; step <= steps; ++step)
        {
            const double t = std::min(simulation_horizon, step * dt);
            JointVector q_now, v_now, a_now;
            const std::array<QuinticTrajectory, 4> * trajectory = &extend_traj;
            if (t <= arrest_duration) trajectory = &arrest_traj;
            else if (t <= arrest_duration + tuck_time) trajectory = &tuck_traj;
            if (t <= arrest_duration + tuck_time + extend_time) {
                for (std::size_t i = 0; i < 4; ++i)
                    (*trajectory)[i].evaluate(t, q_now[i], v_now[i], a_now[i]);
            } else {
                for (std::size_t i = 0; i < 4; ++i) {
                    q_now[i] = land[i]; v_now[i] = 0.0; a_now[i] = 0.0;
                }
            }
            const double dt_local = t - previous_t;
            previous_t = t;
            const double ref_ratio = std::clamp(
                t / std::max(0.05, pitch_transition_duration), 0.0, 1.0);
            const double ref_smooth = ref_ratio * ref_ratio * (3.0 - 2.0 * ref_ratio);
            const double ref_smooth_dot = (ref_ratio > 0.0 && ref_ratio < 1.0)
                ? 6.0 * ref_ratio * (1.0 - ref_ratio) /
                    std::max(0.05, pitch_transition_duration) : 0.0;
            const double pitch_ref = pitch0 + (landing_pitch - pitch0) * ref_smooth;
            const double pitch_ref_rate = std::clamp(
                (landing_pitch - pitch0) * ref_smooth_dot,
                -max_reference_rate, max_reference_rate);
            double required_linear_command = 0.0;
            double desired_pitch_rate = 0.0;
            FastPlanarMomentumTerms cad_terms;
            if (!flight_fast_planar_momentum_terms(q_now, v_now, body_mass, cad_terms)) {
                feasible = false;
                detail = 1;
                break;
            }
            if (!momentum_consistent_wheel_command(
                    cad_terms, total_h, pitch, -previous_theta_rate,
                    pitch_ref, pitch_ref_rate, wheel_radius, relative_wheel_limit,
                    required_linear_command, desired_pitch_rate))
            {
                feasible = false;
                detail = 1;
                break;
            }
            const double linear_slew = (t <= arrest_duration) ? 0.25 : 0.18;
            const double max_step = linear_slew / wheel_radius * (dt_local / 0.005);
            const Eigen::Vector2d previous = wheel_rates;
            const double com_z_for_blend = com_z0 + com_vz0 * t - 0.5 * 9.81 * t * t;
            const double blend_clearance = fast_minimum_wheel_bottom_clearance_yz(
                cad_terms.com_yz, cad_terms.axle_left_yz, cad_terms.axle_right_yz,
                pitch, com_z_for_blend, wheel_radius, ground_z);
            if (com_vz0 - 9.81 * t < 0.0 && std::isfinite(blend_clearance)) {
                const double u = std::clamp((0.080 - blend_clearance) / 0.060, 0.0, 1.0);
                ground_blend = std::max(ground_blend, u * u * (3.0 - 2.0 * u));
            }
            // Reproduce centroidal_catch_target used by landing_capture_target
            // in FLIGHT (velocity/forward references are zero there). Wheel
            // linear command maps to relative wheel rate with +R.
            const Eigen::Vector2d axle_mean = 0.5*(cad_terms.axle_left_yz+cad_terms.axle_right_yz);
            const Eigen::Vector2d relative_com = cad_terms.com_yz - axle_mean;
            const double cp=std::cos(pitch), sp=std::sin(pitch);
            const Eigen::Vector2d rotated_relative_com(
                cp*relative_com.x()+sp*relative_com.y(),
                -sp*relative_com.x()+cp*relative_com.y());
            const double mean_shank_rate = flight_ground_catch_shank_rate(
                v_now, -previous_theta_rate);
            const double ground_linear_target = centroidal_catch_target(
                forward_velocity, rotated_relative_com.x(), rotated_relative_com.y(), mean_shank_rate,
                wheel_radius, 5.0, 0.0, 0.0);
            const double blended_linear_target = (1.0 - ground_blend) * required_linear_command +
                                                 ground_blend * ground_linear_target;
            const double rate_target = std::clamp(
                blended_linear_target / wheel_radius,
                -relative_wheel_limit, relative_wheel_limit);
            for (int side = 0; side < 2; ++side)
                wheel_rates[side] += std::clamp(
                    rate_target - wheel_rates[side], -max_step, max_step);
            if (wheel_rates.cwiseAbs().maxCoeff() > relative_wheel_limit)
            {
                feasible = false;
                detail = 2;
                break;
            }
            double theta_rate = 0.0;
            if (!flight_theta_rate_from_fast_terms(
                    cad_terms, wheel_rates, total_h, theta_rate))
            {
                feasible = false;
                detail = 4;
                break;
            }
            const double theta_acceleration = (theta_rate - previous_theta_rate) / dt_local;
            double torque = 0.0;
            Eigen::Vector2d wheel_motor_torques = Eigen::Vector2d::Zero();
            for (int side = 0; side < 2; ++side)
            {
                const int hip = side * 2;
                const int knee = hip + 1;
                const double absolute_wheel_acceleration = theta_acceleration +
                    a_now[hip] + a_now[knee] +
                    (wheel_rates[side] - previous[side]) / dt_local;
                wheel_motor_torques[side] = wheel_inertia * absolute_wheel_acceleration;
                torque = std::max(torque, std::abs(wheel_motor_torques[side]));
            }
            max_torque = std::max(max_torque, torque);
            if (torque > wheel_torque_limit) { feasible = false; detail = 3; break; }
            // Store sparse C2 extrema/boundary states for the more expensive
            // CAD inverse-dynamics budget below. The 5ms H/contact integration
            // remains dense; finite-difference Christoffel matrices do not run
            // on every controller-sized sample.
            if (torque_sample_index < torque_sample_count &&
                t + 1e-9 >= torque_sample_times[torque_sample_index]) {
                auto & sample = torque_samples[torque_sample_index++];
                sample.q = q_now;
                sample.qdot = v_now;
                sample.qddot = a_now;
                sample.theta_rate = theta_rate;
                sample.wheel_torque = wheel_motor_torques;
            }
            previous_theta_rate = theta_rate;
            pitch += -theta_rate * dt_local;
            result.max_pitch_deviation = std::max(
                result.max_pitch_deviation, std::abs(pitch - pitch0));
            final_controller_rate = -theta_rate;
            max_rate = std::max(max_rate, std::abs(theta_rate));
            max_wheel = std::max(max_wheel, wheel_rates.cwiseAbs().maxCoeff());
            const double com_z = com_z0 + com_vz0 * t - 0.5 * 9.81 * t * t;
            const double wheel_clearance = fast_minimum_wheel_bottom_clearance_yz(
                cad_terms.com_yz, cad_terms.axle_left_yz, cad_terms.axle_right_yz,
                pitch, com_z, wheel_radius, ground_z);
            max_clearance = std::max(max_clearance, wheel_clearance);
            airborne_clearance_observed = airborne_clearance_observed || wheel_clearance > 0.001;
            if (airborne_clearance_observed && previous_clearance > 0.0 && wheel_clearance <= 0.0) {
                first_contact_time = t;
                first_contact_pitch = pitch;
                first_contact_rate = final_controller_rate;
                blend_at_contact = ground_blend;
                break;  // do not integrate a fictitious post-contact free-flight state
            }
            previous_clearance = wheel_clearance;
            if (!std::isfinite(pitch) || std::abs(pitch - landing_pitch) > 0.45 ||
                std::abs(theta_rate) > 3.0)
            {
                feasible = false;
                detail = 5;
                break;
            }
        }
        if (feasible && initial_clearance >= -0.001 && airborne_clearance_observed &&
            first_contact_time > 0.0 && max_clearance >= 0.20 &&
            first_contact_pitch >= 0.0 && first_contact_pitch <= 5.0 * M_PI / 180.0 &&
            std::abs(first_contact_rate) <= 0.20 && max_rate <= 3.0)
        {
            if (torque_sample_index != torque_sample_count) {
                result.rejection_code = 4;
                result.rejection_detail = 11;
                return result;
            }
            for (std::size_t sample_index = 0; sample_index < torque_sample_count; ++sample_index) {
                const auto & sample = torque_samples[sample_index];
                const JointVector joint_ff = flight_joint_dynamics_feedforward_analytic(
                    sample.q, sample.qdot, sample.qddot, sample.theta_rate, body_mass);
                if (!joint_ff.allFinite()) {
                    result.rejection_code = 4;
                    result.rejection_detail = 11;
                    return result;
                }
                for (int side = 0; side < 2; ++side) {
                    const double hip_required = std::abs(joint_ff[2 * side]);
                    const double knee_required = std::abs(joint_ff[2 * side + 1]) +
                        std::abs(sample.wheel_torque[side]);
                    max_hip_torque = std::max(max_hip_torque, hip_required);
                    max_knee_torque = std::max(max_knee_torque, knee_required);
                    if (hip_required > 75.0 || knee_required > 60.0) {
                        result.rejection_code = 4;
                        result.rejection_detail = 12;
                        return result;
                    }
                }
            }
            result.valid = true;
            result.rejection_code = 0;
            result.rejection_detail = 0;
            result.arrest_duration = arrest_duration;
            result.tuck_duration = tuck_time;
            result.extend_duration = extend_time;
            result.final_pitch = first_contact_pitch;
            result.final_pitch_rate = first_contact_rate;
            result.max_abs_pitch_rate = max_rate;
            result.max_abs_wheel_rate = max_wheel;
            result.max_wheel_torque = max_torque;
            result.max_required_hip_torque = max_hip_torque;
            result.max_required_knee_torque = max_knee_torque;
            result.first_contact_time = first_contact_time;
            result.first_contact_pitch = first_contact_pitch;
            result.first_contact_pitch_rate = first_contact_rate;
            result.max_wheel_clearance = max_clearance;
            result.ground_handoff_blend_at_contact = blend_at_contact;
            result.tuck_target_q = tuck;
            result.landing_target_q = land;
            return result;
        }
        result.rejection_code = 4;
        result.rejection_detail = !feasible ? detail :
            (initial_clearance < -0.001 ? 6 : first_contact_time <= 0.0 ? 7 :
             max_clearance < 0.20 ? 8 :
             (first_contact_pitch < 0.0 || first_contact_pitch > 5.0*M_PI/180.0) ? 9 : 10);
    }
    return result;
}

// Positive only: a positive common hip command has negative dH/dt in the
// contact-constrained CAD sensitivity check.  The command is an addition to
// the existing torso request and is clamped again by the controller's normal
// total-body and effort budgets.
inline double thrust_ground_angular_momentum_correction(
    bool enabled, bool effort_active, bool switch_pending, bool thrust_gate_open,
    bool grounded, bool imu_fresh, const StampedAngularMomentumObserver &observer,
    double now, double target_momentum, double gain, double per_hip_limit)
{
    if (!enabled || !effort_active || switch_pending || !thrust_gate_open ||
        !grounded || !imu_fresh || !observer.valid(now) ||
        !std::isfinite(target_momentum) || target_momentum < 0.0 ||
        !std::isfinite(gain) || gain < 0.0 ||
        !std::isfinite(per_hip_limit) || per_hip_limit <= 0.0)
        return 0.0;
    return thrust_positive_angular_momentum_feedback(
        observer.momentum(), target_momentum, gain, per_hip_limit);
}

} // namespace bbot_jump
