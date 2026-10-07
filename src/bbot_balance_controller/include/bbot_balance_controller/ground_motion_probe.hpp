#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace bbot_jump {

// Stage-two probe near the measured Effort standing anchor. The first 0.5 s
// ramps velocity with zero acceleration at both ends, the next 0.5 s cruises,
// and the final 0.5 s ramps velocity to zero. The integrated displacement is
// exactly (+/- 0.01 hip, -/+ 0.02 knee) radians.
constexpr double kGroundMotionAccelerationSeconds = 0.5;
constexpr double kGroundMotionCruiseSeconds = 0.5;
constexpr double kGroundMotionDecelerationSeconds = 0.5;
constexpr double kGroundMotionHipPeakRate = 0.01;
constexpr double kGroundMotionKneePeakRate = 0.02;
constexpr double kGroundMotionDuration = 1.5;

struct GroundMotionReference {
  bool valid{false};
  std::array<double, 4> q{};  // left hip, left knee, right hip, right knee
  std::array<double, 4> v{};
  std::array<double, 4> a{};
  double displacement{0.0};
  double velocity_scale{0.0};
  double acceleration_scale{0.0};
  const char *phase{"invalid"};
};

inline GroundMotionReference ground_motion_probe_reference(
    const std::array<double, 4> &anchor, int direction, double elapsed)
{
  GroundMotionReference out;
  if ((direction != -1 && direction != 1) || !std::isfinite(elapsed) || elapsed < 0.0)
    return out;
  for (double q : anchor)
    if (!std::isfinite(q)) return out;

  const double t = std::clamp(elapsed, 0.0, kGroundMotionDuration);
  const double ramp = kGroundMotionAccelerationSeconds;
  const double cruiseEnd = ramp + kGroundMotionCruiseSeconds;
  double distance = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
  if (t <= ramp) {
    const double u = t / ramp;
    const double s = 3.0 * u * u - 2.0 * u * u * u;
    distance = ramp * (u * u * u - 0.5 * u * u * u * u);
    speed = s;
    acceleration = 6.0 * u * (1.0 - u) / ramp;
    out.phase = "acceleration";
  } else if (t <= cruiseEnd) {
    distance = 0.5 * ramp + (t - ramp);
    speed = 1.0;
    out.phase = "cruise";
  } else {
    const double u = (t - cruiseEnd) / kGroundMotionDecelerationSeconds;
    const double s = 3.0 * u * u - 2.0 * u * u * u;
    distance = 0.5 * ramp + kGroundMotionCruiseSeconds +
        kGroundMotionDecelerationSeconds * (u - u * u * u + 0.5 * u * u * u * u);
    speed = 1.0 - s;
    acceleration = -6.0 * u * (1.0 - u) /
        kGroundMotionDecelerationSeconds;
    out.phase = "normal_stop";
  }
  distance = std::clamp(distance, 0.0,
      kGroundMotionAccelerationSeconds + kGroundMotionCruiseSeconds);
  out.displacement = static_cast<double>(direction) * distance;
  out.velocity_scale = speed;
  out.acceleration_scale = acceleration;
  const std::array<double, 4> peakRate{{
      kGroundMotionHipPeakRate, kGroundMotionKneePeakRate,
      kGroundMotionHipPeakRate, kGroundMotionKneePeakRate}};
  const std::array<double, 4> sign{{
      1.0, -1.0, 1.0, -1.0}};
  for (std::size_t i = 0; i < out.q.size(); ++i) {
    out.q[i] = anchor[i] + static_cast<double>(direction) * sign[i] *
        peakRate[i] * distance;
    out.v[i] = static_cast<double>(direction) * sign[i] * peakRate[i] * speed;
    out.a[i] = static_cast<double>(direction) * sign[i] * peakRate[i] * acceleration;
  }
  out.valid = true;
  return out;
}

struct GroundMotionGuardInput {
  bool effort_active{false};
  bool switch_pending{false};
  bool fresh_state{false};
  bool bilateral_contact{false};
  std::array<double, 4> anchor{};
  std::array<double, 4> q{};
  std::array<double, 4> v{};
  double pitch_delta{std::numeric_limits<double>::quiet_NaN()};
  double pitch_rate{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 4> joint_min{{-1.52, -1.56, -1.52, -1.56}};
  std::array<double, 4> joint_max{{1.52, 1.56, 1.52, 1.56}};
};

inline bool ground_motion_probe_guard(const GroundMotionGuardInput &in,
                                      std::string &reason)
{
  if (!in.effort_active || in.switch_pending) {
    reason = "effort_mode_unavailable";
    return false;
  }
  if (!in.fresh_state || !in.bilateral_contact) {
    reason = "stale_state_or_bilateral_contact_lost";
    return false;
  }
  if (!std::isfinite(in.pitch_delta) || !std::isfinite(in.pitch_rate) ||
      std::abs(in.pitch_delta) > 0.10 || std::abs(in.pitch_rate) > 0.5) {
    reason = "attitude_guard";
    return false;
  }
  for (std::size_t i = 0; i < in.q.size(); ++i) {
    if (!std::isfinite(in.anchor[i]) || !std::isfinite(in.q[i]) ||
        !std::isfinite(in.v[i]) || !std::isfinite(in.joint_min[i]) ||
        !std::isfinite(in.joint_max[i])) {
      reason = "nonfinite_joint_state";
      return false;
    }
    if (std::abs(in.q[i] - in.anchor[i]) > 0.08 ||
        std::abs(in.v[i]) > 0.10) {
      reason = "joint_anchor_or_rate_guard";
      return false;
    }
    if (in.q[i] - in.joint_min[i] < 0.30 ||
        in.joint_max[i] - in.q[i] < 0.30) {
      reason = "joint_soft_limit_guard";
      return false;
    }
  }
  reason = "ok";
  return true;
}

struct GroundMotionEffortInput {
  std::array<double, 4> support{};  // gravity + Jz^T Fz + feedback, gravity once
  double torso_per_hip{0.0};
  double hip_limit{75.0};
  double knee_limit{60.0};
};

struct GroundMotionEffort {
  bool valid{false};
  std::array<double, 4> torque{};
};

// Preserve full common-mode hip support and add the dynamic torso correction
// on top. This intentionally bypasses allocate_torso_hips, whose legacy
// behavior replaces the common leg-support torque with the torso request.
inline GroundMotionEffort ground_motion_direct_full_support(
    const GroundMotionEffortInput &in)
{
  GroundMotionEffort out;
  if (!std::isfinite(in.torso_per_hip) || !std::isfinite(in.hip_limit) ||
      !std::isfinite(in.knee_limit) || in.hip_limit <= 0.0 ||
      in.knee_limit <= 0.0 || in.hip_limit > 75.0 || in.knee_limit > 60.0)
    return out;
  for (double tau : in.support)
    if (!std::isfinite(tau)) return out;
  out.torque = in.support;
  out.torque[0] += in.torso_per_hip;
  out.torque[2] += in.torso_per_hip;
  out.torque[0] = std::clamp(out.torque[0], -in.hip_limit, in.hip_limit);
  out.torque[2] = std::clamp(out.torque[2], -in.hip_limit, in.hip_limit);
  out.torque[1] = std::clamp(out.torque[1], -in.knee_limit, in.knee_limit);
  out.torque[3] = std::clamp(out.torque[3], -in.knee_limit, in.knee_limit);
  out.valid = true;
  return out;
}

}  // namespace bbot_jump
