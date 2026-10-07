#pragma once
#include <algorithm>
#include <cmath>

namespace bbot_jump {
// Adapted phase policy from local wheeled-bipedal-jumping/motion.py::jump.
// BBOT keeps its own CAD force-to-torque allocation, contact observer and limits.
// No optimized Webots coefficients or body-velocity threshold are transplanted.
class ReferenceJumpPulse {
 public:
  void reset() { released_ = false; release_time_ = 0.0; release_extra_ = 0.0; }
  double extra_force(double elapsed, bool fresh, bool permitted, double vz,
                     double target_vz, double support_per_leg) {
    if (!std::isfinite(elapsed) || elapsed < 0.0 ||
        !std::isfinite(support_per_leg) || support_per_leg <= 0.0 ||
        !std::isfinite(target_vz) || target_vz <= 0.0) return 0.0;
    const double u = std::clamp(elapsed / 0.090, 0.0, 1.0);
    const double extra = 3.0 * support_per_leg * u * u * (3.0 - 2.0 * u);
    // Latch only fresh COM speed; timeout terminates an unsuccessful pulse too.
    if (!released_ && ((fresh && permitted && std::isfinite(vz) && vz >= target_vz)
                       || elapsed >= 0.320)) {
      released_ = true; release_time_ = elapsed; release_extra_ = extra;
    }
    if (!fresh || !permitted || !std::isfinite(vz)) return 0.0;
    if (released_) {
      const double r = std::clamp((elapsed - release_time_) / 0.040, 0.0, 1.0);
      return release_extra_ * (1.0 - r * r * (3.0 - 2.0 * r));
    }
    return extra;
  }
  bool released() const { return released_; }
 private:
  bool released_ = false;
  double release_time_ = 0.0;
  double release_extra_ = 0.0;
};

// Predict travel during source age + unloading response. Joint zero offsets
// are irrelevant here: both ends of the unchanged physical range are protected.
inline double reference_travel_scale(double q, double v, double sample_age) {
  if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(sample_age) ||
      sample_age < 0.0 || sample_age > 0.080) return 0.0;
  if (std::abs(v) < 1.0) return std::abs(q) < 1.50 ? 1.0 : 0.0;
  const double remaining = v > 0.0 ? 1.50 - q : q + 1.50;
  const double preview = 0.015 + sample_age;
  // Removing push torque is not instantaneous braking. Reserve stopping travel
  // under a conservative 200 rad/s^2 deceleration, in addition to sample delay.
  // This may lower the attainable impulse; it must not borrow hard-stop impact.
  const double stopping_travel = v * v / (2.0 * 200.0);
  return std::clamp((remaining - preview * std::abs(v) - stopping_travel) / 0.120, 0.0, 1.0);
}

// Apex is an event, but deployment always wins when the landing budget is short.
inline bool reference_deploy_due(bool tuck_done, bool fresh_com, double com_vz,
                                 double remaining, double extend, double margin) {
  if (!std::isfinite(remaining) || !std::isfinite(extend) ||
      !std::isfinite(margin) || extend <= 0.0 || margin < 0.0) return true;
  return remaining <= extend + margin + 0.020 ||
         (tuck_done && fresh_com && std::isfinite(com_vz) && com_vz <= 0.05);
}
} // namespace bbot_jump
