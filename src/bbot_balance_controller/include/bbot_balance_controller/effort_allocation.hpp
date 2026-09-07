#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace bbot_jump {

struct JointEffortLimits { double hip; double knee; };

// Temporary Gazebo experiment, bounded by the existing URDF joint effort=150.
// Flight, landing and non-simulation operation retain the original limits.
inline JointEffortLimits jump_effort_limits(bool thrust, bool use_sim_time, bool relax) {
    return thrust && use_sim_time && relax ? JointEffortLimits{150.0,150.0} :
                                            JointEffortLimits{75.0,60.0};
}

// Largest nonnegative F satisfying |jacobian*F + other_torque| <= limit.
// The separately bounded PD/attitude torque must already be feasible at F=0.
// Opposing torque creates room: with J=-.25, PD=+22, limit=57, the budget is
// (57+22)/.25=316 N, not (57-22)/.25=140 N (which delivers only -13 Nm).
inline double signed_force_limit(double jacobian, double other_torque, double limit) {
    if (!std::isfinite(jacobian) || !std::isfinite(other_torque) ||
        !std::isfinite(limit) || limit<=0.0 || std::abs(other_torque)>limit) return 0.0;
    if (std::abs(jacobian)<1e-8) return std::numeric_limits<double>::infinity();
    return std::max(0.0,(limit-std::copysign(1.0,jacobian)*other_torque)/std::abs(jacobian));
}
} // namespace bbot_jump
