#pragma once

#include <cmath>
#include "bbot_balance_controller/control_timing.hpp"

namespace bbot_jump {

struct LandingForwardAxis {
    double x{0.0};
    double y{0.0};
    bool valid{false};
};

// A body heading defines forward even at rest or during backward wheel roll.
// The caller certifies the pose source before freezing this axis for a jump.
inline LandingForwardAxis landing_forward_axis_from_heading(
    double heading_x, double heading_y, bool source_valid) {
    const double norm = std::hypot(heading_x, heading_y);
    if (!source_valid || !std::isfinite(heading_x) ||
        !std::isfinite(heading_y) || !std::isfinite(norm) || norm <= 0.5)
        return {};
    return {heading_x / norm, heading_y / norm, true};
}

inline LandingForwardAxis freeze_landing_forward_axis(
    double heading_x, double heading_y, bool pose_valid,
    double pose_stamp, double now_stamp) {
    const bool fresh_pose = pose_valid && now_stamp >= pose_stamp &&
        sensor_stamps_within(now_stamp, pose_stamp, 0.080);
    return landing_forward_axis_from_heading(heading_x, heading_y, fresh_pose);
}

} // namespace bbot_jump
