#pragma once

#include <algorithm>
#include <cmath>

namespace bbot_jump {

struct ThrustBrakeAllocation {
    bool valid = false;
    double brake_target = 0.0;
    double delta = 0.0;
    double command = 0.0;
};

// Blend the existing net knee command toward a bounded dissipative target.
// This helper only reallocates the final command: it does not alter the
// propulsive-force budget or the hardware torque limit.
inline ThrustBrakeAllocation thrust_brake_allocation(
    double net_propulsive_command, double joint_velocity, double blend)
{
    ThrustBrakeAllocation out;
    if (!std::isfinite(net_propulsive_command) ||
        !std::isfinite(joint_velocity) || !std::isfinite(blend) ||
        blend < 0.0 || blend > 1.0)
        return out;

    out.brake_target = std::clamp(-1.6 * joint_velocity, -22.0, 22.0);
    out.command = (1.0 - blend) * net_propulsive_command +
                  blend * out.brake_target;
    out.delta = out.command - net_propulsive_command;
    out.valid = std::isfinite(out.brake_target) &&
                std::isfinite(out.command) && std::isfinite(out.delta);
    if (!out.valid) {
        out = {};
    }
    return out;
}

}  // namespace bbot_jump
