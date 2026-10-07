#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "bbot_balance_controller/flight_landing_geometry.hpp"

namespace bbot_jump
{

struct GroundLaunchPose
{
    bool valid{false};
    std::array<double, 2> q{};
    double blend{0.0};
    double target_x{0.0};
    // Diagnostic COM position evaluated at the requested full body pitch.
    double com_forward{0.0};
};

inline double ground_launch_blend(double progress)
{
    if (!std::isfinite(progress))
        return std::numeric_limits<double>::quiet_NaN();
    const double u = std::clamp(progress, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    return std::clamp(10.0 * u3 - 15.0 * u3 * u + 6.0 * u3 * u2, 0.0, 1.0);
}

inline GroundLaunchPose ground_launch_pose(
    const bbot_kinematics::RobotParams &params, double height,
    double body_pitch, double desired_com_forward, double body_mass,
    double blend)
{
    GroundLaunchPose out;
    out.blend = blend;
    if (!std::isfinite(height) || !std::isfinite(body_pitch) ||
        !std::isfinite(desired_com_forward) || !std::isfinite(body_mass) ||
        body_mass <= 0.0 || !std::isfinite(blend) || blend < 0.0 || blend > 1.0)
        return out;

    const auto endpoint = solve_leg_pose_for_com_forward(
        params, height, body_pitch, desired_com_forward, body_mass);
    if (!endpoint.valid)
        return out;

    out.target_x = blend * endpoint.target_x;
    if (!std::isfinite(out.target_x) ||
        !landing_ik_reachable(params, height, blend * body_pitch, out.target_x))
        return out;

    out.q = landing_ik_target(params, height, blend * body_pitch, out.target_x);
    if (!std::isfinite(out.q[0]) || !std::isfinite(out.q[1]) ||
        std::abs(out.q[0]) > 1.52 || std::abs(out.q[1]) > 1.5708)
        return out;

    constexpr double hip_body_vertical_offset = 0.07;
    constexpr double cad_wheel_to_hip_x = -0.01137221;
    const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
    const double phi2_0 = std::atan2(0.28210870, 0.19553796);
    const double pitch_at_pose = blend * body_pitch;
    const double phi1 = phi1_0 + out.q[0] - pitch_at_pose;
    const double phi2 = phi2_0 + out.q[0] + out.q[1] - pitch_at_pose;
    const double shank_angle = std::atan2(0.28210870, 0.19553796) +
                               out.q[0] + out.q[1] - pitch_at_pose;
    const double knee_clearance = params.l1 * std::cos(shank_angle);
    const double fk_height = params.l2 * std::cos(phi1) +
                             params.l1 * std::cos(phi2) +
                             hip_body_vertical_offset + params.wheel_radius;
    const double fk_dx = params.l2 * std::sin(phi1) + params.l1 * std::sin(phi2);
    const double expected_dx = cad_wheel_to_hip_x - out.target_x;
    if (!std::isfinite(shank_angle) || !std::isfinite(knee_clearance) ||
        knee_clearance < 0.20 || !std::isfinite(fk_height) || !std::isfinite(fk_dx) ||
        std::abs(fk_height - height) > 1e-6 ||
        std::abs(fk_dx - expected_dx) > 1e-6)
        return out;

    out.com_forward = landing_com_forward_from_axle(
        {out.q[0], out.q[1], out.q[0], out.q[1]}, body_pitch, body_mass);
    if (!std::isfinite(out.com_forward))
        return out;
    out.valid = true;
    return out;
}

}  // namespace bbot_jump
