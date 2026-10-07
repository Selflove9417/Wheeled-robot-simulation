#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_kinematics/robot_params.hpp"

namespace bbot_jump
{

struct LandingPoseTarget
{
    bool valid = false;
    double target_x = 0.0;
    double com_forward = 0.0;
    double hip = 0.0;
    double knee = 0.0;
    double shank_angle = 0.0;
    double fk_height = 0.0;
    double fk_horizontal_offset = 0.0;
};

inline bool landing_ik_reachable(const bbot_kinematics::RobotParams &params,
                                 double target_height, double body_pitch,
                                 double target_x)
{
    constexpr double hip_body_vertical_offset = 0.07;
    constexpr double cad_wheel_to_hip_x = -0.01137221;
    if (!std::isfinite(target_height) || !std::isfinite(body_pitch) ||
        !std::isfinite(target_x) || !std::isfinite(params.l1) ||
        !std::isfinite(params.l2) || !std::isfinite(params.wheel_radius) ||
        params.l1 <= 0.0 || params.l2 <= 0.0 || params.wheel_radius <= 0.0 ||
        target_x < -0.12 || target_x > 0.14)
        return false;
    const double dz = target_height - (hip_body_vertical_offset + params.wheel_radius);
    const double dx = cad_wheel_to_hip_x - target_x;
    const double d = std::hypot(dx, dz);
    const double min_reach = std::abs(params.l1 - params.l2);
    const double max_reach = params.l1 + params.l2;
    return dz >= 0.10 && dz <= 0.60 &&
           d > min_reach + 1e-6 && d < max_reach - 1e-6;
}

inline std::array<double, 2> landing_ik_target(
    const bbot_kinematics::RobotParams &params, double target_height,
    double body_pitch, double target_x)
{
    constexpr double hip_body_vertical_offset = 0.07;
    constexpr double cad_wheel_to_hip_x = -0.01137221;
    const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
    const double phi2_0 = std::atan2(0.28210870, 0.19553796);
    const double dz = target_height - (hip_body_vertical_offset + params.wheel_radius);
    const double dx = cad_wheel_to_hip_x - target_x;
    const double d2 = dx * dx + dz * dz;
    const double d = std::max(1e-6, std::sqrt(d2));
    const double cos_gamma =
        (params.l2 * params.l2 + params.l1 * params.l1 - d2) /
        (2.0 * params.l2 * params.l1);
    const double gamma = std::acos(cos_gamma);
    const double theta_d = std::atan2(dx, dz);
    const double cos_psi =
        (params.l2 * params.l2 + d2 - params.l1 * params.l1) /
        (2.0 * params.l2 * d);
    const double psi = std::acos(cos_psi);
    const double phi1 = theta_d - psi;
    const double phi2 = phi1 + (M_PI - gamma);
    return {phi1 - phi1_0 + body_pitch,
            (phi2 - phi1) - (phi2_0 - phi1_0)};
}

inline double landing_com_forward_from_axle(
    const std::array<double, 4> &q, double body_pitch, double body_mass)
{
    const auto geometry = centroidal_geometry(q, body_mass);
    const auto relative = rotate_about_hip(-body_pitch, geometry.com - geometry.axle);
    return relative.y();
}

inline LandingPoseTarget solve_leg_pose_for_com_forward(
    const bbot_kinematics::RobotParams &params, double target_height,
    double body_pitch, double desired_com_forward, double body_mass,
    double min_target_x = -0.12, double max_target_x = 0.14)
{
    LandingPoseTarget out;
    if (!std::isfinite(target_height) || !std::isfinite(body_pitch) ||
        !std::isfinite(desired_com_forward) || !std::isfinite(body_mass) ||
        body_mass <= 0.0 || !std::isfinite(min_target_x) ||
        !std::isfinite(max_target_x) || min_target_x >= max_target_x)
        return out;
    const auto evaluate = [&](double x) {
        if (!landing_ik_reachable(params, target_height, body_pitch, x))
            return std::numeric_limits<double>::quiet_NaN();
        const auto ik = landing_ik_target(params, target_height, body_pitch, x);
        return landing_com_forward_from_axle(
            {ik[0], ik[1], ik[0], ik[1]}, body_pitch, body_mass);
    };
    double lo_value = evaluate(min_target_x);
    double hi_value = evaluate(max_target_x);
    if (!std::isfinite(lo_value) || !std::isfinite(hi_value) ||
        desired_com_forward < std::min(lo_value, hi_value) ||
        desired_com_forward > std::max(lo_value, hi_value))
        return out;
    const bool increasing = hi_value >= lo_value;
    double lo = min_target_x;
    double hi = max_target_x;
    for (int i = 0; i < 48; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double value = evaluate(mid);
        if (!std::isfinite(value)) return out;
        if ((value < desired_com_forward) == increasing) lo = mid;
        else hi = mid;
    }
    out.target_x = 0.5 * (lo + hi);
    const auto ik = landing_ik_target(params, target_height, body_pitch, out.target_x);
    out.hip = ik[0];
    out.knee = ik[1];
    out.shank_angle = std::atan2(0.28210870, 0.19553796) + out.hip + out.knee - body_pitch;
    out.com_forward = evaluate(out.target_x);
    const double phi1_0 = std::atan2(-0.29348091, 0.06220095);
    const double phi2_0 = std::atan2(0.28210870, 0.19553796);
    const double phi1 = phi1_0 + out.hip - body_pitch;
    const double phi2 = phi2_0 + out.hip + out.knee - body_pitch;
    const double fk_height = params.l2 * std::cos(phi1) +
                             params.l1 * std::cos(phi2) +
                             0.07 + params.wheel_radius;
    const double fk_dx = params.l2 * std::sin(phi1) +
                         params.l1 * std::sin(phi2);
    const double expected_dx = -0.01137221 - out.target_x;
    out.fk_height = fk_height;
    out.fk_horizontal_offset = fk_dx;
    out.valid = std::isfinite(out.hip) && std::isfinite(out.knee) &&
                std::isfinite(out.shank_angle) && std::isfinite(out.com_forward) &&
                std::isfinite(fk_height) && std::isfinite(fk_dx) &&
                std::abs(fk_height - target_height) <= 1e-6 &&
                std::abs(fk_dx - expected_dx) <= 1e-6 &&
                std::abs(out.hip) <= 1.57 && std::abs(out.knee) <= 1.57;
    return out;
}

// Landing keeps its stricter shank cone; general support/launch IK only
// requires a reachable CAD pose, finite exact FK, and valid joint positions.
inline LandingPoseTarget solve_landing_pose_for_com_forward(
    const bbot_kinematics::RobotParams &params, double target_height,
    double body_pitch, double desired_com_forward, double body_mass,
    double min_target_x = -0.12, double max_target_x = 0.14)
{
    auto out = solve_leg_pose_for_com_forward(
        params, target_height, body_pitch, desired_com_forward, body_mass,
        min_target_x, max_target_x);
    const double knee_clearance = params.l1 * std::cos(out.shank_angle);
    out.valid = out.valid && std::isfinite(knee_clearance) &&
                std::abs(out.shank_angle) <= 0.75 && knee_clearance >= 0.20;
    return out;
}

}  // namespace bbot_jump
