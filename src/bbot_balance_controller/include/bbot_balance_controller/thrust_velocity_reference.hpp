#pragma once

#include <algorithm>
#include "bbot_balance_controller/centroidal_state.hpp"

namespace bbot_jump {

struct ThrustVelocityReference {
    bool valid=false;
    bool speed_limited=false;
    double com_velocity=0.0;
    double knee_jacobian=0.0;
    double knee_velocity_raw=0.0;
    double knee_velocity=0.0;
};

// While both wheels support the robot, their axle height is fixed:
//   z_COM = R + world_z.dot(COM(q)-axle(q)).
// Thus v_COM = J_HL*qdot_HL + J_HR*qdot_HR + J_K*qdot_K
//              - (COM_x-axle_x)*pitch_rate, with a common knee velocity.
// Invert this relation for the SAME vertical velocity demanded by the force
// loop. Use commanded hip/body rates, never measured knee rates: D remains
// feedback against a reference, not a feedthrough that cancels its damping.
// The geometry is mass weighted and includes both legs and wheels.
inline ThrustVelocityReference thrust_velocity_reference(
    const std::array<double,4> & q, double pitch, double body_mass,
    double desired_com_velocity, double hip_velocity_left,
    double hip_velocity_right, double desired_pitch_rate, double speed_limit)
{
    ThrustVelocityReference out;
    if (!std::isfinite(pitch) || !std::isfinite(body_mass) || body_mass<=0 ||
        !std::isfinite(desired_com_velocity) || desired_com_velocity<0 ||
        !std::isfinite(hip_velocity_left) || !std::isfinite(hip_velocity_right) ||
        !std::isfinite(desired_pitch_rate) || !std::isfinite(speed_limit) || speed_limit<=0)
        return out;
    for (double joint:q) if (!std::isfinite(joint)) return out;
    const auto geometry=centroidal_geometry(q,body_mass);
    const Eigen::Vector3d vertical(0.0,-std::sin(pitch),std::cos(pitch));
    const Eigen::RowVector4d j=vertical.transpose()*
        (geometry.com_jacobian-geometry.axle_jacobian);
    const auto r=rotate_about_hip(-pitch,geometry.com-geometry.axle);
    out.knee_jacobian=j[1]+j[3];
    // The supported extension branch has a negative knee Jacobian. Do not
    // invert a singular or reversed branch, or use a fallen geometry.
    if (r.z()<0.10 || out.knee_jacobian>=-1e-4) return out;
    out.com_velocity=desired_com_velocity;
    const double other_velocity=j[0]*hip_velocity_left+j[2]*hip_velocity_right-
                                r.y()*desired_pitch_rate;
    out.knee_velocity_raw=(desired_com_velocity-other_velocity)/out.knee_jacobian;
    out.knee_velocity=std::clamp(out.knee_velocity_raw,-speed_limit,0.0);
    out.speed_limited=std::abs(out.knee_velocity-out.knee_velocity_raw)>1e-9;
    out.valid=true;
    return out;
}

// Share the existing protection envelope with the nominal reference. The
// travel factor is latched monotonically in the controller; zero means no
// extension even if the target COM speed has not been reached.
inline double protected_thrust_knee_velocity(double nominal, double travel, double brake) {
    return nominal*std::clamp(travel,0.0,1.0)*
        (1.0-0.75*std::clamp(brake,0.0,1.0));
}
} // namespace bbot_jump
