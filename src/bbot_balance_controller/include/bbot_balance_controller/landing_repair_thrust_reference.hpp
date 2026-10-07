#pragma once
#include "bbot_balance_controller/landing_repair_momentum.hpp"
#include "bbot_balance_controller/centroidal_state.hpp"

namespace bbot_jump {
struct MomentumThrustScope {
    bool enabled=false, thrust=false, gate_open=false, blocked=true;
    bool effort_active=false, switch_pending=true;
    bool contact_valid=false, contact_continuous=false, bilateral=false;
    bool momentum_valid=false;
    double now=0.0, contact_stamp=0.0, momentum_stamp=0.0, motion_time=0.0;
};
inline double momentum_thrust_blend(const MomentumThrustScope & s) {
    if (!s.enabled || !s.thrust || !s.gate_open || s.blocked ||
        !s.effort_active || s.switch_pending || !s.contact_valid ||
        !s.contact_continuous || !s.bilateral || !s.momentum_valid ||
        !std::isfinite(s.now) || !std::isfinite(s.contact_stamp) ||
        !std::isfinite(s.momentum_stamp) || !std::isfinite(s.motion_time) ||
        s.contact_stamp<=0 || s.momentum_stamp<=0 ||
        s.now<s.contact_stamp || s.now-s.contact_stamp>.010 ||
        s.now<s.momentum_stamp || s.now-s.momentum_stamp>.020 ||
        s.motion_time<=.080 || s.motion_time>=.180) return 0.0;
    const auto smooth=[](double x) {
        x=std::clamp(x,0.0,1.0);return x*x*(3.0-2.0*x);
    };
    return smooth((s.motion_time-.080)/.020)*smooth((.180-s.motion_time)/.020);
}

struct MomentumThrustReference {
    bool valid = false;
    double hip_velocity = 0.0;
    double knee_velocity = 0.0;
    double predicted_com_velocity = 0.0;
    double predicted_momentum = 0.0;
    double normalized_determinant = 0.0;
};

// Experimental supported-wheel kinematic reference, not inverse dynamics.
// Both axle heights must remain fixed; absolute wheel spin is prescribed.
// Changing hip/knee velocity therefore changes relative wheel spin too.
// It cannot be used in free flight, with a lifted wheel, or as a contact wrench.
inline MomentumThrustReference momentum_thrust_reference(
    const JointVector & q, double pitch, double desired_pitch_rate,
    const std::array<double,2> & absolute_wheel_rates, double body_mass,
    double desired_com_velocity, double desired_momentum,
    double hip_speed_limit = 11.0, double knee_speed_limit = 15.0)
{
    MomentumThrustReference out;
    if (!q.allFinite() || !std::isfinite(pitch) ||
        !std::isfinite(desired_pitch_rate) || !std::isfinite(body_mass) || body_mass <= 0 ||
        !std::isfinite(desired_com_velocity) || desired_com_velocity < 0 ||
        !std::isfinite(desired_momentum) ||
        !std::isfinite(absolute_wheel_rates[0]) || !std::isfinite(absolute_wheel_rates[1]) ||
        !std::isfinite(hip_speed_limit) || hip_speed_limit <= 0 ||
        !std::isfinite(knee_speed_limit) || knee_speed_limit <= 0) return out;
    for (int i=0;i<4;i++)
        if (std::abs(q[i]) > (i%2==0 ? 1.52 : 1.5708)) return out;
    const auto geom=centroidal_geometry({q[0],q[1],q[2],q[3]},body_mass);
    const auto r=rotate_about_hip(-pitch,geom.com-geom.axle);
    const Eigen::Vector3d vertical(0,-std::sin(pitch),std::cos(pitch));
    const Eigen::RowVector4d j=vertical.transpose()*(geom.com_jacobian-geom.axle_jacobian);
    const auto m=flight_pitch_joint_mass_matrix(q,body_mass);
    const double jh=j[0]+j[2], jk=j[1]+j[3];
    const double hh=m(0,1)+m(0,3), hk=m(0,2)+m(0,4);
    if (r.z()<.10 || jk>=-1e-4 || !m.allFinite()) return out;
    const double determinant=jh*hk-jk*hh;
    const double norm=std::hypot(jh,jk)*std::hypot(hh,hk);
    if (norm<1e-9) return out;
    out.normalized_determinant=std::abs(determinant)/norm;
    if (out.normalized_determinant<.05) return out;
    const double vz_rhs=desired_com_velocity+r.y()*desired_pitch_rate;
    constexpr double iw=.006481;
    const double h_rhs=desired_momentum+m(0,0)*desired_pitch_rate-
        iw*(absolute_wheel_rates[0]+absolute_wheel_rates[1]);
    const double hvel=(vz_rhs*hk-jk*h_rhs)/determinant;
    const double kvel=(jh*h_rhs-vz_rhs*hh)/determinant;
    // Clipping either result would violate the two equalities. Reject it.
    if (!std::isfinite(hvel) || !std::isfinite(kvel) ||
        std::abs(hvel)>hip_speed_limit || kvel>0 || kvel < -knee_speed_limit) return out;
    // The existing travel guard uses actual velocities. Also check the new
    // reference over its 30ms look-ahead so a legal endpoint cannot request a
    // near-immediate hit of a hard stop.
    for (int i=0;i<4;i++)
        if (std::abs(q[i]+.030*(i%2==0 ? hvel : kvel)) >
            (i%2==0 ? 1.52 : 1.56)) return out;
    out.hip_velocity=hvel;out.knee_velocity=kvel;
    out.predicted_com_velocity=jh*hvel+jk*kvel-r.y()*desired_pitch_rate;
    out.predicted_momentum=-m(0,0)*desired_pitch_rate+hh*hvel+hk*kvel+
        iw*(absolute_wheel_rates[0]+absolute_wheel_rates[1]);
    out.valid=true;
    return out;
}
} // namespace bbot_jump
