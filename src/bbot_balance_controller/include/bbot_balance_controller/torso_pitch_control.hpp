#pragma once

#include <algorithm>
#include <cmath>
#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace bbot_jump {

// Independent, timestamped 10 ms filter for the torso loop. Do not change the
// established gyro filtering used by airborne/wheel control. At 100 Hz the old
// alpha=.15 gyro lags a contact-induced direction reversal by several samples.
class TorsoImuObserver {
    double stamp_=-1.0, period_=0.010;
    double rate_=0.0, fy_=0.0, fz_=9.81;
public:
    void update(double stamp, double rate, double fy, double fz) {
        if (!std::isfinite(stamp) || stamp<=0.0 || !std::isfinite(rate) ||
            !std::isfinite(fy) || !std::isfinite(fz)) return;
        if (stamp==stamp_) return;
        const double dt=stamp-stamp_;
        if (stamp_<0.0 || dt<=0.0 || dt>0.080) {
            rate_=rate; fy_=fy; fz_=fz; period_=0.010;
        } else {
            const double alpha=1.0-std::exp(-dt/0.010);
            rate_+=alpha*(rate-rate_); fy_+=alpha*(fy-fy_); fz_+=alpha*(fz-fz_);
            period_+=0.2*(std::clamp(dt,0.005,0.030)-period_);
        }
        stamp_=stamp;
    }
    bool fresh(double now) const {
        return std::isfinite(now) && stamp_>=0.0 && now>=stamp_ && now-stamp_<=0.080;
    }
    double stamp() const { return stamp_; }
    double period() const { return period_; }
    double rate() const { return rate_; }
    double fy() const { return fy_; }
    double fz() const { return fz_; }
};

struct TorsoPitchTorque {
    double force_feedforward=0.0; // per hip
    double feedback=0.0;         // per hip
    double command=0.0;          // bounded mean of the two hip torques
};

// The IMU sits at the box COM (<0.04 mm offset in current URDF), with axes
// aligned to base_link. f is measured specific force, not world acceleration.
// Box moment balance about its COM, pitch=-roll:
//   I_COM*pitch_ddot = tau_HL+tau_HR - m*(r_z*f_y-r_y*f_z).
// r is hip->box COM: (Y=.00761282, Z=.12396677). This accounts for both gravity
// and translational contact forces without adding a second J^T F hip command.
// The requested box feedback is discretized once using box inertia, not the
// free-leg Schur inertia, and clipped only after that solve.
inline TorsoPitchTorque torso_pitch_torque(
    double pitch, double reference, double rate, double fy, double fz,
    double body_mass, double kp, double kd, double horizon, double per_hip_limit)
{
    TorsoPitchTorque out;
    if (!std::isfinite(pitch) || !std::isfinite(reference) || !std::isfinite(rate) ||
        !std::isfinite(fy) || !std::isfinite(fz) || !std::isfinite(body_mass) || body_mass<=0 ||
        !std::isfinite(kp) || !std::isfinite(kd) || kp<0 || kd<0 ||
        !std::isfinite(horizon) || !std::isfinite(per_hip_limit) || per_hip_limit<=0) return out;
    const double h=std::clamp(horizon,0.030,0.080);
    const double inertia=0.159013*body_mass/14.0;
    out.force_feedforward=0.5*body_mass*(0.12396677*fy-0.00761282*fz);
    out.feedback=0.5*inertia/(inertia+h*kd+h*h*kp)*
        (-kp*(pitch-reference)-(kd+h*kp)*rate);
    out.command=std::clamp(out.force_feedforward+out.feedback,-per_hip_limit,per_hip_limit);
    return out;
}

// Reserve the common hip mode for the box. Leg support and shape feedback
// contribute only to the left/right difference; neither can cancel box PD.
// Limit the difference before output clipping, preserving the requested mean.
inline JointVector allocate_torso_hips(const JointVector & leg_torque,
                                      double torso_per_hip, double hip_limit) {
    JointVector result=leg_torque;
    const double common=std::clamp(torso_per_hip,-hip_limit,hip_limit);
    const double room=hip_limit-std::abs(common);
    const double difference=std::clamp(0.5*(leg_torque[0]-leg_torque[2]),-room,room);
    result[0]=common+difference;
    result[2]=common-difference;
    return result;
}
} // namespace bbot_jump
