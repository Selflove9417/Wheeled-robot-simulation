#pragma once

#include <array>
#include <deque>
#include <algorithm>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>

#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/flight_joint_pd.hpp"
#include "bbot_balance_controller/landing_repair_momentum.hpp"

namespace bbot_jump {

struct VelocityLaunchCaptureReference {
    bool valid = false;
    double hip_velocity = 0.0;
    double knee_velocity = 0.0;
    double predicted_com_vz = 0.0;
    double predicted_momentum = 0.0;
    double normalized_determinant = 0.0;
};

// Symmetric-leg velocity reference for supported, bilateral-wheel motion.
// This solves COM-z velocity and pitch-axis angular momentum equalities. It is
// a kinematic/momentum reference, not inverse dynamics or a force allocator.
// Contact/freshness/mode qualification belongs to the caller.
inline VelocityLaunchCaptureReference velocity_launch_capture_reference(
    const JointVector & q, double pitch, double desired_pitch_rate,
    double desired_com_vz, double target_momentum,
    const std::array<double, 2> & wheel_relative_velocity,
    double body_mass, double hip_speed_limit = 11.0,
    double knee_speed_limit = 15.0, double travel_horizon = 0.030)
{
    VelocityLaunchCaptureReference out;
    if (!q.allFinite() || !std::isfinite(pitch) ||
        !std::isfinite(desired_pitch_rate) || !std::isfinite(desired_com_vz) ||
        !std::isfinite(target_momentum) ||
        !std::isfinite(wheel_relative_velocity[0]) ||
        !std::isfinite(wheel_relative_velocity[1]) ||
        !std::isfinite(body_mass) || body_mass <= 0.0 ||
        !std::isfinite(hip_speed_limit) || hip_speed_limit <= 0.0 ||
        !std::isfinite(knee_speed_limit) || knee_speed_limit <= 0.0 ||
        !std::isfinite(travel_horizon) || travel_horizon < 0.0) return out;

    for (int i = 0; i < 4; ++i) {
        const double limit = (i % 2 == 0) ? 1.52 : 1.56;
        if (std::abs(q[i]) > limit) return out;
    }

    const auto geom = centroidal_geometry({q[0], q[1], q[2], q[3]}, body_mass);
    const Eigen::Vector3d relative = rotate_about_hip(-pitch, geom.com - geom.axle);
    const Eigen::Vector3d vertical(0.0, -std::sin(pitch), std::cos(pitch));
    const Eigen::RowVector4d j = vertical.transpose() *
        (geom.com_jacobian - geom.axle_jacobian);
    const auto mass = flight_pitch_joint_mass_matrix(q, body_mass);
    if (!mass.allFinite() || !relative.allFinite() || !j.allFinite()) return out;

    const double com_h = j[0] + j[2];
    const double com_k = j[1] + j[3];
    constexpr double wheel_inertia = 0.006481;
    // theta = -pitch. Each absolute wheel rate is
    // -pitch_rate + hip_rate + knee_rate + wheel_relative_rate.
    // Therefore both wheel inertias contribute to pitch, hip and knee terms.
    const double pitch_inertia = mass(0, 0) + 2.0 * wheel_inertia;
    const double momentum_h = mass(0, 1) + mass(0, 3) + 2.0 * wheel_inertia;
    const double momentum_k = mass(0, 2) + mass(0, 4) + 2.0 * wheel_inertia;

    Eigen::Matrix2d a;
    a << com_h, com_k, momentum_h, momentum_k;
    const Eigen::Vector2d b(
        desired_com_vz + relative.y() * desired_pitch_rate,
        target_momentum + pitch_inertia * desired_pitch_rate -
            wheel_inertia * (wheel_relative_velocity[0] + wheel_relative_velocity[1]));
    if (!a.allFinite() || !b.allFinite()) return out;
    const double det = a.determinant();
    const double scale = std::hypot(com_h, com_k) *
        std::hypot(momentum_h, momentum_k);
    if (!std::isfinite(det) || !std::isfinite(scale) || scale < 1e-12 ||
        std::abs(det) <= 1e-8 * scale) return out;
    out.normalized_determinant = std::abs(det) / scale;
    if (out.normalized_determinant < 0.05) return out;
    const Eigen::Vector2d velocity = a.fullPivLu().solve(b);
    if (!velocity.allFinite() || (a*velocity-b).norm() > 1e-8) return out;
    const double hip = velocity[0], knee = velocity[1];
    if (std::abs(hip) > hip_speed_limit || std::abs(knee) > knee_speed_limit)
        return out;
    for (int i = 0; i < 4; ++i) {
        const double qdot = (i % 2 == 0) ? hip : knee;
        const double limit = (i % 2 == 0) ? 1.52 : 1.56;
        if (std::abs(q[i] + travel_horizon * qdot) > limit) return out;
    }

    out.hip_velocity = hip;
    out.knee_velocity = knee;
    out.predicted_com_vz = com_h * hip + com_k * knee -
        relative.y() * desired_pitch_rate;
    out.predicted_momentum = -pitch_inertia * desired_pitch_rate +
        momentum_h * hip + momentum_k * knee + wheel_inertia *
        (wheel_relative_velocity[0] + wheel_relative_velocity[1]);
    out.valid = std::isfinite(out.predicted_com_vz) &&
        std::isfinite(out.predicted_momentum);
    return out;
}

struct VelocityCapturePairedSample {
    bool valid=false;
    LandingJointSample joints;
    double pitch=0.0, rate=0.0;
};

// Callback order is irrelevant. Only exact, finite, fresh source timestamps
// can form a pair; never interpolate or extrapolate a missing velocity.
class VelocityCaptureSampleHistory {
    struct Imu {double stamp, pitch, rate;};
    std::deque<Imu> imu_;
    std::deque<LandingJointSample> joints_;
public:
    void clear() {imu_.clear(); joints_.clear();}
    void imu(double stamp,double pitch,double rate) {
        if (!std::isfinite(stamp)||stamp<=0||!std::isfinite(pitch)||!std::isfinite(rate)) return;
        if (!imu_.empty()&&stamp<imu_.back().stamp) clear();
        if (!imu_.empty()&&stamp==imu_.back().stamp) imu_.back()={stamp,pitch,rate};
        else imu_.push_back({stamp,pitch,rate});
        while(imu_.size()>64) imu_.pop_front();
    }
    void joints(const LandingJointSample& x) {
        if (!std::isfinite(x.stamp)||x.stamp<=0||!x.q.allFinite()||!x.v.allFinite()||
            !std::isfinite(x.wheel_rate[0])||!std::isfinite(x.wheel_rate[1])) return;
        if (!joints_.empty()&&x.stamp<joints_.back().stamp) clear();
        if (!joints_.empty()&&x.stamp==joints_.back().stamp) joints_.back()=x;
        else joints_.push_back(x);
        while(joints_.size()>64) joints_.pop_front();
    }
    VelocityCapturePairedSample paired(double now) const {
        if (!std::isfinite(now)) return {};
        for(auto j=joints_.rbegin();j!=joints_.rend();++j) {
            if (now<j->stamp||now-j->stamp>0.020) continue;
            for(auto i=imu_.rbegin();i!=imu_.rend();++i)
                if(std::abs(i->stamp-j->stamp)<=1e-9) return {true,*j,i->pitch,i->rate};
        }
        return {};
    }
};

struct VelocityCaptureStoppingDistance {
    bool valid=false, brake=false;
    std::array<double,4> remaining{}, required{};
};

// Deceleration is an empirical lower design estimate qualified by a recorded
// operating envelope, not a model proof or the reference acceleration limit.
inline VelocityCaptureStoppingDistance velocity_capture_stopping_distance(
    const std::array<double,4>& q,const std::array<double,4>& v,
    double hip_deceleration,double knee_deceleration,double latency) {
    VelocityCaptureStoppingDistance out;
    if (!std::isfinite(latency)||latency<0||!std::isfinite(hip_deceleration)||
        !std::isfinite(knee_deceleration)||hip_deceleration<=0||knee_deceleration<=0) return out;
    for (size_t i=0;i<4;++i) {
        if (!std::isfinite(q[i])||!std::isfinite(v[i])) return {};
        const double limit=i%2 ? 1.56 : 1.52;
        const double acceleration=i%2 ? knee_deceleration : hip_deceleration;
        out.remaining[i]=v[i]>=0 ? limit-q[i] : limit+q[i];
        if (std::abs(q[i])>=limit) out.brake=true;
        out.required[i]=std::abs(v[i])*latency+v[i]*v[i]/(2*acceleration)+0.010;
        if(out.remaining[i]<=out.required[i]) out.brake=true;
    }
    out.valid=true;
    return out;
}

enum class VelocityCapturePhase {Waiting=0,Tracking=1,Braking=2};
enum class VelocityCaptureReason {
    None=0,WaitingForGate=1,StaleSensors=2,ContactUnavailable=3,ActuatorUnavailable=4,
    AttitudeBlocked=5,InverseRejected=6,StoppingDistance=7,BrakeEvidenceUnavailable=8,ControlGap=9
};
struct VelocityCaptureSessionSample {
    VelocityCapturePhase phase=VelocityCapturePhase::Waiting;
    VelocityCaptureReason reason=VelocityCaptureReason::WaitingForGate;
    bool owns_command=false;
    std::array<double,4> reference{};
};

// One owner for this launch. A rejection latches braking until the jump ends;
// old PD/FF must never be used as a per-tick fallback after ownership begins.
class VelocityCaptureSession {
    VelocityCaptureSessionSample sample_;
    double stamp_=-1.0;
public:
    void reset(){sample_={};stamp_=-1;}
    const VelocityCaptureSessionSample& sample() const {return sample_;}
    VelocityCaptureSessionSample update(double now,bool gate_open,bool sensors_fresh,
        bool support,bool actuator_ready,bool attitude_blocked,bool inverse_valid,
        const std::array<double,4>& target,const std::array<double,4>& measured_velocity,
        const VelocityCaptureStoppingDistance& stopping) {
        if (!gate_open&&sample_.phase==VelocityCapturePhase::Waiting) return sample_;
        sample_.owns_command=true;
        VelocityCaptureReason reason=VelocityCaptureReason::None;
        if (!sensors_fresh) reason=VelocityCaptureReason::StaleSensors;
        else if (!support) reason=VelocityCaptureReason::ContactUnavailable;
        else if (!actuator_ready) reason=VelocityCaptureReason::ActuatorUnavailable;
        else if (attitude_blocked) reason=VelocityCaptureReason::AttitudeBlocked;
        else if (!stopping.valid) reason=VelocityCaptureReason::BrakeEvidenceUnavailable;
        else if (stopping.brake) reason=VelocityCaptureReason::StoppingDistance;
        else if (!inverse_valid) reason=VelocityCaptureReason::InverseRejected;
        else if(!std::isfinite(now)||(stamp_>=0&&(now<stamp_||now-stamp_>0.020)))
            reason=VelocityCaptureReason::ControlGap;
        for(double x:target) if(!std::isfinite(x)) reason=VelocityCaptureReason::InverseRejected;
        if (sample_.phase!=VelocityCapturePhase::Braking&&reason!=VelocityCaptureReason::None) {
            sample_.phase=VelocityCapturePhase::Braking;sample_.reason=reason;
        }
        if(sample_.phase==VelocityCapturePhase::Braking) {
            sample_.reference.fill(0);stamp_=now;return sample_;
        }
        const bool first=sample_.phase==VelocityCapturePhase::Waiting;
        const double dt=first ? 0.0 : std::max(0.0,now-stamp_);
        for(size_t i=0;i<4;++i) {
            const double previous=first ? measured_velocity[i] : sample_.reference[i];
            if(!std::isfinite(previous)) {
                sample_.phase=VelocityCapturePhase::Braking;
                sample_.reason=VelocityCaptureReason::StaleSensors;sample_.reference.fill(0);return sample_;
            }
            const double step=(i%2 ? 500.0 : 450.0)*dt;
            sample_.reference[i]=previous+std::clamp(target[i]-previous,-step,step);
        }
        stamp_=now;sample_.phase=VelocityCapturePhase::Tracking;sample_.reason=VelocityCaptureReason::None;
        return sample_;
    }
};

}  // namespace bbot_jump
