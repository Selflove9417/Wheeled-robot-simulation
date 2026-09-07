#pragma once

#include <array>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>
#include "bbot_balance_controller/takeoff_detection.hpp"
#include "bbot_balance_controller/world_pose_velocity.hpp"

namespace bbot_jump {

// Current CAD model, expressed in base_link (X: joint axis, Y: forward, Z: up).
// Match the controller's seven-body mass model; the 10 g fixed IMU is omitted.
// Wheel spin does not move its COM. Both legs contribute even when asymmetric.
struct CentroidalGeometry {
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d axle = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,3,4> com_jacobian = Eigen::Matrix<double,3,4>::Zero();
    Eigen::Matrix<double,3,4> axle_jacobian = Eigen::Matrix<double,3,4>::Zero();
};

inline Eigen::Vector3d rotate_about_hip(double angle, const Eigen::Vector3d & p) {
    const double c=std::cos(angle), s=std::sin(angle);
    return {p.x(), c*p.y()-s*p.z(), s*p.y()+c*p.z()};
}
inline Eigen::Vector3d hip_rotation_derivative(const Eigen::Vector3d & p) {
    return {0.0,-p.z(),p.y()};
}

inline CentroidalGeometry centroidal_geometry(const std::array<double,4> & q, double body_mass) {
    CentroidalGeometry out;
    const double total_mass=body_mass+8.0;
    out.com=body_mass*Eigen::Vector3d(0.20001846,0.13261282,0.05396677);
    for (int side=0;side<2;++side) {
        const int h=2*side, k=h+1;
        const Eigen::Vector3d hip(side==0?0.3032:0.0965,0.125,-0.07);
        const auto thigh=rotate_about_hip(q[h],{side==0?0.06107357:-0.06077357,-0.13690699,-0.02116697});
        const auto knee=rotate_about_hip(q[h],{side==0?0.072:-0.0667,-0.29348091,-0.06220095});
        const auto shank=rotate_about_hip(q[h]+q[k],{side==0?-0.01104398:0.00604399,0.11538205,-0.08532288});
        const auto wheel=rotate_about_hip(q[h]+q[k],{side==0?-0.022:-0.0405,0.28210870,-0.19553796});
        const Eigen::Vector3d wheel_com=hip+knee+wheel+
            Eigen::Vector3d(side==0?0.03825001:0.01925,0.0,0.0);
        out.com+=1.2*(hip+thigh)+0.8*(hip+knee+shank)+2.0*wheel_com;
        out.com_jacobian.col(h)=hip_rotation_derivative(
            (1.2*thigh+0.8*(knee+shank)+2.0*(knee+wheel))/total_mass);
        out.com_jacobian.col(k)=hip_rotation_derivative((0.8*shank+2.0*wheel)/total_mass);
        out.axle+=0.5*(hip+knee+wheel);
        out.axle_jacobian.col(h)=0.5*hip_rotation_derivative(knee+wheel);
        out.axle_jacobian.col(k)=0.5*hip_rotation_derivative(wheel);
    }
    out.com/=total_mass;
    return out;
}

// Inverse-dynamics gravity term for the four joints with base pose fixed:
// g_q = d(sum(m_i*g*z_i))/dq. Only the moving leg/wheel masses contribute.
// Ground support is -J_wheel^T*F_contact + g_q. Using only J^T F models
// massless legs and leaves a systematic hip moment for body PD to overcome.
// Add this beside support feedforward, outside the implicit feedback solve.
// Do not use fixed-base gravity compensation in free fall.
inline Eigen::Vector4d leg_gravity_torques(const std::array<double,4> & q,
                                         double pitch, double body_mass) {
    const Eigen::Vector3d world_vertical(0.0,-std::sin(pitch),std::cos(pitch));
    const auto geometry=centroidal_geometry(q,body_mass);
    return (body_mass+8.0)*9.81*geometry.com_jacobian.transpose()*world_vertical;
}

struct CentroidalBalanceState {
    double forward=0.0;
    double height=0.0; // COM above the mean wheel axle, not base_link height
    double angle=0.0;
    double rate=0.0;
    bool valid=false;
};

inline CentroidalBalanceState centroidal_balance_state(
    const std::array<double,4> & q, const std::array<double,4> & v,
    double pitch, double pitch_rate, double body_mass)
{
    CentroidalBalanceState out;
    if (!std::isfinite(body_mass) || body_mass<=0.0 ||
        !std::isfinite(pitch) || !std::isfinite(pitch_rate)) return out;
    for (int i=0;i<4;++i) if (!std::isfinite(q[i]) || !std::isfinite(v[i])) return out;
    const auto geometry=centroidal_geometry(q,body_mass);
    const Eigen::Map<const Eigen::Vector4d> velocity(v.data());
    const auto r=rotate_about_hip(-pitch,geometry.com-geometry.axle);
    const Eigen::Vector3d dr=rotate_about_hip(-pitch,
        (geometry.com_jacobian-geometry.axle_jacobian)*velocity)-
        pitch_rate*hip_rotation_derivative(r);
    out.forward=r.y();
    out.height=r.z();
    out.angle=std::atan2(r.y(),r.z());
    const double length2=r.y()*r.y()+r.z()*r.z();
    if (length2<0.01 || r.z()<0.10) return out;
    out.rate=(r.z()*dr.y()-r.y()*dr.z())/length2;
    out.valid=std::isfinite(out.rate);
    return out;
}

// Horizontal capture must use world COM translation, not wheel spin or the
// moving base origin. Align joints to each pose before differentiating.
class CentroidalWorldObserver {
    WorldPoseVelocity velocity_;
public:
    bool update(double stamp, double now, const Eigen::Vector3d & base,
                const Eigen::Matrix3d & rotation, const JointPoseHistory & history,
                double body_mass) {
        std::array<double,4> q{};
        if (!std::isfinite(now) || !std::isfinite(stamp) || !base.allFinite() ||
            !rotation.allFinite() ||
            (rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
            std::abs(rotation.determinant()-1.0)>1e-5 ||
            !std::isfinite(body_mass) || body_mass<=0 ||
            now<stamp || now-stamp>0.080 || !history.interpolate(stamp,q)) return false;
        const Eigen::Vector3d com=base+rotation*centroidal_geometry(q,body_mass).com;
        return velocity_.update(stamp,{com.x(),com.y(),com.z()});
    }
    bool valid(double now) const {
        return std::isfinite(now) && velocity_.valid() &&
            now>=velocity_.sample_stamp() && now-velocity_.sample_stamp()<=0.080;
    }
    double forward_velocity(const Eigen::Vector3d & heading) const {
        const auto & v=velocity_.velocity();
        return v[0]*heading.x()+v[1]*heading.y();
    }
};

// Differentiate the world COM position, after aligning q and odometry. Never
// combine an old base position with newer retracting legs, or treat base_link
// speed as total vertical momentum. Keep timestamp/freshness semantics explicit.
class CentroidalHeightObserver {
    WorldPoseVelocity velocity_;
    double height_=0.0;
public:
    bool update(double stamp, double now, double base_z, const Eigen::Vector3d & world_z_in_body,
                const JointPoseHistory & history, double body_mass) {
        std::array<double,4> q{};
        if (!std::isfinite(now) || !std::isfinite(stamp) ||
            !std::isfinite(base_z) || !world_z_in_body.allFinite() ||
            std::abs(world_z_in_body.norm()-1.0)>1e-5 ||
            !std::isfinite(body_mass) || body_mass<=0.0 ||
            now-stamp>0.080 || stamp>now+0.001 || !history.interpolate(stamp,q)) return false;
        height_=base_z+world_z_in_body.dot(centroidal_geometry(q,body_mass).com);
        return velocity_.update(stamp,{0.0,0.0,height_});
    }
    bool valid(double now) const {
        return std::isfinite(now) && velocity_.valid() &&
            now>=velocity_.sample_stamp() && now-velocity_.sample_stamp()<=0.080;
    }
    double height() const { return height_; }
    double velocity() const { return velocity_.velocity()[2]; }
    double stamp() const { return velocity_.sample_stamp(); }
};
} // namespace bbot_jump
