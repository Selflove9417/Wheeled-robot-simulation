#pragma once

#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace bbot_jump
{
// Ground leg tasks have three independent modes: hip difference and the two
// knees. The common hip mode belongs to the independent torso pitch servo.
// Reduce BEFORE the implicit solve so mean hip target error cannot leak into
// knee commands through the free-leg inertia coupling. Keep differential hip
// and both knee stiffness/damping at their existing values.
inline JointVector discrete_ground_leg_feedback(
    const JointMatrix & mass, const JointVector & q, const JointVector & v,
    const JointVector & q_des, const JointVector & v_des,
    const JointVector & kp, const JointVector & kd, double horizon)
{
    Eigen::Matrix<double,4,3> basis=Eigen::Matrix<double,4,3>::Zero();
    constexpr double inv_sqrt2=0.7071067811865475244;
    basis(0,0)=inv_sqrt2; basis(2,0)=-inv_sqrt2;
    basis(1,1)=1.0; basis(3,2)=1.0;
    const Eigen::Matrix3d m=basis.transpose()*mass*basis;
    const Eigen::Matrix3d k=basis.transpose()*kp.asDiagonal()*basis;
    const Eigen::Matrix3d d=basis.transpose()*kd.asDiagonal()*basis;
    const double h=std::clamp(horizon,0.025,0.060);
    const Eigen::Matrix3d implicit=m+h*d+h*h*k;
    const Eigen::Vector3d rhs=k*basis.transpose()*(q_des-q)+
        (d+h*k)*basis.transpose()*(v_des-v);
    return basis*m*implicit.ldlt().solve(rhs);
}

// Return feedback only. The caller adds the support feedforward separately:
// filtering J^T F through this solve would remove the static load compensation.
//
// A grounded hip-axis mode has pitch acceleration approximately b^T qddot,
// b=[1/2,0,1/2,0]. Include BOTH joint and body feedback in the implicit solve;
// leaving the larger body damping outside would retain a delayed explicit loop.
// attitude_feedback is the existing, already bounded body correction per hip.
// The rank-one body approximation does not model rolling/contact dynamics.
inline JointVector discrete_ground_feedback(
    const JointMatrix & mass,
    const JointVector & q, const JointVector & v,
    const JointVector & q_des, const JointVector & v_des,
    const JointVector & kp, const JointVector & kd,
    const JointVector & attitude_feedback, double pitch_rate,
    double body_kp, double body_kd, double horizon)
{
    const double h = std::clamp(horizon, 0.025, 0.060);
    const double kb = std::max(0.0, body_kp);
    const double db = std::max(0.0, body_kd);
    const JointVector b(0.5, 0.0, 0.5, 0.0);
    JointMatrix implicit_mass = mass;
    implicit_mass.diagonal() += h * kd + h * h * kp;
    implicit_mass += (h * db + h * h * kb) * b * b.transpose();
    // The current body P/D correction is already in attitude_feedback.
    // Predict its P term to pitch+h*pitch_rate before solving the acceleration.
    const JointVector rhs = kp.cwiseProduct(q_des - q) +
        (kd + h * kp).cwiseProduct(v_des - v) +
        attitude_feedback - h * kb * pitch_rate * b;
    return mass * implicit_mass.ldlt().solve(rhs);
}
} // namespace bbot_jump
