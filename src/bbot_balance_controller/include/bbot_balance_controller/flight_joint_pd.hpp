#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace bbot_jump
{
using JointVector = Eigen::Matrix<double, 4, 1>;
using JointMatrix = Eigen::Matrix<double, 4, 4>;
using FloatingMassMatrix = Eigen::Matrix<double, 7, 7>;
using PitchJointMassMatrix = Eigen::Matrix<double, 5, 5>;
using PitchJointMatrix = Eigen::Matrix<double, 5, 5>;
using PitchJointVector = Eigen::Matrix<double, 5, 1>;

// 平面浮动基座惯量，关节顺序 HL/KL/HR/KR。
// 参数来自当前 bbot.urdf.xacro 的质量、COM、ixx 和关节偏移。
// 基座平移/俯仰是自由度，不能把空中腿当作固定在地面的独立转子。
// 轮子独立自转的转动惯量不锁在腿轴上；轮驱动作用作为外部扰动。
inline FloatingMassMatrix flight_floating_mass_matrix(const JointVector & q, double body_mass)
{
    FloatingMassMatrix mass = FloatingMassMatrix::Zero();
    const auto rotate = [](double angle, const Eigen::Vector2d & v) -> Eigen::Vector2d {
        const double c = std::cos(angle), s = std::sin(angle);
        return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
    };
    const auto perpendicular = [](const Eigen::Vector2d & v) -> Eigen::Vector2d {
        return {-v.y(), v.x()};
    };
    const auto add = [&](double m, double inertia, const Eigen::Vector2d & r,
                         int hip, int knee, const Eigen::Vector2d & rh,
                         const Eigen::Vector2d & rk) {
        Eigen::Matrix<double, 2, 7> j = Eigen::Matrix<double, 2, 7>::Zero();
        j.leftCols<2>().setIdentity();
        j.col(2) = perpendicular(r);
        Eigen::Matrix<double, 7, 1> w = Eigen::Matrix<double, 7, 1>::Zero();
        w[2] = 1.0;
        if (hip >= 0) { j.col(hip) = perpendicular(rh); w[hip] = 1.0; }
        if (knee >= 0) { j.col(knee) = perpendicular(rk); w[knee] = 1.0; }
        mass += m * j.transpose() * j + inertia * w * w.transpose();
    };
    add(body_mass, 0.159013 * body_mass / 14.0, {0.13261282, 0.05396677},
        -1, -1, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
    const Eigen::Vector2d hip_offset(0.125, -0.07);
    for (int side = 0; side < 2; ++side) {
        const int h = 3 + 2 * side, k = h + 1;
        const double qh = q[2 * side], qk = q[2 * side + 1];
        const Eigen::Vector2d thigh_com = rotate(qh, {-0.13690699, -0.02116697});
        add(1.20, 0.017921, hip_offset + thigh_com, h, -1,
            thigh_com, Eigen::Vector2d::Zero());
        const Eigen::Vector2d knee_offset = rotate(qh, {-0.29348091, -0.06220095});
        const Eigen::Vector2d shank_com = rotate(qh + qk, {0.11538205, -0.08532288});
        add(0.80, 0.013130, hip_offset + knee_offset + shank_com, h, k,
            knee_offset + shank_com, shank_com);
        const Eigen::Vector2d wheel_offset = rotate(qh + qk, {0.28210870, -0.19553796});
        add(2.00, 0.0, hip_offset + knee_offset + wheel_offset, h, k,
            knee_offset + wheel_offset, wheel_offset);
    }
    return (0.5 * (mass + mass.transpose())).eval();
}

// Eliminate only base translations. Retain [theta, HL, KL, HR, KR] so the
// Coriolis bias keeps the nonzero base angular-momentum contribution.
inline PitchJointMassMatrix flight_pitch_joint_mass_matrix(
    const JointVector & q, double body_mass)
{
    const FloatingMassMatrix mass = flight_floating_mass_matrix(q, body_mass);
    const Eigen::Matrix2d translation = mass.topLeftCorner<2, 2>();
    const Eigen::Matrix<double, 2, 5> translation_cross = mass.block<2, 5>(0, 2);
    PitchJointMassMatrix result = mass.block<5, 5>(2, 2) -
        translation_cross.transpose() * translation.ldlt().solve(translation_cross);
    return (0.5 * (result + result.transpose())).eval();
}

// Schur complement of the retained pitch coordinate recovers the historical
// 4x4 joint inertia without changing its physical model.
inline JointMatrix flight_joint_inertia(const JointVector & q, double body_mass)
{
    const PitchJointMassMatrix mass = flight_pitch_joint_mass_matrix(q, body_mass);
    JointMatrix result = mass.bottomRightCorner<4, 4>() -
        mass.bottomLeftCorner<4, 1>() *
        (mass.topLeftCorner<1, 1>().ldlt().solve(mass.topRightCorner<1, 4>()));
    result = (0.5 * (result + result.transpose())).eval();
    result.diagonal().array() += 1e-8;
    return result;
}

// Christoffel velocity bias of the five-coordinate translation-reduced model,
// subsequently Schur-eliminated in theta. theta_rate follows CAD's positive
// CCW (y,z) convention; controller pitch is its negative.
inline bool flight_joint_coriolis_bias(
    const JointVector & q, const JointVector & qdot, double theta_rate,
    double body_mass, JointVector & bias,
    PitchJointVector * full_bias_out = nullptr)
{
    bias.setZero();
    if (full_bias_out) full_bias_out->setZero();
    if (!q.allFinite() || !qdot.allFinite() || !std::isfinite(theta_rate) ||
        !std::isfinite(body_mass) || body_mass <= 0.0)
        return false;
    const PitchJointMassMatrix base = flight_pitch_joint_mass_matrix(q, body_mass);
    if (!base.allFinite() || base(0, 0) <= 1e-9)
        return false;
    Eigen::Matrix<double, 5, 5> derivative[5];
    derivative[0].setZero();
    constexpr double eps = 1e-5;
    for (int k = 0; k < 4; ++k)
    {
        JointVector qp = q, qm = q;
        qp[k] += eps;
        qm[k] -= eps;
        derivative[k + 1] =
            (flight_pitch_joint_mass_matrix(qp, body_mass) -
             flight_pitch_joint_mass_matrix(qm, body_mass)) / (2.0 * eps);
    }
    PitchJointVector velocity;
    velocity << theta_rate, qdot;
    PitchJointVector christoffel = PitchJointVector::Zero();
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            for (int k = 0; k < 5; ++k)
                christoffel[i] += 0.5 *
                    (derivative[k](i, j) + derivative[j](i, k) - derivative[i](j, k)) *
                    velocity[j] * velocity[k];
    if (full_bias_out) *full_bias_out = christoffel;
    bias = christoffel.tail<4>() -
        base.block<4, 1>(1, 0) * (christoffel[0] / base(0, 0));
    return bias.allFinite();
}

inline JointVector flight_joint_dynamics_feedforward(
    const JointVector & q, const JointVector & qdot, const JointVector & qddot,
    double theta_rate, double body_mass, JointVector * inertial = nullptr,
    JointVector * velocity_bias = nullptr)
{
    if (inertial) inertial->setZero();
    if (velocity_bias) velocity_bias->setZero();
    if (!q.allFinite() || !qdot.allFinite() || !qddot.allFinite() ||
        !std::isfinite(theta_rate) || !std::isfinite(body_mass) || body_mass <= 0.0)
        return JointVector::Zero();
    const JointMatrix reduced_mass = flight_joint_inertia(q, body_mass);
    JointVector bias = JointVector::Zero();
    if (!flight_joint_coriolis_bias(q, qdot, theta_rate, body_mass, bias))
        return JointVector::Zero();
    const JointVector inertia = reduced_mass * qddot;
    if (inertial) *inertial = inertia;
    if (velocity_bias) *velocity_bias = bias;
    return inertia + bias;
}

inline JointVector bound_arrest_dynamics_feedforward(
    const JointVector & raw, const JointVector & limits, double blend)
{
    if (!raw.allFinite() || !limits.allFinite() || !std::isfinite(blend) ||
        (limits.array() <= 0.0).any())
        return JointVector::Zero();
    return (raw * std::clamp(blend, 0.0, 1.0)).cwiseMax(-limits).cwiseMin(limits);
}

enum class ArrestDynamicsGuard : int
{
    DisabledOrWrongPhase = 0,
    EffortUnavailable = 1,
    JointSampleStale = 2,
    ImuSampleStale = 3,
    InvalidInput = 4,
    Active = 5,
};

inline ArrestDynamicsGuard arrest_dynamics_guard(
    bool enabled, bool arrest, bool effort_active, bool switch_pending,
    double now, double joint_stamp, double imu_stamp,
    const JointVector & q, const JointVector & qdot, const JointVector & qddot,
    double pitch_rate, double max_joint_age = 0.040, double max_imu_age = 0.080)
{
    if (!enabled || !arrest) return ArrestDynamicsGuard::DisabledOrWrongPhase;
    if (!effort_active || switch_pending) return ArrestDynamicsGuard::EffortUnavailable;
    if (!std::isfinite(now) || !std::isfinite(joint_stamp) ||
        now < joint_stamp || now - joint_stamp > max_joint_age)
        return ArrestDynamicsGuard::JointSampleStale;
    if (!std::isfinite(imu_stamp) || now < imu_stamp || now - imu_stamp > max_imu_age)
        return ArrestDynamicsGuard::ImuSampleStale;
    if (!q.allFinite() || !qdot.allFinite() || !qddot.allFinite() ||
        !std::isfinite(pitch_rate))
        return ArrestDynamicsGuard::InvalidInput;
    return ArrestDynamicsGuard::Active;
}

inline double arrest_dynamics_feedforward_blend(double elapsed, double ramp = 0.020)
{
    if (!std::isfinite(elapsed) || !std::isfinite(ramp) || ramp <= 0.0 || elapsed <= 0.0)
        return 0.0;
    const double u = std::clamp(elapsed / ramp, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

// 隐式离散 PD：按下一持有周期的 q/v 求解，而非让高D作用于旧采样。
// A = M + h*D + h²*K
// a = A^-1 [ff + K*(qd-q) + (D+h*K)*(vd-v)]，tau = M*a。
// h 包含100 Hz状态/执行器周期及采样延迟；不降低地面承重PD。
inline JointVector discrete_flight_pd(const JointMatrix & mass,
                                     const JointVector & q, const JointVector & v,
                                     const JointVector & q_des, const JointVector & v_des,
                                     const JointVector & kp, const JointVector & kd,
                                     const JointVector & feedforward, double horizon)
{
    const double h = std::clamp(horizon, 0.02, 0.06);
    JointMatrix implicit_mass = mass;
    implicit_mass.diagonal() += h * kd + h * h * kp;
    const JointVector rhs = feedforward + kp.cwiseProduct(q_des - q) +
        (kd + h * kp).cwiseProduct(v_des - v);
    return mass * implicit_mass.ldlt().solve(rhs);
}
} // namespace bbot_jump
