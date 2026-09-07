#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace bbot_jump
{
using JointVector = Eigen::Matrix<double, 4, 1>;
using JointMatrix = Eigen::Matrix<double, 4, 4>;

// 平面浮动基座惯量，关节顺序 HL/KL/HR/KR。
// 参数来自当前 bbot.urdf.xacro 的质量、COM、ixx 和关节偏移。
// 基座平移/俯仰是自由度，不能把空中腿当作固定在地面的独立转子。
// 轮子独立自转的转动惯量不锁在腿轴上；轮驱动作用作为外部扰动。
inline JointMatrix flight_joint_inertia(const JointVector & q, double body_mass)
{
    Eigen::Matrix<double, 7, 7> mass = Eigen::Matrix<double, 7, 7>::Zero();
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
    // Schur 补消去自由基座，不显式计算矩阵逆。
    JointMatrix result = mass.bottomRightCorner<4, 4>() -
        mass.bottomLeftCorner<4, 3>() *
        mass.topLeftCorner<3, 3>().ldlt().solve(mass.topRightCorner<3, 4>());
    result = (0.5 * (result + result.transpose())).eval();
    result.diagonal().array() += 1e-8;
    return result;
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
