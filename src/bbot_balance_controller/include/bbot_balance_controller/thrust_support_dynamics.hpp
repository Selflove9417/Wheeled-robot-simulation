#pragma once

// Diagnostic / candidate model for bilateral rolling support.  It does not
// read contact forces and must not be described as a contact-wrench sensor.
// Coordinates are [base-forward, base-z, theta, hipL, kneeL, hipR, kneeR,
// wheelL-relative, wheelR-relative], with theta = -controller_pitch.

#include <Eigen/Core>
#include <Eigen/LU>
#include <array>
#include <cmath>
#include <limits>

#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace bbot_jump
{
using SupportQ9 = Eigen::Matrix<double, 9, 1>;
using SupportMatrix9 = Eigen::Matrix<double, 9, 9>;
using SupportContactJacobian = Eigen::Matrix<double, 4, 9>;
using SupportTaskJacobian = Eigen::Matrix<double, 3, 9>;
using SupportActuatorVector = Eigen::Matrix<double, 6, 1>;
using SupportContactVector = Eigen::Matrix<double, 4, 1>;
using SupportDecisionVector = Eigen::Matrix<double, 19, 1>;
using SupportTaskRow = Eigen::Matrix<double, 1, 19>;

constexpr double kSupportWheelRadius = 0.07;
constexpr double kSupportWheelAxialInertia = 0.006481;
constexpr double kSupportGravity = 9.81;

struct ThrustSupportDynamicsInput
{
    SupportQ9 q = SupportQ9::Zero();
    SupportQ9 v = SupportQ9::Zero();
    double body_mass = 9.5;
    double wheel_radius = kSupportWheelRadius;
    double gravity = kSupportGravity;
    double hip_knee_damping = 0.5;
    double hip_knee_coulomb = 0.1;
    double wheel_damping = 0.1;
    // Known from the existing wheel velocity-command path. This is not a
    // freely optimized wheel torque or a measured wheel acceleration.
    // Prescribed acceleration reference from the existing velocity servo.
    // The model tests the torque needed to track it; this is not measured
    // acceleration and is not guaranteed to be achieved by that servo.
    std::array<double, 2> wheel_relative_acceleration_reference{};
    std::array<double, 6> actuator_limits{150., 150., 150., 150., 50., 50.};
    std::array<double, 4> joint_position_min{-1.57, -1.57, -1.57, -1.57};
    std::array<double, 4> joint_position_max{1.57, 1.57, 1.57, 1.57};
    std::array<double, 6> actuator_speed_limits{30., 30., 30., 30., 30., 30.};
    double friction_coefficient = 1.0;
    double prediction_dt = 0.005;
};

struct ThrustSupportDynamicsModel
{
    bool valid = false;
    SupportMatrix9 mass = SupportMatrix9::Zero();
    SupportQ9 velocity_bias = SupportQ9::Zero();
    SupportQ9 gravity = SupportQ9::Zero();
    SupportQ9 joint_damping = SupportQ9::Zero();
    SupportContactJacobian contact_jacobian = SupportContactJacobian::Zero();
    Eigen::Matrix<double, 4, 1> contact_bias = Eigen::Matrix<double, 4, 1>::Zero();
    SupportTaskJacobian com_jacobian = SupportTaskJacobian::Zero(); // rows: forward, up, pitch-axis
    Eigen::Vector3d com_bias = Eigen::Vector3d::Zero();
    Eigen::Vector2d com = Eigen::Vector2d::Zero(); // world planar y,z
    // COM position/acceleration relative to the mean of the two wheel axles.
    // This is distinct from world-forward COM motion because the axles roll.
    Eigen::RowVector<double, 9> com_axle_relative_forward_jacobian =
        Eigen::RowVector<double, 9>::Zero();
    double com_axle_relative_forward_bias = 0.0;
    double com_axle_relative_forward = 0.0;
    Eigen::Matrix<double, 15, 19> equality = Eigen::Matrix<double, 15, 19>::Zero();
    Eigen::Matrix<double, 15, 1> equality_rhs = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 38, 19> inequality = Eigen::Matrix<double, 38, 19>::Zero();
    Eigen::Matrix<double, 38, 1> inequality_rhs = Eigen::Matrix<double, 38, 1>::Zero();
    int inequality_count = 0;
    Eigen::Matrix<double, 1, 4> contact_hdot_jacobian = Eigen::Matrix<double, 1, 4>::Zero();
};

struct ThrustSupportTasks
{
    // Lexicographic priority: vertical acceleration, centroidal H-dot,
    // forward COM acceleration. These are targets, not guarantees.
    double desired_com_vertical_acceleration = 0.0;
    double desired_centroidal_hdot = 0.0;
    double desired_com_forward_acceleration = 0.0;
    // Relative support-line task. This is the acceleration of COM forward
    // position minus the mean wheel-axle forward position, not world COM acc.
    double desired_com_axle_relative_forward_acceleration = 0.0;
    // Private opt-in priority: vertical COM, body pitch acceleration, H-dot,
    // then COM relative to the mean axle. Physical pitch is minus model theta.
    bool body_pitch_enabled = false;
    double desired_body_pitch_acceleration = 0.0;
};

struct ThrustSupportTaskResiduals
{
    bool valid = false;
    double vertical_acceleration = 0.0;
    double centroidal_hdot = 0.0;
    double body_pitch_acceleration = 0.0;
    double forward_acceleration = 0.0;
    double com_axle_relative_forward_acceleration = 0.0;
    double max_dynamics_residual = 0.0;
    double max_contact_acceleration_residual = 0.0;
    double max_inequality_violation = 0.0;
};

inline Eigen::Vector2d support_rotate(double angle, const Eigen::Vector2d &p)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * p.x() - s * p.y(), s * p.x() + c * p.y()};
}

inline Eigen::Vector2d support_perp(const Eigen::Vector2d &p)
{
    return {-p.y(), p.x()};
}

inline SupportMatrix9 thrust_support_mass_matrix(
    const SupportQ9 &q, double body_mass)
{
    SupportMatrix9 m = SupportMatrix9::Zero();
    const double theta = q[2];
    // Base link inertia and translational/rotational kinetic energy.
    const Eigen::Vector2d body_com(.13261282, .05396677);
    Eigen::Matrix<double, 2, 9> jb = Eigen::Matrix<double, 2, 9>::Zero();
    jb(0, 0) = 1.; jb(1, 1) = 1.;
    jb.col(2) = support_perp(support_rotate(theta, body_com));
    m.noalias() += body_mass * jb.transpose() * jb;
    Eigen::Matrix<double, 9, 1> wb = Eigen::Matrix<double, 9, 1>::Zero();
    wb[2] = 1.;
    m.noalias() += (0.159013 * body_mass / 14.0) * wb * wb.transpose();

    const Eigen::Vector2d hip_offset(.125, -.07);
    for (int side = 0; side < 2; ++side)
    {
        const int h = 3 + 2 * side, k = h + 1, wr = 7 + side;
        const double qh = q[h], qk = q[k];
        const Eigen::Vector2d thigh_local = support_rotate(qh, {-.13690699, -.02116697});
        const Eigen::Vector2d knee_local = support_rotate(qh, {-.29348091, -.06220095});
        const Eigen::Vector2d shank_local = support_rotate(qh + qk, {.11538205, -.08532288});
        const Eigen::Vector2d wheel_local = support_rotate(qh + qk, {.28210870, -.19553796});

        const auto add_link = [&](double mass_value, double inertia,
                                  const Eigen::Vector2d &local,
                                  const Eigen::Vector2d &dqh,
                                  const Eigen::Vector2d &dqk,
                                  bool knee_link) {
            const Eigen::Vector2d r = hip_offset + local;
            Eigen::Matrix<double, 2, 9> j = Eigen::Matrix<double, 2, 9>::Zero();
            j(0, 0) = 1.; j(1, 1) = 1.;
            j.col(2) = support_perp(support_rotate(theta, r));
            j.col(h) = support_rotate(theta, dqh);
            j.col(k) = support_rotate(theta, dqk);
            m.noalias() += mass_value * j.transpose() * j;
            Eigen::Matrix<double, 9, 1> w = Eigen::Matrix<double, 9, 1>::Zero();
            w[2] = 1.; w[h] = 1.;
            if (inertia > 0.)
            {
                if (knee_link) w[k] = 1.;
                m.noalias() += inertia * w * w.transpose();
            }
        };
        add_link(1.20, .017921, thigh_local,
                 support_perp(thigh_local), Eigen::Vector2d::Zero(), false);
        add_link(.80, .013130, knee_local + shank_local,
                 support_perp(knee_local + shank_local), support_perp(shank_local), true);
        // Wheel body COM is a point mass; independent axial rotor inertia is
        // added below using its absolute rate.
        add_link(2.00, 0., knee_local + wheel_local,
                 support_perp(knee_local + wheel_local), support_perp(wheel_local), true);

        Eigen::Matrix<double, 9, 1> rotor = Eigen::Matrix<double, 9, 1>::Zero();
        rotor[2] = 1.; rotor[h] = 1.; rotor[k] = 1.; rotor[wr] = 1.;
        m.noalias() += kSupportWheelAxialInertia * rotor * rotor.transpose();
    }
    return (0.5 * (m + m.transpose())).eval();
}

inline Eigen::Matrix<double, 2, 9> thrust_support_wheel_center_jacobian(
    const SupportQ9 &q, int side)
{
    const int h = 3 + 2 * side, k = h + 1;
    const Eigen::Vector2d hip_offset(.125, -.07);
    const Eigen::Vector2d knee = support_rotate(q[h], {-.29348091, -.06220095});
    const Eigen::Vector2d wheel = support_rotate(q[h] + q[k], {.28210870, -.19553796});
    const Eigen::Vector2d local = hip_offset + knee + wheel;
    const Eigen::Vector2d dqh = support_perp(knee + wheel);
    const Eigen::Vector2d dqk = support_perp(wheel);
    const Eigen::Vector2d rotated = support_rotate(q[2], local);
    Eigen::Matrix<double, 2, 9> j = Eigen::Matrix<double, 2, 9>::Zero();
    j(0, 0) = 1.; j(1, 1) = 1.;
    j.col(2) = support_perp(rotated);
    j.col(h) = support_rotate(q[2], dqh);
    j.col(k) = support_rotate(q[2], dqk);
    return j;
}

inline Eigen::Vector4d thrust_support_contact_coordinates(
    const SupportQ9 &q, double radius)
{
    Eigen::Vector4d g=Eigen::Vector4d::Zero();
    for (int side=0;side<2;++side)
    {
        const int h=3+2*side,k=h+1;
        const Eigen::Vector2d knee=support_rotate(q[h],{-.29348091,-.06220095});
        const Eigen::Vector2d wheel=support_rotate(q[h]+q[k],{.28210870,-.19553796});
        const Eigen::Vector2d local=Eigen::Vector2d(.125,-.07)+knee+wheel;
        const Eigen::Vector2d center=Eigen::Vector2d(q[0],q[1])+support_rotate(q[2],local);
        const double absolute_wheel_angle=q[2]+q[h]+q[k]+q[7+side];
        g[2*side]=center.x()+radius*absolute_wheel_angle;
        g[2*side+1]=center.y()-radius;
    }
    return g;
}

inline SupportContactJacobian thrust_support_contact_jacobian(
    const SupportQ9 &q, double radius)
{
    SupportContactJacobian j = SupportContactJacobian::Zero();
    for (int side = 0; side < 2; ++side)
    {
        const auto center = thrust_support_wheel_center_jacobian(q, side);
        const int r = 2 * side, wr = 7 + side;
        j.row(r) = center.row(0);
        // Required convention: axle-forward velocity + R*absolute wheel rate = 0.
        j(r, 2) += radius; j(r, 3 + 2 * side) += radius;
        j(r, 4 + 2 * side) += radius; j(r, wr) += radius;
        j.row(r + 1) = center.row(1);
    }
    return j;
}

inline Eigen::Vector2d thrust_support_com(const SupportQ9 &q, double body_mass)
{
    const double total = body_mass + 8.0;
    Eigen::Vector2d sum = body_mass * Eigen::Vector2d(.13261282, .05396677);
    const Eigen::Vector2d hip_offset(.125, -.07);
    for (int side = 0; side < 2; ++side)
    {
        const int h = 3 + 2 * side, k = h + 1;
        const auto thigh = support_rotate(q[h], {-.13690699, -.02116697});
        const auto knee = support_rotate(q[h], {-.29348091, -.06220095});
        const auto shank = support_rotate(q[h] + q[k], {.11538205, -.08532288});
        const auto wheel = support_rotate(q[h] + q[k], {.28210870, -.19553796});
        sum += 1.20 * (hip_offset + thigh);
        sum += .80 * (hip_offset + knee + shank);
        sum += 2.0 * (hip_offset + knee + wheel);
    }
    return Eigen::Vector2d(q[0], q[1]) + support_rotate(q[2], sum / total);
}

inline SupportTaskJacobian thrust_support_com_jacobian(
    const SupportQ9 &q, double body_mass)
{
    SupportTaskJacobian j = SupportTaskJacobian::Zero();
    const Eigen::Vector2d c = thrust_support_com(q, body_mass) - Eigen::Vector2d(q[0], q[1]);
    j(0, 0) = 1.; j(1, 1) = 1.;
    j.block<2,1>(0,2) = support_perp(c);
    constexpr double eps = 1e-6;
    for (int k = 3; k <= 6; ++k)
    {
        SupportQ9 qp=q, qm=q; qp[k]+=eps; qm[k]-=eps;
        const Eigen::Vector2d d=(thrust_support_com(qp,body_mass)-thrust_support_com(qm,body_mass))/(2*eps);
        j(0,k)=d.x(); j(1,k)=d.y();
    }
    j(2,2)=1.;
    return j;
}

inline Eigen::Vector2d thrust_support_wheel_center(
    const SupportQ9 &q, int side)
{
    const int h = 3 + 2 * side, k = h + 1;
    const Eigen::Vector2d hip_offset(.125, -.07);
    const Eigen::Vector2d knee=support_rotate(q[h],{-.29348091,-.06220095});
    const Eigen::Vector2d wheel=support_rotate(q[h]+q[k],{.28210870,-.19553796});
    return Eigen::Vector2d(q[0],q[1])+support_rotate(q[2],hip_offset+knee+wheel);
}

inline double thrust_support_com_forward_from_mean_axle(
    const SupportQ9 &q, double body_mass)
{
    const auto com=thrust_support_com(q,body_mass);
    const double axle_forward=.5*(thrust_support_wheel_center(q,0).x()+
                                   thrust_support_wheel_center(q,1).x());
    return com.x()-axle_forward;
}

inline Eigen::Matrix<double, 1, 9> thrust_support_com_axle_relative_jacobian(
    const SupportQ9 &q, double body_mass)
{
    const auto com_j=thrust_support_com_jacobian(q,body_mass);
    return (com_j.row(0)-.5*(thrust_support_wheel_center_jacobian(q,0).row(0)+
                             thrust_support_wheel_center_jacobian(q,1).row(0))).eval();
}

inline SupportQ9 thrust_support_damping(const ThrustSupportDynamicsInput &in)
{
    SupportQ9 d = SupportQ9::Zero();
    for (int i = 3; i <= 6; ++i)
        d[i] = in.hip_knee_damping * in.v[i] +
               in.hip_knee_coulomb * (in.v[i] > 0. ? 1. : in.v[i] < 0. ? -1. : 0.);
    d[7] = in.wheel_damping * in.v[7];
    d[8] = in.wheel_damping * in.v[8];
    return d;
}

inline bool thrust_support_dynamics_model(
    const ThrustSupportDynamicsInput &in, ThrustSupportDynamicsModel &out)
{
    out = ThrustSupportDynamicsModel{};
    if (!in.q.allFinite() || !in.v.allFinite() || !std::isfinite(in.body_mass) ||
        in.body_mass <= 0. || !std::isfinite(in.wheel_radius) || in.wheel_radius <= 0. ||
        !std::isfinite(in.gravity) || in.gravity <= 0. ||
        !std::isfinite(in.friction_coefficient) || in.friction_coefficient < 0. ||
        !std::isfinite(in.prediction_dt) || in.prediction_dt <= 0. ||
        !std::isfinite(in.wheel_relative_acceleration_reference[0]) ||
        !std::isfinite(in.wheel_relative_acceleration_reference[1])) return false;
    for (double x : in.actuator_limits) if (!std::isfinite(x) || x <= 0.) return false;
    for (double x : in.joint_position_min) if (!std::isfinite(x)) return false;
    for (double x : in.joint_position_max) if (!std::isfinite(x)) return false;
    for (int i=0;i<4;++i) if (in.joint_position_min[i] >= in.joint_position_max[i]) return false;
    for (double x : in.actuator_speed_limits) if (!std::isfinite(x) || x <= 0.) return false;

    out.mass = thrust_support_mass_matrix(in.q, in.body_mass);
    if (!out.mass.allFinite() || Eigen::LLT<SupportMatrix9>(out.mass).info()!=Eigen::Success) return false;
    out.joint_damping = thrust_support_damping(in);
    out.contact_jacobian = thrust_support_contact_jacobian(in.q,in.wheel_radius);
    out.com = thrust_support_com(in.q,in.body_mass);
    out.com_jacobian = thrust_support_com_jacobian(in.q,in.body_mass);
    out.com_axle_relative_forward =
        thrust_support_com_forward_from_mean_axle(in.q,in.body_mass);
    out.com_axle_relative_forward_jacobian =
        thrust_support_com_axle_relative_jacobian(in.q,in.body_mass);
    for (int side=0;side<2;++side)
    {
        const int h=3+2*side,k=h+1;
        const Eigen::Vector2d knee=support_rotate(in.q[h],{-.29348091,-.06220095});
        const Eigen::Vector2d wheel=support_rotate(in.q[h]+in.q[k],{.28210870,-.19553796});
        const Eigen::Vector2d local=Eigen::Vector2d(.125,-.07)+knee+wheel;
        const Eigen::Vector2d center=Eigen::Vector2d(in.q[0],in.q[1])+support_rotate(in.q[2],local);
        const Eigen::Vector2d contact=center-Eigen::Vector2d(0.,in.wheel_radius);
        const Eigen::Vector2d arm=contact-out.com;
        const int ft=2*side, fn=ft+1;
        // x-axis moment in world coordinates: r_y F_z - r_z F_y.
        out.contact_hdot_jacobian[ft]=-arm.y();
        out.contact_hdot_jacobian[fn]=arm.x();
    }
    const double total_mass=in.body_mass+8.;
    out.gravity = total_mass*in.gravity*out.com_jacobian.row(1).transpose();

    // Base translation does not affect M. In this world-coordinate 9D form,
    // the theta derivative is nonzero because the translational coordinates
    // are not rotated into the body frame; theta and all four leg angles are
    // differentiated for the Christoffel velocity bias.
    Eigen::Matrix<double,9,9> dM[9];
    for (auto &d : dM) d.setZero();
    constexpr double eps=1e-4;
    for (int c=2;c<=6;++c)
    {
        SupportQ9 qp=in.q,qm=in.q; qp[c]+=eps; qm[c]-=eps;
        dM[c]=(thrust_support_mass_matrix(qp,in.body_mass)-
               thrust_support_mass_matrix(qm,in.body_mass))/(2*eps);
    }
    for (int i=0;i<9;++i)
        for (int j=0;j<9;++j)
            for (int k=0;k<9;++k)
                out.velocity_bias[i] += .5*(dM[k](i,j)+dM[j](i,k)-dM[i](j,k))*in.v[j]*in.v[k];

    const auto jplus=thrust_support_com_jacobian(in.q+eps*in.v,in.body_mass);
    const auto jminus=thrust_support_com_jacobian(in.q-eps*in.v,in.body_mass);
    out.com_bias=((jplus-jminus)/(2*eps))*in.v;
    const auto rplus=thrust_support_com_axle_relative_jacobian(in.q+eps*in.v,in.body_mass);
    const auto rminus=thrust_support_com_axle_relative_jacobian(in.q-eps*in.v,in.body_mass);
    out.com_axle_relative_forward_bias=
        (((rplus-rminus)/(2*eps))*in.v)(0,0);
    const auto cplus=thrust_support_contact_jacobian(in.q+eps*in.v,in.wheel_radius);
    const auto cminus=thrust_support_contact_jacobian(in.q-eps*in.v,in.wheel_radius);
    out.contact_bias=((cplus-cminus)/(2*eps))*in.v;

    // Decision x = [qdd(9), leg_tau(4), wheel_tau(2), contact_(t,n)L/R(4)].
    // Dynamics: M qdd + C + G + D = B tau + Jc' lambda.
    out.equality.block<9,9>(0,0)=out.mass;
    for (int i=0;i<4;++i) out.equality(3+i,9+i)=-1.;
    out.equality(7,13)=-1.; out.equality(8,14)=-1.;
    out.equality.block<9,4>(0,15)=-out.contact_jacobian.transpose();
    out.equality_rhs.head<9>()=-(out.velocity_bias+out.gravity+out.joint_damping);
    out.equality.block<4,9>(9,0)=out.contact_jacobian;
    out.equality_rhs.segment<4>(9)=-out.contact_bias;
    out.equality(13,7)=1.; out.equality_rhs[13]=in.wheel_relative_acceleration_reference[0];
    out.equality(14,8)=1.; out.equality_rhs[14]=in.wheel_relative_acceleration_reference[1];

    // Actuator effort box, unilateral normal load and point-contact friction
    // pyramid. No free contact couple/CoP variable is introduced.
    int row=0;
    for (int a=0;a<6;++a)
    {
        const int col=9+a;
        out.inequality(row,col)=1.; out.inequality_rhs[row++]=in.actuator_limits[a];
        out.inequality(row,col)=-1.; out.inequality_rhs[row++]=in.actuator_limits[a];
    }
    for (int side=0;side<2;++side)
    {
        const int t=15+2*side,n=t+1;
        out.inequality(row,n)=-1.; out.inequality_rhs[row++]=0.;
        out.inequality(row,t)=1.; out.inequality(row,n)=-in.friction_coefficient; out.inequality_rhs[row++]=0.;
        out.inequality(row,t)=-1.; out.inequality(row,n)=-in.friction_coefficient; out.inequality_rhs[row++]=0.;
    }
    const double dt=in.prediction_dt;
    for (int j=0;j<6;++j)
    {
        const int qcol=(j<4)?3+j:7+(j-4);
        const double vlo=-in.actuator_speed_limits[j], vhi=in.actuator_speed_limits[j];
        const double acoef=dt;
        out.inequality(row,qcol)=acoef; out.inequality_rhs[row++]=vhi-in.v[qcol];
        out.inequality(row,qcol)=-acoef; out.inequality_rhs[row++]=in.v[qcol]-vlo;
    }
    for (int j=0;j<4;++j)
    {
        const int qcol=3+j;
        const double acoef=.5*dt*dt;
        const double base=in.q[qcol]+dt*in.v[qcol];
        out.inequality(row,qcol)=acoef; out.inequality_rhs[row++]=in.joint_position_max[j]-base;
        out.inequality(row,qcol)=-acoef; out.inequality_rhs[row++]=base-in.joint_position_min[j];
    }
    out.inequality_count=row;
    out.valid=out.equality.allFinite()&&out.equality_rhs.allFinite()&&
              out.inequality.topRows(row).allFinite()&&out.inequality_rhs.head(row).allFinite()&&
              out.com_jacobian.allFinite()&&out.com_bias.allFinite()&&out.contact_bias.allFinite();
    out.valid = out.valid && out.com_axle_relative_forward_jacobian.allFinite() &&
                std::isfinite(out.com_axle_relative_forward_bias) &&
                std::isfinite(out.com_axle_relative_forward);
    return out.valid;
}

inline double support_vertical_acceleration_from_per_wheel_force(
    double per_wheel_normal_force, double total_mass,
    double gravity=kSupportGravity)
{
    if (!std::isfinite(per_wheel_normal_force) || per_wheel_normal_force < 0. ||
        !std::isfinite(total_mass) || total_mass <= 0. ||
        !std::isfinite(gravity) || gravity <= 0.)
        return std::numeric_limits<double>::quiet_NaN();
    return 2.0*per_wheel_normal_force/total_mass-gravity;
}

inline SupportTaskRow support_vertical_acceleration_row(
    const ThrustSupportDynamicsModel &model, double desired_acceleration,
    double &rhs)
{
    SupportTaskRow row=SupportTaskRow::Zero();
    rhs=std::numeric_limits<double>::quiet_NaN();
    if (!model.valid || !std::isfinite(desired_acceleration)) return row;
    row.head<9>()=model.com_jacobian.row(1);
    rhs=desired_acceleration-model.com_bias[1];
    return row;
}

inline SupportTaskRow support_forward_acceleration_row(
    const ThrustSupportDynamicsModel &model, double desired_acceleration,
    double &rhs)
{
    SupportTaskRow row=SupportTaskRow::Zero();
    rhs=std::numeric_limits<double>::quiet_NaN();
    if (!model.valid || !std::isfinite(desired_acceleration)) return row;
    row.head<9>()=model.com_jacobian.row(0);
    rhs=desired_acceleration-model.com_bias[0];
    return row;
}

inline SupportTaskRow support_com_axle_relative_forward_acceleration_row(
    const ThrustSupportDynamicsModel &model, double desired_acceleration,
    double &rhs)
{
    SupportTaskRow row=SupportTaskRow::Zero();
    rhs=std::numeric_limits<double>::quiet_NaN();
    if (!model.valid || !std::isfinite(desired_acceleration)) return row;
    row.head<9>()=model.com_axle_relative_forward_jacobian;
    rhs=desired_acceleration-model.com_axle_relative_forward_bias;
    return row;
}

inline SupportTaskRow support_body_pitch_acceleration_row(
    const ThrustSupportDynamicsModel &model,double desired_pitch_acceleration,double &rhs)
{
    SupportTaskRow row=SupportTaskRow::Zero();
    rhs=std::numeric_limits<double>::quiet_NaN();
    if(!model.valid||!std::isfinite(desired_pitch_acceleration))return row;
    row[2]=-1.0; // physical pitch_ddot = -theta_ddot
    rhs=desired_pitch_acceleration;return row;
}

inline SupportTaskRow support_centroidal_hdot_row(
    const ThrustSupportDynamicsModel &model, double desired_hdot,
    double &rhs)
{
    SupportTaskRow row=SupportTaskRow::Zero();
    rhs=std::numeric_limits<double>::quiet_NaN();
    if (!model.valid || !std::isfinite(desired_hdot)) return row;
    row.segment<4>(15)=model.contact_hdot_jacobian;
    rhs=desired_hdot;
    return row;
}

inline double support_hdot_from_contact(
    const ThrustSupportDynamicsModel &model,
    const SupportContactVector &lambda)
{
    if (!model.valid || !lambda.allFinite()) return std::numeric_limits<double>::quiet_NaN();
    return model.contact_hdot_jacobian.dot(lambda);
}

inline ThrustSupportTaskResiduals evaluate_thrust_support_solution(
    const ThrustSupportDynamicsModel &model,
    const SupportDecisionVector &x,
    const ThrustSupportTasks &tasks)
{
    ThrustSupportTaskResiduals out;
    if (!model.valid || !x.allFinite() ||
        !std::isfinite(tasks.desired_com_vertical_acceleration) ||
        !std::isfinite(tasks.desired_com_forward_acceleration) ||
        !std::isfinite(tasks.desired_com_axle_relative_forward_acceleration) ||
        !std::isfinite(tasks.desired_centroidal_hdot) ||
        (tasks.body_pitch_enabled && !std::isfinite(tasks.desired_body_pitch_acceleration))) return out;
    const SupportQ9 qdd=x.head<9>();
    out.max_dynamics_residual=(model.equality.topRows(9)*x-model.equality_rhs.head<9>()).cwiseAbs().maxCoeff();
    out.max_contact_acceleration_residual=(model.equality.middleRows(9,4)*x-model.equality_rhs.segment<4>(9)).cwiseAbs().maxCoeff();
    out.vertical_acceleration=(model.com_jacobian.row(1).dot(qdd)+model.com_bias[1])-tasks.desired_com_vertical_acceleration;
    out.forward_acceleration=(model.com_jacobian.row(0).dot(qdd)+model.com_bias[0])-tasks.desired_com_forward_acceleration;
    out.com_axle_relative_forward_acceleration=
        (model.com_axle_relative_forward_jacobian.dot(qdd)+
         model.com_axle_relative_forward_bias)-
        tasks.desired_com_axle_relative_forward_acceleration;
    out.body_pitch_acceleration=-qdd[2]-tasks.desired_body_pitch_acceleration;
    out.centroidal_hdot=model.contact_hdot_jacobian.dot(x.segment<4>(15))-
                         tasks.desired_centroidal_hdot;
    double violation=0.;
    for (int i=0;i<model.inequality_count;++i)
        violation=std::max(violation,model.inequality.row(i).dot(x)-model.inequality_rhs[i]);
    out.max_inequality_violation=std::max(0.,violation);
    out.valid=std::isfinite(out.max_dynamics_residual)&&
              std::isfinite(out.max_contact_acceleration_residual)&&
              std::isfinite(out.vertical_acceleration)&&
              std::isfinite(out.centroidal_hdot)&&
              std::isfinite(out.body_pitch_acceleration)&&
              std::isfinite(out.forward_acceleration)&&
              std::isfinite(out.com_axle_relative_forward_acceleration)&&
              std::isfinite(out.max_inequality_violation);
    return out;
}
} // namespace bbot_jump
