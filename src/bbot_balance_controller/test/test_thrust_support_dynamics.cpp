#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "bbot_balance_controller/thrust_support_dynamics.hpp"

namespace {
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}
double max_violation(const bbot_jump::ThrustSupportDynamicsModel &m,
                     const bbot_jump::SupportDecisionVector &x)
{
    return (m.inequality.topRows(m.inequality_count)*x-
            m.inequality_rhs.head(m.inequality_count)).maxCoeff();
}
}

int main()
{
    using namespace bbot_jump;
    ThrustSupportDynamicsInput in;
    in.q << .17,.39,.11,.62,-.83,.54,-.71,.20,-.13;
    in.v << .31,.07,-.18,4.2,-7.1,3.7,-6.4,2.3,-1.8;
    in.wheel_relative_acceleration_reference={12.0,-8.0};
    ThrustSupportDynamicsModel m;
    require(thrust_support_dynamics_model(in,m),"valid 9D support state rejected");
    require(m.inequality_count==38,"hard-constraint set has wrong row count");
    require((m.mass-m.mass.transpose()).cwiseAbs().maxCoeff()<1e-12,
            "9D mass matrix is not symmetric");
    Eigen::LLT<SupportMatrix9> llt(m.mass);
    require(llt.info()==Eigen::Success,"9D mass matrix is not positive definite");

    JointVector q4(in.q[3],in.q[4],in.q[5],in.q[6]);
    const auto legacy=flight_floating_mass_matrix(q4,in.body_mass);
    SupportMatrix9 expected=SupportMatrix9::Zero();
    expected.topLeftCorner<7,7>()=legacy;
    for (int side=0;side<2;++side)
    {
        Eigen::Matrix<double,9,1> rotor=Eigen::Matrix<double,9,1>::Zero();
        const int h=3+2*side,k=h+1,wr=7+side;
        rotor[2]=rotor[h]=rotor[k]=rotor[wr]=1.;
        expected.noalias()+=kSupportWheelAxialInertia*rotor*rotor.transpose();
    }
    Eigen::Matrix<double,9,9> body_to_world=Eigen::Matrix<double,9,9>::Identity();
    const double c=std::cos(in.q[2]),s=std::sin(in.q[2]);
    body_to_world(0,0)=c;body_to_world(0,1)=-s;
    body_to_world(1,0)=s;body_to_world(1,1)=c;
    const auto world_to_body=body_to_world.inverse();
    expected=world_to_body.transpose()*expected*world_to_body;
    require((m.mass-expected).cwiseAbs().maxCoeff()<1e-10,
            "world 9D matrix differs from rotated production CAD plus wheel rotors");

    // Contact Jacobian must differentiate wheel-center motion plus the signed
    // absolute wheel spin used by axle_forward_velocity + R*omega_abs = 0.
    constexpr double eps=1e-7;
    const auto gp=thrust_support_contact_coordinates(in.q+eps*in.v,in.wheel_radius);
    const auto gm=thrust_support_contact_coordinates(in.q-eps*in.v,in.wheel_radius);
    const Eigen::Vector4d finite=(gp-gm)/(2*eps);
    require((finite-m.contact_jacobian*in.v).cwiseAbs().maxCoeff()<2e-7,
            "contact Jacobian sign/absolute wheel-rate convention mismatch");
    SupportQ9 rolling=in.v;
    for (int side=0;side<2;++side)
    {
        const auto center=thrust_support_wheel_center_jacobian(in.q,side);
        const int h=3+2*side,k=h+1,wr=7+side;
        const double axle_forward=center.row(0).dot(rolling);
        rolling[wr]=-(axle_forward/in.wheel_radius+rolling[2]+rolling[h]+rolling[k]);
    }
    const Eigen::Vector4d rolling_residual=thrust_support_contact_jacobian(in.q,in.wheel_radius)*rolling;
    require(std::abs(rolling_residual[0])<1e-12 &&
            std::abs(rolling_residual[2])<1e-12,
            "wheel relative rate was not converted to absolute rolling rate");

    // Gravity must sum to total mass*g in the base-z coordinate. Joint gravity
    // terms are CAD COM-Jacobian terms, not guessed knee-only compensation.
    require(std::abs(m.gravity[1]-(in.body_mass+8.)*in.gravity)<1e-10,
            "base vertical gravity term does not equal total weight");
    require(m.gravity.allFinite() && m.velocity_bias.allFinite() &&
            m.contact_bias.allFinite(),"nonfinite dynamic bias");
    const double expected_leg_drag=in.hip_knee_damping*in.v[3]+in.hip_knee_coulomb;
    require(std::abs(m.joint_damping[3]-expected_leg_drag)<1e-12 &&
            std::abs(m.joint_damping[7]-in.wheel_damping*in.v[7])<1e-12,
            "URDF damping/friction convention mismatch");

    // The H-dot task is only the moment of explicit wheel-bottom point forces;
    // no free contact couple is available to the decision vector.
    SupportContactVector lambda; lambda << 35.,180.,-20.,210.;
    const double hdot=support_hdot_from_contact(m,lambda);
    require(std::isfinite(hdot),"contact Hdot evaluation failed");
    double hdot_independent=0.;
    for (int side=0;side<2;++side)
    {
        const auto centerJ=thrust_support_wheel_center_jacobian(in.q,side);
        (void)centerJ;
        const double dy=m.contact_hdot_jacobian[2*side+1];
        const double minus_dz=m.contact_hdot_jacobian[2*side];
        hdot_independent += dy*lambda[2*side+1]+minus_dz*lambda[2*side];
    }
    require(std::abs(hdot-hdot_independent)<1e-12,
            "Hdot must arise from contact-force lever arms only");

    // The existing wheel path supplies a reference, not a measured acceleration.
    // The equality computes the torque required if its reference were tracked.
    require(m.equality(13,7)==1. && m.equality_rhs[13]==12.0 &&
            m.equality(14,8)==1. && m.equality_rhs[14]==-8.0,
            "wheel command acceleration not fixed in dynamics constraints");
    require(m.equality.block<9,9>(0,0).isApprox(m.mass,0.),
            "full 9D equations of motion not assembled");
    require(m.equality.block<3,6>(0,9).isZero(0.),
            "internal actuators must not apply directly to floating-base rows");

    SupportQ9 qdd; qdd << -.4,.8,.7,12.,-18.,-9.,14.,22.,-17.;
    const Eigen::Vector2d base_dynamics=(m.mass*qdd+m.velocity_bias).head<2>();
    const Eigen::Vector2d com_acc=(m.com_jacobian.topRows<2>()*qdd+
                                   m.com_bias.head<2>())*(in.body_mass+8.);
    require((base_dynamics-com_acc).cwiseAbs().maxCoeff()<5e-5,
            "base momentum equations do not close against CAD COM acceleration");

    SupportDecisionVector x=SupportDecisionVector::Zero();
    require(max_violation(m,x)<=1e-12,"zero torque/force violates box or friction inequalities");
    x[15]=100.; x[16]=100.; x[17]=-100.; x[18]=100.;
    require(max_violation(m,x)<=1e-12,"friction-cone boundary should be feasible");
    x[15]=101.;
    require(std::abs(max_violation(m,x)-1.)<1e-12,
            "friction-cone violation not exposed to task-priority solver");

    const double a_z=support_vertical_acceleration_from_per_wheel_force(170.,17.5);
    require(std::abs(a_z-(340./17.5-9.81))<1e-12,
            "per-wheel normal force conversion has wrong mass/sign");
    double rhs=0.;
    const auto vertical_row=support_vertical_acceleration_row(m,.5,rhs);
    require(vertical_row.allFinite() && std::abs(rhs-(.5-m.com_bias[1]))<1e-12,
            "vertical acceleration task row lost its kinematic bias");

    // The support-line task subtracts the mean moving axle. Its acceleration
    // row must match the second derivative of COM-forward minus axle-forward,
    // and must differ from world COM acceleration for rolling motion.
    const auto rel_row=support_com_axle_relative_forward_acceleration_row(m,.25,rhs);
    require(rel_row.allFinite() &&
            std::abs(rhs-(.25-m.com_axle_relative_forward_bias))<1e-12,
            "relative support-line acceleration task lost its bias");
    const double direction_step=1e-5;
    const double rel_plus=thrust_support_com_forward_from_mean_axle(
        in.q+direction_step*in.v,in.body_mass);
    const double rel_minus=thrust_support_com_forward_from_mean_axle(
        in.q-direction_step*in.v,in.body_mass);
    const double rel_velocity_fd=(rel_plus-rel_minus)/(2.*direction_step);
    require(std::abs(rel_velocity_fd-
                     m.com_axle_relative_forward_jacobian.dot(in.v))<2e-7,
            "relative support-line Jacobian does not match moving-axle geometry");
    const double trajectory_step=2e-4;
    const auto q_future=in.q+trajectory_step*in.v+
                        .5*trajectory_step*trajectory_step*qdd;
    const auto q_past=in.q-trajectory_step*in.v+
                       .5*trajectory_step*trajectory_step*qdd;
    const double rel_accel_fd=(thrust_support_com_forward_from_mean_axle(q_future,in.body_mass)-
                               2.*m.com_axle_relative_forward+
                               thrust_support_com_forward_from_mean_axle(q_past,in.body_mass))/
                              (trajectory_step*trajectory_step);
    const double rel_accel_model=
        m.com_axle_relative_forward_jacobian.dot(qdd)+
        m.com_axle_relative_forward_bias;
    require(std::abs(rel_accel_fd-rel_accel_model)<2e-5,
            "relative support-line acceleration bias does not close by finite difference");
    require(std::abs(rel_accel_model-
                     (m.com_jacobian.row(0).dot(qdd)+m.com_bias[0]))>1e-3,
            "moving axle was accidentally treated as a fixed world origin");
    ThrustSupportTasks task_targets;
    task_targets.desired_com_axle_relative_forward_acceleration=.25;
    task_targets.desired_com_forward_acceleration=.0;
    const auto evaluated=evaluate_thrust_support_solution(m,
        SupportDecisionVector::Zero(),task_targets);
    require(evaluated.valid &&
            std::abs(evaluated.com_axle_relative_forward_acceleration-
                     (m.com_axle_relative_forward_bias-.25))<1e-12,
            "task residual does not report the axle-relative task explicitly");

    auto invalid=in; invalid.friction_coefficient=-.1;
    require(!thrust_support_dynamics_model(invalid,m),"negative friction accepted");
    invalid=in; invalid.q[8]=std::numeric_limits<double>::quiet_NaN();
    require(!thrust_support_dynamics_model(invalid,m),"nonfinite wheel state accepted");
    std::cout << "9D CAD/contact dynamics, rolling signs, damping, and hard feasible-set rows passed\n";
}
