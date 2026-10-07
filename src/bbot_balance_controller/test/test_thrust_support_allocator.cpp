#include <iostream>
#include <limits>
#include <stdexcept>
#include "bbot_balance_controller/thrust_support_allocator.hpp"

using namespace bbot_jump;
static void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
static ThrustSupportAllocationInput fixture() {
    ThrustSupportAllocationInput in;
    auto& d=in.dynamics;
    d.q << 0.,0.,0.,.55,-.70,.55,-.70,0.,0.;
    const Eigen::Vector2d arm=thrust_support_com(d.q,d.body_mass)-thrust_support_wheel_center(d.q,0);
    d.q[2]=std::atan2(arm.x(),arm.y());
    d.q[1]=d.wheel_radius-thrust_support_wheel_center(d.q,0).y();
    d.actuator_limits={75.,60.,75.,60.,10.,10.};
    d.actuator_speed_limits={11.,15.,11.,15.,30.,30.};
    d.joint_position_min={-1.52,-1.56,-1.52,-1.56};
    d.joint_position_max={1.52,1.56,1.52,1.56};
    in.wheel_mode=SupportWheelMode::FixedMotorEffort;
    return in;
}
static void hard_check(const ThrustSupportAllocation& out) {
    if(!out.valid) std::cerr<<"allocation: "<<out.reason<<" slack="<<out.phase_one_slack<<'\n';
    require(out.valid,"support allocation rejected feasible fixture");
    require(out.maximum_equality_residual<1e-6,"dynamics/contact/wheel/task equalities violated");
    require(out.residuals.max_inequality_violation<1e-6,"hard inequalities violated");
    for(int s=0;s<2;++s) {
        require(out.decision[16+2*s]>=-1e-7,"negative normal force");
        require(std::abs(out.decision[15+2*s])<=out.decision[16+2*s]+1e-7,"friction cone violated");
    }
}
int main() {
    ThrustSupportTasks reference;
    require(make_thrust_support_task_reference(85.8375,17.5,0.20,-0.08,0.12,
            0.0,0.0,reference),"valid support task reference rejected");
    require(std::abs(reference.desired_com_vertical_acceleration)<1e-9,
            "per-wheel vertical force conversion has wrong total-force factor");
    require(std::abs(reference.desired_centroidal_hdot+2.333333333333333)<1e-9,
            "theta-axis Hdot sign/unit conversion changed");
    require(std::abs(reference.desired_com_axle_relative_forward_acceleration-1.0)<1e-9,
            "relative COM acceleration reference is wrong");
    require(make_thrust_support_task_reference(85.8375,17.5,0.20,-0.08,0.12,
            -1.0,0.0,reference) &&
            std::abs(reference.desired_com_axle_relative_forward_acceleration-20.0)<1e-9,
            "relative COM acceleration upper clamp failed");
    require(!make_thrust_support_task_reference(85.8375,17.5,
            std::numeric_limits<double>::quiet_NaN(),-0.08,0.12,0.0,0.0,reference),
            "nonfinite H was accepted as a task reference");
    require(!make_thrust_support_task_reference(85.8375,17.5,0.20,-0.08,0.0,
            0.0,0.0,reference),"zero Hdot horizon was accepted");
    auto in=fixture();
    auto out=allocate_thrust_support(in); hard_check(out);
    require(std::abs(out.residuals.vertical_acceleration)<1e-4,"static vertical task missed");
    require(std::abs(out.residuals.centroidal_hdot)<1e-4,"static Hdot task missed");
    require(std::abs(out.decision[13])<1e-7 && std::abs(out.decision[14])<1e-7,
            "fixed zero wheel motor effort was optimized away");
    // Real static inverse balance is independently constructed from gravity
    // and vertical wheel-bottom point forces, not from allocator output.
    SupportContactVector force;force<<0.,(in.dynamics.body_mass+8.)*9.81/2.,0.,(in.dynamics.body_mass+8.)*9.81/2.;
    SupportDecisionVector static_x=SupportDecisionVector::Zero();static_x.tail<4>()=force;
    const SupportQ9 needed=out.model.gravity-out.model.contact_jacobian.transpose()*force;
    static_x.segment<6>(9)=needed.segment<6>(3);
    require((out.model.equality*static_x-out.model.equality_rhs).cwiseAbs().maxCoeff()<1e-7,
            "independent static force/motor balance does not close");

    auto changed=in;
    changed.tasks.desired_centroidal_hdot=1000.;
    changed.tasks.desired_com_axle_relative_forward_acceleration=-1000.;
    auto conflict=allocate_thrust_support(changed);hard_check(conflict);
    require(std::abs(conflict.residuals.vertical_acceleration-out.residuals.vertical_acceleration)<1e-6,
            "lower tasks disturbed frozen vertical task");
    require(std::abs(conflict.residuals.centroidal_hdot)>1.,"unachievable task hidden");
    auto lower=changed;lower.tasks.desired_com_axle_relative_forward_acceleration=1000.;
    auto opposite=allocate_thrust_support(lower);hard_check(opposite);
    require(std::abs(opposite.achieved_task_lhs[1]-conflict.achieved_task_lhs[1])<1e-6,
            "COM task disturbed frozen Hdot task");

    changed=in;changed.tasks.desired_com_vertical_acceleration=1e5;
    auto unreachable=allocate_thrust_support(changed);hard_check(unreachable);
    require(std::abs(unreachable.residuals.vertical_acceleration)>1.,"unachievable vertical task hidden");

    changed=in;changed.dynamics.wheel_relative_acceleration_reference={123.,-987.};
    auto irrelevant=allocate_thrust_support(changed);hard_check(irrelevant);
    require((irrelevant.decision-out.decision).cwiseAbs().maxCoeff()<1e-7,
            "fixed motor mode still prescribes wheel acceleration");
    changed=in;changed.wheel_mode=SupportWheelMode::ObservedAccelerationReplay;
    changed.dynamics.wheel_relative_acceleration_reference={20.,-10.};
    auto observed=allocate_thrust_support(changed);hard_check(observed);
    require(std::abs(observed.decision[7]-20.)<1e-7 && std::abs(observed.decision[8]+10.)<1e-7,
            "observed wheel acceleration condition not enforced");
    changed=in;changed.dynamics.q[3]+=.01;changed.dynamics.q[5]-=.01;
    auto asymmetric=allocate_thrust_support(changed);hard_check(asymmetric);
    changed=in;changed.wheel_mode=SupportWheelMode::BoundedWheelServo;
    changed.dynamics.v[7]=-.3;changed.dynamics.v[8]=-.6;
    changed.tasks.desired_centroidal_hdot=.2;
    auto servo=allocate_thrust_support(changed);hard_check(servo);
    std::array<double,2> reconstructed_effort;
    require(support_wheel_servo_effort(servo.allocated_linear_command,0.,
        {changed.dynamics.v[7],changed.dynamics.v[8]},1.,true,true,false,reconstructed_effort),
        "allocated reference rejected by actual wheel servo");
    require(std::abs(reconstructed_effort[0]-servo.decision[13])<1e-6 &&
            std::abs(reconstructed_effort[1]-servo.decision[14])<1e-6,
            "allocated wheel effort cannot be executed by speed servo");
    require(std::abs(servo.allocated_linear_command)<=2.+1e-7,"linear command cap exceeded");

    // Real native THRUST state at 8.803s, including tiny left/right
    // asymmetry. After freezing COM vertical acceleration and H-dot, the
    // relative COM task is numerically dependent; amplifying its projection
    // previously made the active set cycle despite a feasible hard set.
    changed=in;
    changed.dynamics.q << .9967254396348842,.35040610848949305,-.10524827622305476,
        -.1391677303562973,.19819495033361545,-.1391677358567995,.19819494974273574,
        -14.053618734764864,-14.053618734764864;
    changed.dynamics.v << .3759406006364942,-.05837498031009171,.08191071111093151,
        .6568806460720719,.25415076103883727,.6568806209411403,.2541507619980205,
        -9.30489581520129,-9.30489581520129;
    changed.dynamics.prediction_dt=.010;
    changed.dynamics.actuator_limits={150.,150.,150.,150.,10.,10.};
    changed.fixed_wheel_motor_effort={1.1428529580584321,1.1428529580584321};
    changed.tasks.desired_com_vertical_acceleration=4.2138985157836404;
    changed.tasks.desired_centroidal_hdot=0.;
    ThrustSupportDynamicsModel replay_model;
    require(thrust_support_dynamics_model(changed.dynamics,replay_model),"native fixture invalid");
    changed.tasks.desired_com_axle_relative_forward_acceleration=
        std::clamp(100.*(.010-replay_model.com_axle_relative_forward)-
            10.*replay_model.com_axle_relative_forward_jacobian.dot(changed.dynamics.v),-20.,20.);
    auto native=allocate_thrust_support(changed);hard_check(native);
    require(std::abs(native.residuals.vertical_acceleration)<1e-4 &&
            std::abs(native.residuals.centroidal_hdot)<1e-4,
            "nearly dependent lower task disturbed higher native tasks");

    changed=in;changed.dynamics.actuator_limits={1e-6,1e-6,1e-6,1e-6,10.,10.};
    changed.dynamics.actuator_speed_limits={1e-4,1e-4,1e-4,1e-4,1e-4,1e-4};
    require(!allocate_thrust_support(changed).valid,"truly infeasible hard set accepted");
    changed=in;changed.wheel_mode=SupportWheelMode::Unspecified;
    require(!allocate_thrust_support(changed).valid,"unspecified wheel actuation accepted");
    changed=in;changed.fixed_wheel_motor_effort={10.01,0.};
    require(!allocate_thrust_support(changed).valid,"wheel effort beyond actual servo limit accepted");
    changed=in;changed.dynamics.q[0]=std::numeric_limits<double>::quiet_NaN();
    require(!allocate_thrust_support(changed).valid,"nonfinite state accepted");
    changed=in;changed.tasks.desired_centroidal_hdot=std::numeric_limits<double>::infinity();
    require(!allocate_thrust_support(changed).valid,"nonfinite task accepted");

    std::array<double,2> effort;
    require(support_wheel_servo_effort(-2.,0.,{3.,-4.},1.,true,true,false,effort),"valid wheel servo input rejected");
    require(effort[0]==-10. && effort[1]==-10.,"actual 10Nm effort cap absent");
    require(support_wheel_servo_effort(.1,0.,{0.,0.},1.,false,true,false,effort) && effort[0]==0.,
            "stale wheel command still produces torque");
    require(support_wheel_servo_effort(.1,0.,{0.,0.},1.,true,true,true,effort) && effort[0]==0.,
            "zero-effort request not respected");
    std::cout<<"Support hierarchy, rigid point contacts, actual wheel-servo effort, limits and rejection checks passed\n";
}
