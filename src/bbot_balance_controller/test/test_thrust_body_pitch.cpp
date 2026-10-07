#include <Eigen/QR>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "bbot_balance_controller/thrust_support_allocator.hpp"
using namespace bbot_jump;
void need(bool x,const char*s){if(!x)throw std::runtime_error(s);}
ThrustSupportAllocationInput fixture(){
 ThrustSupportAllocationInput i;auto &d=i.dynamics;
 d.q<<0.,0.,-.088,.55,-.70,.55,-.70,0.,0.;d.q[1]=.07-thrust_support_wheel_center(d.q,0).y();
 d.actuator_limits={150.,150.,150.,150.,10.,10.};d.actuator_speed_limits={11.,15.,11.,15.,30.,30.};
 d.joint_position_min={-1.52,-1.56,-1.52,-1.56};d.joint_position_max={1.52,1.56,1.52,1.56};d.prediction_dt=.010;
 i.wheel_mode=SupportWheelMode::BoundedWheelServo;i.tasks.desired_com_vertical_acceleration=3.;
 return i;
}
void check(const ThrustSupportAllocationInput&i,const ThrustSupportAllocation&o){
 if(!o.valid)std::cerr<<o.reason<<'\n';
 need(o.valid,"pitch allocation infeasible");
 need(o.maximum_equality_residual<1e-6&&o.residuals.max_inequality_violation<1e-6,"hard constraints violated");
 std::array<double,2> tau;need(support_wheel_servo_effort(o.allocated_linear_command,0.,{i.dynamics.v[7],i.dynamics.v[8]},1.,true,true,false,tau),"P servo invalid");
 // Independent forward response from all six recoverable motor inputs.
 Eigen::Matrix<double,13,13> A=Eigen::Matrix<double,13,13>::Zero();
 A.topLeftCorner<9,9>()=o.model.mass;A.topRightCorner<9,4>()=-o.model.contact_jacobian.transpose();A.bottomLeftCorner<4,9>()=o.model.contact_jacobian;
 Eigen::Matrix<double,13,1>b;b.head<9>()=-(o.model.velocity_bias+o.model.gravity+o.model.joint_damping);b.segment<4>(3)+=o.decision.segment<4>(9);b[7]+=tau[0];b[8]+=tau[1];b.tail<4>()=-o.model.contact_bias;
 const Eigen::Matrix<double,13,1>x=A.completeOrthogonalDecomposition().solve(b);
 need((x.head<9>()-o.decision.head<9>()).norm()<1e-6,"published motors do not reconstruct allocated accelerations");
 need(std::abs(-x[2]-i.tasks.desired_body_pitch_acceleration)<1e-4,"body pitch task was not realized");
}
int main(){
 ThrustSupportTasks t;
 need(make_thrust_body_pitch_reference(.108,0.,.088,.010,t)&&t.desired_body_pitch_acceleration<0.,"excess forward pitch needs corrective negative acceleration");
 need(make_thrust_body_pitch_reference(.068,0.,.088,.010,t)&&t.desired_body_pitch_acceleration>0.,"insufficient forward pitch correction reversed");
 need(make_thrust_body_pitch_reference(.088,.2,.088,.010,t)&&t.desired_body_pitch_acceleration<0.,"forward tipping rate not damped");
 need(make_thrust_body_pitch_reference(.088,-.2,.088,.010,t)&&t.desired_body_pitch_acceleration>0.,"backward tipping rate not damped");
 need(make_thrust_body_pitch_reference(.088,100.,.088,.010,t)&&t.desired_body_pitch_acceleration==-35.,"pitch acceleration cap absent");
 need(!make_thrust_body_pitch_reference(std::numeric_limits<double>::quiet_NaN(),0.,.088,.01,t),"invalid pitch accepted");
 auto i=fixture();i.tasks.body_pitch_enabled=true;i.tasks.desired_body_pitch_acceleration=-1.;auto o=allocate_thrust_support(i);check(i,o);
 auto opp=i;opp.tasks.desired_body_pitch_acceleration=1.;auto op=allocate_thrust_support(opp);check(opp,op);
 need(std::abs(o.decision[9]-op.decision[9])>.1,"hip actuator does not participate in pitch correction");
 auto conflict=i;conflict.tasks.desired_centroidal_hdot=1000.;conflict.tasks.desired_com_axle_relative_forward_acceleration=-1000.;auto c=allocate_thrust_support(conflict);check(conflict,c);
 need(std::abs(c.achieved_task_lhs[0]-o.achieved_task_lhs[0])<1e-6&&std::abs(c.achieved_task_lhs[1]-o.achieved_task_lhs[1])<1e-6,"H/COM disturbed vertical or pitch task");
 need(std::abs(c.residuals.centroidal_hdot)>1.,"unreachable Hdot was hidden");
 conflict.tasks.desired_com_axle_relative_forward_acceleration=1000.;auto c2=allocate_thrust_support(conflict);check(conflict,c2);
 need(std::abs(c2.achieved_task_lhs[2]-c.achieved_task_lhs[2])<1e-6,"relative COM disturbed Hdot");
 auto invalid=i;invalid.tasks.desired_body_pitch_acceleration=std::numeric_limits<double>::quiet_NaN();need(!allocate_thrust_support(invalid).valid,"nonfinite pitch task accepted");
 std::cout<<"Body-pitch sign/damping, hip participation, priority and reconstructed six-input dynamics PASS\n";
}
