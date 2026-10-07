#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/allocator_flight_plan.hpp"
using namespace bbot_jump;
void need(bool x,const char*s){if(!x)throw std::runtime_error(s);}
int main(){
 SupportQ9 q,v;
 q<<.4043808927,.5493585999,-.1094610444,.4595013142,-.5061506865,.4595006429,-.5061506714,-5.78290899,-5.78290835;
 v<<.3378302721,2.6974606175,.5529670872,6.5451964845,-10.8420422485,6.5450007804,-10.8420368935,-6.2111770174,-6.2109294864;
 const auto h=landing_momentum(q.segment<4>(3),v.segment<4>(3),{v[7],v[8]},-v[2],9.5);
 const auto com=thrust_support_com(q,9.5);const auto cj=thrust_support_com_jacobian(q,9.5);
 auto plan=[&](double H){return make_allocator_flight_plan(3.252,q.segment<4>(3),v.segment<4>(3),{v[7],v[8]},-q[2],H,com.y(),cj.row(1).dot(v),0.,9.5,bbot_kinematics::RobotParams());};
 need(!plan(h.total).valid,"unsafe real old takeoff momentum accepted");
 // Synthetic already-airborne fixture verifies the unchanged 60 ms tuck
 // timing and mechanics. It is not a measured takeoff or a jump result.
 const auto mid=solve_leg_pose_for_com_forward(bbot_kinematics::RobotParams(),.44,2.5*M_PI/180.,.010,9.5);
 JointVector q0(mid.hip,mid.knee,mid.hip,mid.knee),v0=JointVector::Zero();
 const auto p=make_allocator_flight_plan(3.252,q0,v0,{0.,0.},2.5*M_PI/180.,0.,.75,1.8,0.,9.5,bbot_kinematics::RobotParams());
 need(p.valid,"feasible unit fixture rejected");
 need(p.tuck_duration==.06,"normal tuck timing was slowed");
 need(p.peak_clearance>=.20,"per-wheel CAD clearance gate ignored");
 need(std::abs(p.predicted_contact_pitch-2.5*M_PI/180.)<.003,"contact pitch incorrect");
 JointVector vv,aa;auto qq=p.sample(p.start,vv,aa);
 need((qq-q0).norm()<1e-8&&(vv-v0).norm()<1e-8,"real initial q/v boundary lost");
 for(int k=0;k<=50;++k){
  const double t=p.duration*k/50.;qq=p.sample(p.start+t,vv,aa);double wa;const double w=p.wheel(t,&wa);
  SupportActuatorVector motor;
  need(allocator_flight_curve_effort(qq,vv,aa,.03,.1,w,wa,9.5,motor),"curve inverse dynamics failed");
  ThrustSupportDynamicsInput d;d.q[2]=-.03;d.v[2]=-.1;d.q.segment<4>(3)=qq;d.v.segment<4>(3)=vv;d.v[7]=w;d.v[8]=w;
  ThrustSupportDynamicsModel m;need(thrust_support_dynamics_model(d,m),"mass model failed");
  JointVector recovered;SupportQ9 full;
  need(allocator_flight_effort(d,aa,{motor[4],motor[5]},recovered,&full),"free flight inverse failed");
  need((recovered-motor.head<4>()).norm()<1e-7,"independent base/wheel elimination mismatch");
  SupportQ9 input=SupportQ9::Zero();input.tail<6>()=motor;
  need((m.mass*full+m.velocity_bias+m.gravity+m.joint_damping-input).norm()<1e-7,"motor dynamics residual");
 }
 std::cout<<"allocator flight admissibility and full motor/base dynamics PASS\n";
}
