#pragma once
// Private normal-speed flight plan initialized from an aligned airborne q/v/H.
// Whole-body momentum predicts torso motion; wheel clearance is evaluated from
// ballistic COM minus the actual CAD COM-to-each-axle distance.
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/flight_landing_geometry.hpp"
#include "bbot_balance_controller/landing_repair_momentum.hpp"
#include "bbot_balance_controller/thrust_support_dynamics.hpp"
namespace bbot_jump {
struct AllocatorFlightPlan {
  bool valid=false;
  const char *reason="not_initialized";
  double start=0., duration=0., tuck_duration=.06, deploy_start=0.;
  double initial_H=0., initial_pitch=0., initial_wheel=0., final_wheel=0., wheel_amplitude=0.;
  double mass=9.5, com_z0=0., com_vz0=0., ground=0., peak_clearance=0., predicted_contact_pitch=0.;
  LandingPoseTarget landing;
  std::array<QuinticTrajectory,4> tuck,deploy;
  JointVector sample(double t,JointVector &v,JointVector &a) const {
    JointVector q;
    for(int i=0;i<4;++i) {
      const auto &tr=(t<start+deploy_start)?tuck[i]:deploy[i];
      tr.evaluate(t,q[i],v[i],a[i]);
      if(t>=start+tuck_duration && t<start+deploy_start) {v[i]=0.;a[i]=0.;}
    }
    return q;
  }
  double wheel(double relative_time,double *acc=nullptr) const {
    const double s=std::clamp(relative_time/duration,0.,1.);
    const double blend=s*s*(3.-2.*s),dblend=6.*s*(1.-s)/duration;
    const double z=std::sin(M_PI*s);
    if(acc)*acc=(final_wheel-initial_wheel)*dblend+wheel_amplitude*M_PI*std::sin(2*M_PI*s)/duration;
    return initial_wheel+(final_wheel-initial_wheel)*blend+wheel_amplitude*z*z;
  }
};
// Required six inputs for a reference flight curve. Solve the free base,
// then recover motor efforts (passive damping is separate from motor input).
inline bool allocator_flight_curve_effort(const JointVector &q,const JointVector &v,
    const JointVector &leg_acc,double pitch,double pitch_rate,double w,double wa,
    double mass,SupportActuatorVector &effort) {
  ThrustSupportDynamicsInput d;d.body_mass=mass;d.q[2]=-pitch;d.v[2]=-pitch_rate;
  d.q.segment<4>(3)=q;d.v.segment<4>(3)=v;d.v[7]=w;d.v[8]=w;
  ThrustSupportDynamicsModel m;if(!thrust_support_dynamics_model(d,m))return false;
  const SupportQ9 passive=m.velocity_bias+m.gravity+m.joint_damping;
  SupportQ9 a=SupportQ9::Zero();a.segment<4>(3)=leg_acc;a[7]=wa;a[8]=wa;
  const Eigen::Vector3d rhs=-passive.head<3>()-m.mass.block<3,6>(0,3)*a.tail<6>();
  a.head<3>()=m.mass.block<3,3>(0,0).ldlt().solve(rhs);
  effort=(m.mass*a+passive).tail<6>();return effort.allFinite();
}
inline AllocatorFlightPlan make_allocator_flight_plan(double stamp,const JointVector &q,
    const JointVector &v,const std::array<double,2>& wheel,double pitch,double H,
    double com_z,double com_vz,double ground,double mass,const bbot_kinematics::RobotParams &params,
    const JointVector &initial_acceleration=JointVector::Zero(),bool adaptive_capture=false) {
  AllocatorFlightPlan out;
  if(!std::isfinite(stamp)||stamp<=0.||!std::isfinite(ground)||
     !std::isfinite(mass)||mass<=0.||std::abs(wheel[0]-wheel[1])>.5||
     !q.allFinite()||!v.allFinite()||!initial_acceleration.allFinite()||!std::isfinite(H)||!std::isfinite(pitch)||
     !std::isfinite(com_z)||!std::isfinite(com_vz)||!std::isfinite(wheel[0])||!std::isfinite(wheel[1]))return out;
  out.start=stamp;out.initial_H=H;out.initial_pitch=pitch;out.mass=mass;
  out.com_z0=com_z;out.com_vz0=com_vz;out.ground=ground;
  constexpr double landing_pitch=2.5*M_PI/180.;
  out.landing=solve_landing_pose_for_com_forward(params,.52,landing_pitch,.010,mass);
  auto mid=solve_leg_pose_for_com_forward(params,.44,landing_pitch,.010,mass);
  const JointVector stopping_pose=q+.5*out.tuck_duration*v+
      (out.tuck_duration*out.tuck_duration/12.)*initial_acceleration;
  if(!out.landing.valid||!mid.valid){out.reason="CAD_pose_unreachable";return out;}
  const auto lg=centroidal_geometry({out.landing.hip,out.landing.knee,out.landing.hip,out.landing.knee},mass);
  const double contact_z=ground+.07+rotate_about_hip(-landing_pitch,lg.com-lg.axle).z();
  const double discriminant=com_vz*com_vz+2.*9.81*(com_z-contact_z);
  if(discriminant<=0.){out.reason="no_ballistic_contact_time";return out;}
  out.duration=(com_vz+std::sqrt(discriminant))/9.81;
  const double deploy_duration=.12;
  out.deploy_start=out.duration-deploy_duration;
  if(out.deploy_start<out.tuck_duration+.015||out.duration>.65){out.reason="insufficient_ballistic_time";return out;}
  for(int i=0;i<4;++i) {
    const double qm=adaptive_capture?stopping_pose[i]:(i%2?mid.knee:mid.hip), qe=i%2?out.landing.knee:out.landing.hip;
    out.tuck[i].init(stamp,out.tuck_duration,q[i],v[i],initial_acceleration[i],qm,0.,0.);
    out.deploy[i].init(stamp+out.deploy_start,deploy_duration,qm,0.,0.,qe,0.,0.);
  }
  // Keep all four requested curves available for diagnostics on rejection;
  // validity still requires every braking and deployment segment to pass.
  for(int i=0;i<4;++i) {
    if(!flight_trajectory_admissible(out.tuck[i],i%2?15.:11.,i%2?500.:450.,i%2?1.56:1.52)||
       !flight_trajectory_admissible(out.deploy[i],i%2?15.:11.,i%2?500.:450.,i%2?1.56:1.52)){
      out.reason="joint_curve_limit";return out;
    }
  }
  out.initial_wheel=.5*(wheel[0]+wheel[1]);
  // Zero endpoint joint rates and zero torso rate fix the wheel spin by H.
  out.final_wheel=H/(2.*kSupportWheelAxialInertia);
  if(std::abs(out.final_wheel)>2./.07){out.reason="endpoint_momentum_exceeds_wheel_authority";return out;}
  const int steps=static_cast<int>(std::ceil(out.duration/.002));
  const double dt=out.duration/steps;
  double pitch_nom=pitch,coefficient=0.;
  for(int k=0;k<steps;++k) {
    const double t=(k+.5)*dt;JointVector vv,aa;const auto qq=out.sample(stamp+t,vv,aa);
    const auto hm=landing_momentum(qq,vv,{0.,0.},0.,mass);
    const double w=out.wheel(t);
    pitch_nom+=(hm.joint_rate_term+2.*kSupportWheelAxialInertia*w-H)/hm.pitch_inertia*dt;
    coefficient+=2.*kSupportWheelAxialInertia*std::pow(std::sin(M_PI*t/out.duration),2)/hm.pitch_inertia*dt;
  }
  if(coefficient<=1e-9){out.reason="invalid_momentum_projection";return out;}
  out.wheel_amplitude=(landing_pitch-pitch_nom)/coefficient;
  double predicted_pitch=pitch;
  out.peak_clearance=-1.;
  for(int k=0;k<=steps;++k) {
    const double t=k*dt;JointVector vv,aa;const auto qq=out.sample(stamp+t,vv,aa);
    double wheel_acc;const double w=out.wheel(t,&wheel_acc);
    const double rate=landing_pitch_rate_for_momentum(qq,vv,{w,w},H,mass);
    if(!std::isfinite(rate)||std::abs(w)>2./.07||std::abs(wheel_acc)>600.){
      out.reason="wheel_curve_limit";return out;
    }
    if(k) {
      JointVector vm,am;const auto qm=out.sample(stamp+t-.5*dt,vm,am);
      predicted_pitch+=landing_pitch_rate_for_momentum(qm,vm,{out.wheel(t-.5*dt),out.wheel(t-.5*dt)},H,mass)*dt;
    }
    if(std::abs(predicted_pitch)>.35){out.reason="torso_curve_limit";return out;}
    SupportActuatorVector motor;
    if(!allocator_flight_curve_effort(qq,vv,aa,predicted_pitch,rate,w,wheel_acc,mass,motor)) {
      out.reason="flight_inverse_dynamics_invalid";return out;
    }
    for(int j=0;j<6;++j) {
      const double cap=j<4?(j%2?60.:75.):10.;
      if(std::abs(motor[j])>cap+1e-6){out.reason="flight_motor_curve_limit";return out;}
    }
    // Bounded P servo must have room for the feedforward effort and tracking.
    if(std::abs(w+.5*(motor[4]+motor[5]))>2./.07) {
      out.reason="flight_servo_target_limit";return out;
    }
    const auto geom=centroidal_geometry({qq[0],qq[1],qq[2],qq[3]},mass);
    const double ballistic=com_z+com_vz*t-.5*9.81*t*t;
    double clearance=1e9;
    for(int side=0;side<2;++side) {
      const auto knee=support_rotate(qq[2*side],{-.29348091,-.06220095});
      const auto wheel_offset=support_rotate(qq[2*side]+qq[2*side+1],{.28210870,-.19553796});
      const Eigen::Vector3d axle(0.,.125+knee.x()+wheel_offset.x(),-.07+knee.y()+wheel_offset.y());
      clearance=std::min(clearance,ballistic+rotate_about_hip(-predicted_pitch,axle-geom.com).z()-.07-ground);
    }
    out.peak_clearance=std::max(out.peak_clearance,clearance);
    if(k>3&&k<steps-2&&clearance<-.003){out.reason="premature_ground_intersection";return out;}
  }
  out.predicted_contact_pitch=predicted_pitch;
  if(out.peak_clearance<.20||std::abs(predicted_pitch-landing_pitch)>.003){out.reason="clearance_or_contact_pose";return out;}
  out.valid=true;out.reason="feasible_momentum_curve";return out;
}

// In flight, solve base and wheel accelerations with prescribed leg qdd and
// actual bounded P-servo wheel effort, then recover the four actuator efforts.
inline bool allocator_flight_effort(const ThrustSupportDynamicsInput &state,const JointVector &leg_acc,
    const std::array<double,2>& wheel_effort,JointVector &leg_effort,SupportQ9 *full_acc=nullptr) {
  ThrustSupportDynamicsModel m;
  if(!leg_acc.allFinite()||!thrust_support_dynamics_model(state,m))return false;
  const std::array<int,5> free{0,1,2,7,8};
  Eigen::Matrix<double,5,5> mat;Eigen::Matrix<double,5,1> rhs;
  const SupportQ9 passive=m.velocity_bias+m.gravity+m.joint_damping;
  for(int i=0;i<5;++i) {
    for(int j=0;j<5;++j)mat(i,j)=m.mass(free[i],free[j]);
    rhs[i]=-passive[free[i]]-m.mass.block<1,4>(free[i],3).dot(leg_acc)+(i>=3?wheel_effort[i-3]:0.);
  }
  const auto solved=mat.ldlt().solve(rhs).eval();
  if(!solved.allFinite()||(mat*solved-rhs).cwiseAbs().maxCoeff()>1e-6)return false;
  SupportQ9 acc;acc.segment<4>(3)=leg_acc;
  for(int i=0;i<5;++i)acc[free[i]]=solved[i];
  leg_effort=(m.mass*acc+passive).segment<4>(3);
  if(full_acc)*full_acc=acc;
  return leg_effort.allFinite();
}
} // namespace bbot_jump
