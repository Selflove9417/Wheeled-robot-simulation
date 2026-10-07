#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include "bbot_balance_controller/ground_contact_motion_fast_certificate.hpp"

using namespace bbot_jump;
static std::ofstream witness;

static GroundContactFastCertificateInput make_hold() {
  GroundContactFastCertificateInput in;
  in.q << .67284161996328307,.50713710928565003,-.064389900807068995,
      .25520010829184914,-.36925458170020709,.25520010923480746,
      -.3692545821888305,-9.2668105162323808,-9.2668104601098005;
  in.v << -.0033648902344868129,5.7078255696882597e-05,
      -.00065758052184974175,-4.2008757583644751e-13,4.881373083520657e-13,
      -4.2010839251815923e-13,4.8798465268617974e-13,
      .052833907416522746,.052833907362268492;
  in.command << -.72345083572675517,-15.31390561244141,
      -.72345084855659703,-15.313905619364968,
      .0089971939605110185,.0089971940042035264;
  return in;
}

static GroundContactFastCertificateInput make_age4() {
  GroundContactFastCertificateInput in;
  in.q << .65881672026013416,.51199177778941907,-.060680581366889419,
      .26520497206377536,-.38925489170835548,.265204977940106,-.38925489151770176,
      -9.0656783425930314,-9.0656783205539373;
  in.v << -.002317450765813031,4.0038760744420541e-05,-.00041518951306884647,
      .00011519310753815741,1.8866597917510158e-17,
      .00011519308459181988,-5.7061825142311026e-17,
      .035428117664294649,.035428117760872818;
  in.command << -.7391546690797972,-15.32644108279416,
      -.7391546690797972,-15.32644108279416,
      .01081191159339033,.010811911496812161;
  in.command[0]=-.57675610852759163; in.command[1]=-15.168920635254469;
  in.command[2]=-.57675610852784132; in.command[3]=-15.16892063525467;
  return in;
}

static GroundFrictionResponseResult full81(const GroundContactFastCertificateInput &in) {
  GroundFrictionResponseInput fi;
  fi.q=in.q; fi.v=in.v; fi.command=in.command; fi.dt=in.dt;
  fi.bilateral_contact=in.bilateral_contact;
  fi.wheel_ground_friction=in.wheel_ground_friction;
  GroundFrictionResponseOptions fo;
  fo.response_profile=GroundUrdfResponseProfile::URDFGravityAndFixedIMU;
  return ground_friction_response(fi,fo);
}

static void compare(const GroundContactFastCertificateInput &in,const char *name,
                    bool allow_deadline_reject=false) {
  const auto fast=ground_contact_motion_fast_certificate(in);
  const auto ref=full81(in);
  std::cout<<name<<" fast="<<fast.valid<<":"<<fast.reason
      <<" ref="<<ref.valid<<":"<<ref.reason<<" modes="
      <<fast.leg_mode[0]<<","<<fast.leg_mode[1]<<","<<fast.leg_mode[2]<<","<<fast.leg_mode[3]
      <<" candidates="<<fast.candidate_count<<" feasible="<<fast.feasible_candidate_count
      <<" rank="<<fast.kkt_rank<<"/"<<fast.kkt_dimension
      <<" elapsed_us="<<fast.elapsed_us;
  if(fast.valid&&ref.valid) {
    const double da=(fast.acceleration-ref.acceleration).cwiseAbs().maxCoeff();
    const double df=(fast.contact_force-ref.contact_force).cwiseAbs().maxCoeff();
    const double dtau=(fast.leg_friction_torque-ref.leg_friction_torque).cwiseAbs().maxCoeff();
    std::cout<<" da="<<da<<" dcontact="<<df<<" dfriction="<<dtau
        <<" maxstatic="<<fast.max_stick_effort;
    assert(da<=1e-7);
    assert(df<=1e-6); // contact load may be nonunique only inside boundary witnesses.
    assert(dtau<=1e-7);
  }
  std::cout<<"\n";
  if(witness.good()) {
    witness<<name<<','<<in.dt<<','<<fast.valid<<','<<fast.reason<<','
      <<ref.valid<<','<<ref.reason<<','<<fast.candidate_count<<','
      <<fast.feasible_candidate_count<<','<<fast.kkt_rank<<','<<fast.kkt_dimension<<','
      <<fast.equation_residual<<','<<fast.contact_residual<<','<<fast.max_stick_effort<<','
      <<fast.max_contact_cone_violation<<','<<fast.elapsed_us<<',';
    for(int i=0;i<9;++i) witness<<in.q[i]<<',';
    for(int i=0;i<9;++i) witness<<in.v[i]<<',';
    for(int i=0;i<6;++i) witness<<in.command[i]<<',';
    for(int i=0;i<4;++i) witness<<fast.leg_mode[i]<<',';
    for(int i=0;i<9;++i) witness<<fast.acceleration[i]<<',';
    for(int i=0;i<4;++i) witness<<fast.contact_force[i]<<',';
    for(int i=0;i<4;++i) witness<<fast.leg_friction_torque[i]<<',';
    witness<<ref.kkt_rank<<','<<ref.max_equation_residual<<','
        <<ref.max_contact_residual<<','<<ref.max_stick_effort<<',';
    for(int i=0;i<4;++i) witness<<ref.leg_mode[i]<<',';
    for(int i=0;i<9;++i) witness<<ref.acceleration[i]<<',';
    for(int i=0;i<4;++i) witness<<ref.contact_force[i]<<',';
    for(int i=0;i<4;++i) witness<<ref.leg_friction_torque[i]<<',';
    if(fast.valid&&ref.valid)
      witness<<(fast.acceleration-ref.acceleration).cwiseAbs().maxCoeff()<<','
          <<(fast.contact_force-ref.contact_force).cwiseAbs().maxCoeff()<<','
          <<(fast.leg_friction_torque-ref.leg_friction_torque).cwiseAbs().maxCoeff();
    else witness<<"nan,nan,nan";
    witness<<'\n';
  }
  if(allow_deadline_reject) {
    assert(!fast.valid&&fast.reason=="fast_certificate_deadline_exceeded");
    assert(ref.valid);
    return;
  }
  assert(fast.valid==ref.valid);
  if(fast.valid) assert(fast.reason=="ok");
}

int main() {
  std::cout<<std::setprecision(17);
  const char *path=std::getenv("GROUND_FAST_WITNESS_CSV");
  if(path) {
    witness.open(path);
    assert(witness.good()); witness<<std::setprecision(17);
    witness<<"case,dt,fast_valid,fast_reason,full81_valid,full81_reason,candidates,feasible,kkt_rank,kkt_dimension,eq_residual,contact_residual,max_stick_effort,max_cone_violation,elapsed_us,";
    for(int i=0;i<9;++i) witness<<"q"<<i<<',';
    for(int i=0;i<9;++i) witness<<"v"<<i<<',';
    for(int i=0;i<6;++i) witness<<"u"<<i<<',';
    for(int i=0;i<4;++i) witness<<"mode"<<i<<',';
    for(int i=0;i<9;++i) witness<<"fast_a"<<i<<',';
    for(int i=0;i<4;++i) witness<<"fast_lambda"<<i<<',';
    for(int i=0;i<4;++i) witness<<"fast_friction"<<i<<',';
    witness<<"ref_rank,ref_eq_residual,ref_contact_residual,ref_max_stick,";
    for(int i=0;i<4;++i) witness<<"ref_mode"<<i<<',';
    for(int i=0;i<9;++i) witness<<"ref_a"<<i<<',';
    for(int i=0;i<4;++i) witness<<"ref_lambda"<<i<<',';
    for(int i=0;i<4;++i) witness<<"ref_friction"<<i<<',';
    witness<<"delta_a_linf,delta_lambda_linf,delta_friction_linf\n";
  }
  auto hold=make_hold();
  compare(hold,"alpha0");
  // The measured held command is the alpha=0 endpoint. Intermediate and final
  // blends are calculated from the unchanged reference inverse law.
  GroundContactMotionInput inv;
  inv.q=hold.q; inv.v=hold.v; inv.wheel_torque<<hold.command[4],hold.command[5];
  auto inverse=ground_contact_motion_inverse(inv);
  assert(inverse.valid);
  const double alphas[]={.25,.5,1.0};
  const Eigen::Vector4d baseline=hold.command.head<4>();
  for(double alpha:alphas) {
    GroundContactFastCertificateInput blend=hold;
    blend.command.head<4>()=(1-alpha)*baseline+alpha*inverse.leg_torque;
    compare(blend,alpha==.25?"alpha025":(alpha==.5?"alpha05":"alpha1"));
  }
  GroundContactMotionControlInput control;
  control.state_and_reference_acceleration=inv;
  control.q_reference<<hold.q[3],hold.q[4],hold.q[5],hold.q[6];
  control.v_reference.setZero(); control.joint_anchor=control.q_reference;
  control.pitch_anchor=hold.q[2]; control.baseline_leg_torque=baseline;
  for(double alpha:{0.0,.25,.5,1.0}) {
    control.command_blend_alpha=alpha;
    const auto fast=ground_contact_motion_control_fast(control);
    const auto full=ground_contact_motion_control(control);
    std::cout<<"full_control_alpha="<<alpha<<" fast="<<fast.valid<<":"<<fast.reason
        <<" reference="<<full.valid<<":"<<full.reason<<" total_us="<<fast.elapsed_us
        <<" final_certificate_us="<<fast.fast_final_elapsed_us
        <<" candidate_count="<<fast.fast_candidate_count<<"\n";
    assert(fast.valid&&full.valid);
    assert(fast.elapsed_us<=1000.0);
    assert((fast.leg_torque-full.leg_torque).cwiseAbs().maxCoeff()<1e-12);
    assert((fast.predicted_acceleration-full.predicted_acceleration).cwiseAbs().maxCoeff()<1e-7);
  }

  auto age4=make_age4();
  compare(age4,"corrected_age4_requestedwheel");
  GroundContactMotionInput reverse_request;
  reverse_request.q=age4.q; reverse_request.v=age4.v;
  reverse_request.leg_acceleration<<-.006768,-.013536,-.006768,-.013536;
  reverse_request.wheel_torque<<age4.command[4],age4.command[5];
  reverse_request.friction_mode_hint={{-1,-1,-1,-1}};
  const auto reverse_inverse=ground_contact_motion_inverse(reverse_request);
  assert(reverse_inverse.valid);
  GroundContactFastCertificateInput reverse=age4;
  reverse.command<<reverse_inverse.leg_torque[0],reverse_inverse.leg_torque[1],
      reverse_inverse.leg_torque[2],reverse_inverse.leg_torque[3],
      age4.command[4],age4.command[5];
  compare(reverse,"negative_slip_inverse_request");

  // Static-effort boundary fixtures are generated from the full-precision
  // all-stick equation itself.  We retain both sides of its ±0.1 N m interval
  // at 1e-10, 1e-9, and 1e-8 offsets and compare with the fixed reference.
  ThrustSupportDynamicsInput di; di.q=hold.q; di.v=hold.v; di.prediction_dt=hold.dt;
  GroundUrdfResponseModel model;
  assert(build_ground_urdf_response_model(di,GroundUrdfResponseProfile::URDFGravityAndFixedIMU,model));
  GroundFrictionResponseOptions fo;
  fo.response_profile=GroundUrdfResponseProfile::URDFGravityAndFixedIMU;
  auto effort_at=[&](const GroundContactFastCertificateInput &x,int joint)->double {
    GroundFrictionResponseInput fi; fi.q=x.q;fi.v=x.v;fi.command=x.command;fi.dt=x.dt;
    GroundFrictionResponseOptions o=fo;
    SupportMatrix9 free_mass=model.M; SupportQ9 free_rhs=-(model.C+model.G);
    SupportMatrix9 D=SupportMatrix9::Zero();
    for(int j=0;j<4;++j)D(3+j,3+j)=.5; D(7,7)=D(8,8)=.1;
    for(int a=0;a<6;++a){int d=3+a;double vis=a<4?.5:.1;free_mass(d,d)+=x.dt*vis;free_rhs[d]+=x.command[a]-vis*x.v[d];}
    Eigen::FullPivLU<SupportMatrix9> lu(free_mass);
    auto afree=lu.solve(free_rhs);
    ground_friction_detail::Candidate c; ground_friction_detail::Reject rej;
    (void)ground_friction_detail::solve_mode(fi,o,model,afree,{{0,0,0,0}},c,rej);
    return c.friction[joint];
  };
  // Locate the command where knee-left stick reaction is -0.1; command is
  // adjusted in the opposite direction because the reaction cancels it.
  double lo=hold.command[1]-.20, hi=hold.command[1]+.20;
  for(int i=0;i<80;++i) {
    double mid=.5*(lo+hi); auto x=hold; x.command[1]=mid;
    const double e=effort_at(x,1);
    if(e>-.1) lo=mid; else hi=mid;
  }
  const double threshold=.5*(lo+hi);
  { auto x=hold; x.command[1]=threshold;
    std::cout<<"boundary_command="<<threshold<<" allstick_reaction="<<effort_at(x,1)<<"\n"; }
  for(double eps:{1e-10,1e-9,1e-8}) for(double sign:{-1.0,1.0}) {
    auto x=hold; x.command[1]=threshold+sign*eps;
    const std::string label=std::string("static_boundary_")+(sign<0?"inside_":"outside_")+std::to_string(eps);
    compare(x,label.c_str());
  }
  double lo_right=hold.command[3]-.20,hi_right=hold.command[3]+.20;
  for(int i=0;i<80;++i) {
    const double mid=.5*(lo_right+hi_right); auto x=hold; x.command[1]=threshold; x.command[3]=mid;
    if(effort_at(x,3)>-.1) lo_right=mid; else hi_right=mid;
  }
  auto mixed_boundary=hold;
  mixed_boundary.command[1]=threshold;
  mixed_boundary.command[3]=.5*(lo_right+hi_right);
  compare(mixed_boundary,"mixed_two_knee_static_boundaries",true);
  const auto mixed_fast=ground_contact_motion_fast_certificate(mixed_boundary);
  assert(!mixed_fast.valid&&mixed_fast.reason=="fast_certificate_deadline_exceeded");

  // Inputs or configuration that bypass a physical guard must fail closed.
  auto invalid=hold; invalid.command[0]=std::numeric_limits<double>::quiet_NaN();
  assert(!ground_contact_motion_fast_certificate(invalid).valid);
  auto deadline=hold; deadline.remaining_budget_us=1e-6;
  assert(!ground_contact_motion_fast_certificate(deadline).valid);
  GroundContactMotionOptions wrong; wrong.profile=GroundUrdfResponseProfile::OriginalCAD;
  assert(!ground_contact_motion_fast_certificate(hold,wrong).valid);
  wrong=GroundContactMotionOptions{}; wrong.max_leg_rate=std::numeric_limits<double>::quiet_NaN();
  assert(!ground_contact_motion_fast_certificate(hold,wrong).valid);
  wrong=GroundContactMotionOptions{}; wrong.rank_relative_threshold=1e-13;
  assert(!ground_contact_motion_fast_certificate(hold,wrong).valid);

  // Bounded-work benchmark uses representative slip/hold fixtures. The
  // certifier rejects any individual call that exceeds 1 ms.
  std::vector<double> times;
  for(int i=0;i<80;++i) {
    const auto &x=(i%2)?hold:age4;
    const auto start=std::chrono::steady_clock::now();
    const auto r=ground_contact_motion_fast_certificate(x);
    const double us=std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-start).count();
    if(r.valid) times.push_back(us);
  }
  std::sort(times.begin(),times.end());
  assert(!times.empty());
  std::cout<<"benchmark_valid_n="<<times.size()<<" p50_us="<<times[times.size()/2]
      <<" p95_us="<<times[(times.size()*95)/100]<<" max_us="<<times.back()<<"\n";
  if(witness.is_open()) witness.close();
  return 0;
}
