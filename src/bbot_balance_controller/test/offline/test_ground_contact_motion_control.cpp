#include <cassert>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "bbot_balance_controller/ground_contact_motion_control.hpp"
#include "bbot_balance_controller/ground_friction_response.hpp"

using namespace bbot_jump;
static GroundContactMotionInput anchor_input() {
  GroundContactMotionInput in;
  in.q << .6716299641397077,.5072433930066407,-.06435687859340004,
      .2552001082917468,-.36970399098083095,.2552001092347051,
      -.36970399147363603,-9.247733753452447,-9.247733697350917;
  in.v << -.001157518494013159,.001756470558116048,.004550379028576565,
      2.7988722450800196e-13,-.008858430989028387,
      2.7998436902265667e-13,-.008858431046409238,
      .023122635202929495,.023122635235357847;
  in.leg_acceleration << .098,-.2760739062,.098,-.2760739062;
  in.wheel_torque << .0022951719,.0022951722;
  return in;
}
int main(int argc, char **argv) {
  auto in=anchor_input();
  auto r=ground_contact_motion_inverse(in);
  if(!r.valid) { std::cerr<<"FAIL "<<r.reason<<" rank="<<r.rank<<" residual="<<r.equation_residual<<"\n"; return 1; }
  assert(r.rank==17 && r.equation_residual<1e-8 && r.contact_residual<1e-8);
  assert(std::abs(r.leg_torque[0]-r.leg_torque[2])<1e-8);
  assert(std::abs(r.leg_torque[1]-r.leg_torque[3])<1e-8);
  assert(std::abs(0.5*(r.acceleration[3]+r.acceleration[5])-
      0.5*(in.leg_acceleration[0]+in.leg_acceleration[2]))<1e-8);
  assert(std::abs(0.5*(r.acceleration[4]+r.acceleration[6])-
      0.5*(in.leg_acceleration[1]+in.leg_acceleration[3]))<1e-8);
  GroundFrictionResponseInput forward;
  forward.q=in.q; forward.v=in.v; forward.dt=in.dt;
  forward.command << r.leg_torque[0],r.leg_torque[1],r.leg_torque[2],r.leg_torque[3],
      in.wheel_torque[0],in.wheel_torque[1];
  GroundFrictionResponseOptions forward_options;
  forward_options.response_profile=GroundUrdfResponseProfile::URDFGravityAndFixedIMU;
  auto check=ground_friction_response(forward,forward_options);
  if(!check.valid) { std::cerr<<"FORWARD_FAIL "<<check.reason<<"\n"; return 3; }
  std::cout<<"forward qdd target-vs-observed max="
      <<(check.acceleration-r.acceleration).cwiseAbs().maxCoeff()
      <<" friction modes=";
  for(int mode:check.leg_mode) std::cout<<mode<<',';
  std::cout<<" predicted=";
  for(int mode:r.leg_friction_mode) std::cout<<mode<<',';
  std::cout<<"\n";
  assert((check.acceleration-r.acceleration).cwiseAbs().maxCoeff()<1e-6);
  std::cout<<"valid rank="<<r.rank<<" sigma_min="<<r.smallest_kept_singular_value
      <<" sigma_max="<<r.largest_singular_value<<" eq="<<r.equation_residual
      <<" contact="<<r.contact_residual<<" hip_tau="<<r.leg_torque[0]
      <<" knee_tau="<<r.leg_torque[1]<<" pitch_qdd="<<r.acceleration[2]
      <<" height_qdd="<<r.acceleration[1]<<"\n";
  in.wheel_torque[1]+=.001;
  const SupportQ9 raw_q=in.q, raw_v=in.v;
  auto asym=ground_contact_motion_inverse(in);
  assert(asym.valid && asym.rank==17);
  assert((in.q-raw_q).norm()==0.0 && (in.v-raw_v).norm()==0.0);
  assert(std::abs(asym.leg_torque[0]-asym.leg_torque[2])<1e-8);
  assert(std::abs(asym.leg_torque[1]-asym.leg_torque[3])<1e-8);
  in=anchor_input(); in.dt=.005;
  auto wrong_dt=ground_contact_motion_inverse(in);
  assert(!wrong_dt.valid && wrong_dt.reason=="requires_1ms_physics_model_step");
  in=anchor_input(); in.bilateral_contact=false;
  auto no_contact=ground_contact_motion_inverse(in);
  assert(!no_contact.valid && no_contact.reason=="bilateral_contact_required");
  GroundContactMotionOptions bad_options;
  bad_options.max_wheel_rate=std::numeric_limits<double>::quiet_NaN();
  const char *bad_reason=nullptr;
  assert(!ground_contact_motion_options_valid(bad_options,&bad_reason));
  assert(std::string(bad_reason)=="nonfinite_option");
  bad_options=GroundContactMotionOptions{}; bad_options.static_friction=.11;
  assert(!ground_contact_motion_options_valid(bad_options,&bad_reason));
  assert(std::string(bad_reason)=="fixed_physics_parameters_required");
  bad_options=GroundContactMotionOptions{}; bad_options.rank_relative_threshold=1e-13;
  assert(!ground_contact_motion_options_valid(bad_options,&bad_reason));
  bad_options=GroundContactMotionOptions{}; bad_options.max_residual=2e-8;
  assert(!ground_contact_motion_options_valid(bad_options,&bad_reason));
  bad_options=GroundContactMotionOptions{}; bad_options.leg_effort_limit[0]=75.1;
  assert(!ground_contact_motion_options_valid(bad_options,&bad_reason));
  GroundContactMotionInput asymmetric_fixture;
  asymmetric_fixture.q << .65881672026013416,.51199177778941907,-.060680581366889419,
      .26520497206377536,-.38925489170835548,.265204977940106,-.38925489151770176,
      -9.0656783425930314,-9.0656783205539373;
  asymmetric_fixture.v << -.002317450765813031,4.0038760744420541e-05,-.00041518951306884647,
      .00011519310753815741,1.8866597917510158e-17,.00011519308459181988,
      -5.7061825142311026e-17,.035428117664294649,.035428117760872818;
  asymmetric_fixture.leg_acceleration << -.0067680000000107923,.013536000000021585,
      -.0067680000000107923,.013536000000021585;
  asymmetric_fixture.wheel_torque << .01081191159339033,.010811911496812161;
  const SupportQ9 unprojected_q=asymmetric_fixture.q, unprojected_v=asymmetric_fixture.v;
  auto compatible=ground_contact_motion_inverse(asymmetric_fixture);
  assert(compatible.valid && compatible.rank==17);
  assert(compatible.equation_residual<1e-8 && compatible.contact_residual<1e-8);
  assert((asymmetric_fixture.q-unprojected_q).norm()==0.0);
  assert((asymmetric_fixture.v-unprojected_v).norm()==0.0);
  assert(std::abs(0.5*(compatible.acceleration[3]+compatible.acceleration[5])-
      asymmetric_fixture.leg_acceleration[0])<1e-8);
  assert(std::abs(0.5*(compatible.acceleration[4]+compatible.acceleration[6])-
      asymmetric_fixture.leg_acceleration[1])<1e-8);
  forward.q=asymmetric_fixture.q; forward.v=asymmetric_fixture.v;
  forward.command << compatible.leg_torque[0],compatible.leg_torque[1],
      compatible.leg_torque[2],compatible.leg_torque[3],
      asymmetric_fixture.wheel_torque[0],asymmetric_fixture.wheel_torque[1];
  auto compatible_forward=ground_friction_response(forward,forward_options);
  assert(compatible_forward.valid && compatible_forward.distinct_accelerations==1);
  assert(compatible_forward.leg_mode==compatible.leg_friction_mode);
  assert((compatible_forward.acceleration-compatible.acceleration).cwiseAbs().maxCoeff()<1e-7);
  assert(asymmetric_fixture.dt*std::abs(compatible.acceleration[3]-compatible.acceleration[5])<1e-6);
  assert(asymmetric_fixture.dt*std::abs(compatible.acceleration[4]-compatible.acceleration[6])<1e-6);
  std::cout<<"asym fixture 17x17 rank="<<compatible.rank<<" sigma_min="
      <<compatible.smallest_kept_singular_value<<" sigma_max="<<compatible.largest_singular_value
      <<" eq="<<compatible.equation_residual<<" contact="<<compatible.contact_residual
      <<" accel=["<<compatible.acceleration[3]<<','<<compatible.acceleration[4]<<','
      <<compatible.acceleration[5]<<','<<compatible.acceleration[6]<<"] lr_gap="
      <<compatible.acceleration[3]-compatible.acceleration[5]<<','
      <<compatible.acceleration[4]-compatible.acceleration[6]<<" full81err="
      <<(compatible_forward.acceleration-compatible.acceleration).cwiseAbs().maxCoeff()
      <<" modes=";
  for(int m:compatible_forward.leg_mode) std::cout<<m<<',';
  std::cout<<"\n";
  in=anchor_input();
  GroundContactMotionControlInput ci;
  ci.state_and_reference_acceleration=in;
  ci.q_reference << .25772010829274028,-.37429458170079477,
      .25772010923569871,-.37429458218941797;
  ci.v_reference << .01,-.02,.01,-.02;
  ci.joint_anchor << in.q[3],in.q[4],in.q[5],in.q[6];
  ci.pitch_anchor=in.q[2];
  auto control=ground_contact_motion_control(ci);
  if(!control.valid) { std::cerr<<"CONTROL_FAIL "<<control.reason<<"\n"; return 2; }
  assert(control.task_mobility.allFinite() && control.effective_inertia.allFinite());
  assert(std::abs(control.leg_torque[0]-control.leg_torque[2])<1e-8);
  assert(std::abs(control.leg_torque[1]-control.leg_torque[3])<1e-8);
  assert(std::isfinite(control.predicted_pitch_delta));
  assert(control.predicted_soft_margin>.30);
  forward.command << control.leg_torque[0],control.leg_torque[1],
      control.leg_torque[2],control.leg_torque[3],
      in.wheel_torque[0],in.wheel_torque[1];
  forward.q=in.q; forward.v=in.v; forward.dt=in.dt;
  auto control_forward=ground_friction_response(forward,forward_options);
  if(!control_forward.valid) { std::cerr<<"CONTROL_FORWARD_FAIL "<<control_forward.reason<<"\n"; return 4; }
  std::cout<<"control full-forward delta="
      <<(control_forward.acceleration-control.predicted_acceleration).cwiseAbs().maxCoeff()
      <<" helper modes=";
  for(int m:control.predicted_leg_friction_mode) std::cout<<m<<',';
  std::cout<<" ref modes=";
  for(int m:control_forward.leg_mode) std::cout<<m<<',';
  std::cout<<" helper_candidates="<<control.final_candidate_modes
      <<" ref_candidates="<<control_forward.candidate_modes<<"\n";
  assert((control_forward.acceleration-control.predicted_acceleration)
      .cwiseAbs().maxCoeff()<1e-6);
  assert(control_forward.leg_mode==control.predicted_leg_friction_mode);
  assert(std::abs(control.predicted_hip_position_difference)<=1e-6);
  assert(std::abs(control.predicted_knee_position_difference)<=1e-6);
  assert(std::abs(control.predicted_hip_rate_difference)<=1e-6);
  assert(std::abs(control.predicted_knee_rate_difference)<=1e-6);
  std::cout<<"control mobility=["<<control.task_mobility(0,0)<<','
      <<control.task_mobility(0,1)<<';'<<control.task_mobility(1,0)<<','
      <<control.task_mobility(1,1)<<"] effective=["
      <<control.effective_inertia(0,0)<<','<<control.effective_inertia(0,1)<<';'
      <<control.effective_inertia(1,0)<<','<<control.effective_inertia(1,1)
      <<"] pd="<<control.pd_torque.transpose()<<" correction="
      <<control.implicit_pd_correction.transpose()<<" output="
      <<control.leg_torque.transpose()<<" predicted_pitch_delta="
      <<control.predicted_pitch_delta<<" predicted_soft_margin="
      <<control.predicted_soft_margin<<"\n";
  // The launch handoff blends from the measured baseline command. Verify the
  // blended command itself against the independent full 81-mode forward model.
  GroundContactMotionControlInput blended=ci;
  blended.command_blend_alpha=.5;
  blended.baseline_leg_torque=control.reference_inverse.leg_torque;
  auto blend_result=ground_contact_motion_control(blended);
  if(!blend_result.valid) { std::cerr<<"BLEND_FAIL "<<blend_result.reason<<"\n"; return 5; }
  forward.command << blend_result.leg_torque[0],blend_result.leg_torque[1],
      blend_result.leg_torque[2],blend_result.leg_torque[3],
      in.wheel_torque[0],in.wheel_torque[1];
  auto blend_forward=ground_friction_response(forward,forward_options);
  if(!blend_forward.valid) { std::cerr<<"BLEND_FORWARD_FAIL "<<blend_forward.reason<<"\n"; return 6; }
  assert((blend_forward.acceleration-blend_result.predicted_acceleration)
      .cwiseAbs().maxCoeff()<1e-6);
  assert(blend_forward.leg_mode==blend_result.predicted_leg_friction_mode);
  assert(std::abs(blend_result.leg_torque[0]-blend_result.leg_torque[2])<1e-8);
  std::cout<<"blend alpha=.5 torque="<<blend_result.leg_torque.transpose()
      <<" full-forward-error="<<(blend_forward.acceleration-blend_result.predicted_acceleration)
      .cwiseAbs().maxCoeff()<<" modes=";
  for(int mode:blend_forward.leg_mode) std::cout<<mode<<',';
  std::cout<<"\n";
  blended.command_blend_alpha=1.01;
  assert(!ground_contact_motion_control(blended).valid);
  blended=ci;
  blended.baseline_leg_torque<<0.0,0.0,0.01,0.0;
  assert(!ground_contact_motion_control(blended).valid);
  // The actual first-motion anchor from the frozen NCS1 record. All four leg
  // joints were stationary here, while the measured applied leg command and
  // both wheel commands were nonzero. Exercise each point of the approved
  // 2-second command blend against the independent 81-mode forward model.
  GroundContactMotionInput hold_state;
  hold_state.q << .67284161996328307,.50713710928565003,-.064389900807068995,
      .25520010829184914,-.36925458170020709,.25520010923480746,
      -.3692545821888305,-9.2668105162323808,-9.2668104601098005;
  hold_state.v << -.0033648902344868129,5.7078255696882597e-05,
      -.00065758052184974175,-4.2008757583644751e-13,4.881373083520657e-13,
      -4.2010839251815923e-13,4.8798465268617974e-13,
      .052833907416522746,.052833907362268492;
  hold_state.leg_acceleration.setZero();
  hold_state.wheel_torque << .0089971939605110185,.0089971940042035264;
  GroundContactMotionControlInput hold_control;
  hold_control.state_and_reference_acceleration=hold_state;
  hold_control.q_reference<<hold_state.q[3],hold_state.q[4],hold_state.q[5],hold_state.q[6];
  hold_control.v_reference.setZero();
  hold_control.joint_anchor=hold_control.q_reference;
  hold_control.pitch_anchor=hold_state.q[2];
  hold_control.baseline_leg_torque<<-.72345083572675517,-15.31390561244141,
      -.72345084855659703,-15.313905619364968;
  auto static_reference=ground_contact_motion_inverse(hold_state);
  std::cout<<"static reference valid="<<static_reference.valid
      <<" reason="<<static_reference.reason<<" tau="
      <<static_reference.leg_torque.transpose()<<" friction="
      <<static_reference.leg_friction_torque.transpose()<<" modes=";
  for(int mode:static_reference.leg_friction_mode) std::cout<<mode<<',';
  std::cout<<"\n";
  const double blend_alpha[]={0.0,.25,.5,1.0};
  // Test output belongs to the calling build/replay directory, so repeated
  // verification cannot overwrite the frozen qualification record.
  std::ofstream blend_csv(argc > 1 ? argv[1] : "static_blend_witness.csv");
  assert(blend_csv.good());
  blend_csv<<"sim_ns,alpha,helper_valid,helper_reason,forward_valid,forward_reason,";
  for(int i=0;i<9;++i) blend_csv<<"q"<<i<<',';
  for(int i=0;i<9;++i) blend_csv<<"v"<<i<<',';
  blend_csv<<"wheel_u0,wheel_u1,";
  for(int i=0;i<4;++i) blend_csv<<"baseline_u"<<i<<',';
  for(int i=0;i<4;++i) blend_csv<<"inverse_u"<<i<<',';
  for(int i=0;i<4;++i) blend_csv<<"blended_u"<<i<<',';
  for(int i=0;i<4;++i) blend_csv<<"helper_mode"<<i<<',';
  for(int i=0;i<4;++i) blend_csv<<"forward_mode"<<i<<',';
  for(int i=0;i<4;++i) blend_csv<<"forward_friction"<<i<<',';
  blend_csv<<"max_stick_effort,";
  for(int i=0;i<9;++i) blend_csv<<"forward_a"<<i<<',';
  for(int i=0;i<9;++i) blend_csv<<"helper_a"<<i<<',';
  blend_csv<<"max_a_error,helper_eq_residual,helper_contact_residual,helper_candidate_modes,"
      <<"helper_distinct_accelerations,helper_static_effort\n"<<std::setprecision(17);
  for(double alpha:blend_alpha) {
    hold_control.command_blend_alpha=alpha;
    auto hold_result=ground_contact_motion_control(hold_control);
    assert(hold_result.valid);
    std::cout<<"hold alpha="<<alpha<<" helper_valid="<<hold_result.valid
        <<" helper_reason="<<hold_result.reason;
    if(!static_reference.valid) continue;
    Eigen::Matrix<double,4,1> final_tau=hold_result.leg_torque;
    GroundFrictionResponseInput hold_forward;
    hold_forward.q=hold_state.q; hold_forward.v=hold_state.v;
    hold_forward.dt=hold_state.dt;
    hold_forward.command << final_tau[0],final_tau[1],final_tau[2],final_tau[3],
        hold_state.wheel_torque[0],hold_state.wheel_torque[1];
    auto hold_check=ground_friction_response(hold_forward,forward_options);
    assert(hold_check.valid);
    assert(hold_check.leg_mode==hold_result.predicted_leg_friction_mode);
    for(int mode:hold_check.leg_mode) assert(mode==0);
    std::cout<<" final_tau="<<final_tau.transpose();
    if(hold_result.valid) {
      std::cout<<" helper_modes=";
      for(int mode:hold_result.reference_inverse.leg_friction_mode) std::cout<<mode<<',';
    }
    if(!hold_check.valid) {
      std::cout<<" forward_valid=0 reason="<<hold_check.reason<<"\n";
      continue;
    }
    std::cout<<" forward_valid=1 modes=";
    for(int mode:hold_check.leg_mode) std::cout<<mode<<',';
    std::cout<<" static_effort="<<hold_check.max_stick_effort
        <<" qdd="<<hold_check.acceleration.transpose();
    const double hold_qdd_error=(hold_check.acceleration-hold_result.predicted_acceleration)
        .cwiseAbs().maxCoeff();
    std::cout<<" qdd_error="<<hold_qdd_error;
    assert(hold_qdd_error<1e-6);
    assert(hold_result.maximum_static_friction_effort<=.1+1e-8);
    if(alpha<1.0) assert(hold_result.maximum_static_friction_effort>1e-3);
    assert(hold_result.final_candidate_modes>0 && hold_result.final_distinct_accelerations==1);
    assert(hold_result.final_equation_residual<=1e-8 && hold_result.final_contact_residual<=1e-8);
    assert(std::abs(hold_result.predicted_hip_position_difference)<=1e-6);
    assert(std::abs(hold_result.predicted_knee_position_difference)<=1e-6);
    assert(std::abs(hold_result.predicted_hip_rate_difference)<=1e-6);
    assert(std::abs(hold_result.predicted_knee_rate_difference)<=1e-6);
    std::cout<<" candidates="<<hold_result.final_candidate_modes
        <<" eq_residual="<<hold_result.final_equation_residual
        <<" contact_residual="<<hold_result.final_contact_residual
        <<" kkt_rank="<<hold_check.kkt_rank;
    blend_csv<<18567000000LL<<','<<alpha<<','<<(hold_result.valid?1:0)<<','<<hold_result.reason<<','
        <<(hold_check.valid?1:0)<<','<<hold_check.reason<<',';
    for(int i=0;i<9;++i) blend_csv<<hold_state.q[i]<<',';
    for(int i=0;i<9;++i) blend_csv<<hold_state.v[i]<<',';
    blend_csv<<hold_state.wheel_torque[0]<<','<<hold_state.wheel_torque[1]<<',';
    for(int i=0;i<4;++i) blend_csv<<hold_control.baseline_leg_torque[i]<<',';
    for(int i=0;i<4;++i) blend_csv<<static_reference.leg_torque[i]<<',';
    for(int i=0;i<4;++i) blend_csv<<final_tau[i]<<',';
    for(int i=0;i<4;++i) blend_csv<<hold_result.reference_inverse.leg_friction_mode[i]<<',';
    for(int i=0;i<4;++i) blend_csv<<hold_check.leg_mode[i]<<',';
    for(int i=0;i<4;++i) blend_csv<<hold_check.leg_friction_torque[i]<<',';
    blend_csv<<hold_check.max_stick_effort<<',';
    for(int i=0;i<9;++i) blend_csv<<hold_check.acceleration[i]<<',';
    for(int i=0;i<9;++i) blend_csv<<(hold_result.valid?hold_result.predicted_acceleration[i]:
        std::numeric_limits<double>::quiet_NaN())<<',';
    blend_csv<<(hold_result.valid?(hold_check.acceleration-hold_result.predicted_acceleration)
        .cwiseAbs().maxCoeff():std::numeric_limits<double>::quiet_NaN())<<','
        <<hold_result.final_equation_residual<<','<<hold_result.final_contact_residual<<','
        <<hold_result.final_candidate_modes<<','<<hold_result.final_distinct_accelerations<<','
        <<hold_result.maximum_static_friction_effort<<'\n';
    std::cout<<"\n";
  }
  blend_csv.close();
  GroundContactMotionControlInput asym_control;
  asym_control.state_and_reference_acceleration=asymmetric_fixture;
  asym_control.q_reference<<asymmetric_fixture.q[3],asymmetric_fixture.q[4],
      asymmetric_fixture.q[5],asymmetric_fixture.q[6];
  asym_control.v_reference<<asymmetric_fixture.v[3],asymmetric_fixture.v[4],
      asymmetric_fixture.v[5],asymmetric_fixture.v[6];
  asym_control.joint_anchor=asym_control.q_reference;
  asym_control.pitch_anchor=asymmetric_fixture.q[2];
  asym_control.baseline_leg_torque=compatible.leg_torque;
  auto asym_control_check=ground_contact_motion_control(asym_control);
  assert(asym_control_check.valid);
  const auto bench_start=std::chrono::steady_clock::now();
  std::size_t bench_valid=0;
  std::array<double,100> bench_samples{};
  for(int i=0;i<100;++i) {
    const auto one=std::chrono::steady_clock::now();
    const GroundContactMotionControlInput *bench_input=nullptr;
    if(i%3==0) bench_input=&ci;
    else if(i%3==1) { hold_control.command_blend_alpha=.25; bench_input=&hold_control; }
    else bench_input=&asym_control;
    bench_valid+=ground_contact_motion_control(*bench_input).valid?1u:0u;
    bench_samples[i]=std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-one).count();
  }
  const double bench_mean=std::chrono::duration<double,std::micro>(
      std::chrono::steady_clock::now()-bench_start).count()/100.0;
  std::sort(bench_samples.begin(),bench_samples.end());
  std::cout<<"100 helper calls (moving, static-blend, asymmetric slip): valid="<<bench_valid<<" mean_us="<<bench_mean
      <<" p50_us="<<bench_samples[49]<<" p95_us="<<bench_samples[94]
      <<" max_us="<<bench_samples.back()
      <<" (offline host timing; excludes controller/ROS scheduling)\n";
}
