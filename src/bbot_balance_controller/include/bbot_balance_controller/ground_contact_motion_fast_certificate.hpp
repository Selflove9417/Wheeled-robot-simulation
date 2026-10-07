#pragma once

// Offline bounded-work final-command certificate for ground joint friction.
// This is intentionally separate from the frozen full-81 reference and is not
// connected to a controller.
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "bbot_balance_controller/ground_contact_motion_control.hpp"

namespace bbot_jump {

struct GroundContactFastCertificateInput {
  SupportQ9 q = SupportQ9::Zero();
  SupportQ9 v = SupportQ9::Zero();
  SupportActuatorVector command = SupportActuatorVector::Zero();
  double dt = 0.001;
  bool bilateral_contact = true;
  double wheel_ground_friction = 1.0;
  double remaining_budget_us = 1000.0;
  // Optional caller-provided preference (for example last certified mode).
  // It changes search order only; it cannot make an infeasible mode valid.
  bool has_priority_mode = false;
  std::array<int, 4> priority_mode{{0, 0, 0, 0}};
};

struct GroundContactFastCertificateResult {
  bool valid{false};
  std::string reason{"uninitialized"};
  SupportQ9 acceleration = SupportQ9::Zero();
  SupportContactVector contact_force = SupportContactVector::Zero();
  Eigen::Matrix<double, 4, 1> leg_friction_torque = Eigen::Matrix<double, 4, 1>::Zero();
  std::array<int, 4> leg_mode{{0, 0, 0, 0}};
  int kkt_rank{0};
  int kkt_dimension{0};
  int candidate_count{0};
  int feasible_candidate_count{0};
  int distinct_accelerations{0};
  double equation_residual{std::numeric_limits<double>::infinity()};
  double contact_residual{std::numeric_limits<double>::infinity()};
  double max_stick_effort{0.0};
  double max_contact_cone_violation{0.0};
  double elapsed_us{0.0};
};

namespace ground_contact_fast_detail {

inline int encode_mode(const std::array<int, 4> &mode) {
  int code=0, place=1;
  for(int j=0;j<4;++j) { code += (mode[j]+1)*place; place*=3; }
  return code;
}

inline std::array<int,4> decode_mode(int code) {
  std::array<int,4> mode{};
  for(int j=0;j<4;++j) { mode[j]=(code%3)-1; code/=3; }
  return mode;
}

inline bool same_mode(const std::array<int,4> &a,const std::array<int,4> &b) {
  for(int j=0;j<4;++j) if(a[j]!=b[j]) return false;
  return true;
}

inline GroundFrictionResponseOptions response_options(
    const GroundContactMotionOptions &o, double dt) {
  GroundFrictionResponseOptions f;
  f.response_profile=o.profile; f.dt=dt;
  f.static_friction=o.static_friction; f.dynamic_friction=o.dynamic_friction;
  f.leg_viscous=o.leg_viscous; f.wheel_viscous=o.wheel_viscous;
  f.max_contact_friction=o.wheel_ground_friction;
  f.max_abs_equation_residual=o.max_residual;
  f.contact_residual_tolerance=o.max_residual;
  f.input_effort_limits={{o.leg_effort_limit[0],o.leg_effort_limit[1],
      o.leg_effort_limit[2],o.leg_effort_limit[3],o.wheel_effort_limit,
      o.wheel_effort_limit}};
  return f;
}

inline bool deadline_expired(
    const std::chrono::steady_clock::time_point &start,
    double budget_us,
    GroundContactFastCertificateResult &out) {
  out.elapsed_us=std::chrono::duration<double,std::micro>(
      std::chrono::steady_clock::now()-start).count();
  if(out.elapsed_us<=budget_us) return false;
  out.valid=false; out.reason="fast_certificate_deadline_exceeded";
  return true;
}

} // namespace ground_contact_fast_detail

// Certify an already-final six-actuator command against the fixed 9-DOF
// bilateral model. A valid fixed-mode candidate is sufficient away from
// numerical mode boundaries because the discrete Coulomb problem is a
// strictly convex quadratic (M SPD) plus a convex L1 term on an affine contact
// set: acceleration is unique, although reaction multipliers need not be.
// At the solver's explicit slip/static tolerances, neighboring candidates are
// enumerated and clustered exactly as in the full-81 reference. More than one
// acceleration cluster is rejected.
inline GroundContactFastCertificateResult ground_contact_motion_fast_certificate(
    const GroundContactFastCertificateInput &in,
    const GroundContactMotionOptions &options = GroundContactMotionOptions{}) {
  using namespace ground_contact_fast_detail;
  using Clock=std::chrono::steady_clock;
  const auto started=Clock::now();
  GroundContactFastCertificateResult out;
  const auto reject=[&](const char *why) {
    out.valid=false; out.reason=why;
    out.elapsed_us=std::chrono::duration<double,std::micro>(Clock::now()-started).count();
    return out;
  };
  const char *why=nullptr;
  if(!ground_contact_motion_options_valid(options,&why)) return reject(why);
  if(!in.q.allFinite()||!in.v.allFinite()||!in.command.allFinite()||
      !std::isfinite(in.dt)||!std::isfinite(in.wheel_ground_friction))
    return reject("nonfinite_input");
  if(!std::isfinite(in.remaining_budget_us)||in.remaining_budget_us<=0.0||
     in.remaining_budget_us>1000.0)
    return reject("invalid_fast_deadline_budget");
  if(!in.bilateral_contact) return reject("bilateral_contact_required");
  if(std::abs(in.dt-.001)>1e-12||std::abs(in.wheel_ground_friction-1.0)>1e-12)
    return reject("fixed_1ms_contact_profile_required");
  for(int j=0;j<6;++j) {
    const double limit=j<4?options.leg_effort_limit[j]:options.wheel_effort_limit;
    if(std::abs(in.command[j])>limit+1e-9) return reject("input_force_limit_exceeded");
  }
  if(in.has_priority_mode)
    for(int m:in.priority_mode) if(m < -1 || m > 1) return reject("invalid_priority_mode");

  ThrustSupportDynamicsInput dynamics;
  dynamics.q=in.q; dynamics.v=in.v; dynamics.prediction_dt=in.dt;
  dynamics.friction_coefficient=in.wheel_ground_friction;
  GroundUrdfResponseModel model;
  if(!build_ground_urdf_response_model(dynamics,options.profile,model))
    return reject("fixed_ground_model_invalid");
  if(deadline_expired(started,in.remaining_budget_us,out))
    return reject("fast_certificate_deadline_exceeded");
  Eigen::LLT<SupportMatrix9> mass_llt(model.M);
  if(mass_llt.info()!=Eigen::Success) return reject("mass_not_positive_definite");
  Eigen::JacobiSVD<Eigen::Matrix<double,4,9>> contact_svd(
      model.contactJacobian,Eigen::ComputeFullU|Eigen::ComputeFullV);
  if(contact_svd.info()!=Eigen::Success||!contact_svd.singularValues().allFinite())
    return reject("contact_rank_check_failed");
  int contact_rank=0;
  for(int i=0;i<contact_svd.singularValues().size();++i)
    if(contact_svd.singularValues()[i]>options.rank_relative_threshold*
       contact_svd.singularValues()[0]) ++contact_rank;
  if(contact_rank!=4) return reject("contact_jacobian_rank_not_four");
  if(deadline_expired(started,in.remaining_budget_us,out))
    return reject("fast_certificate_deadline_exceeded");

  SupportMatrix9 free_mass=model.M;
  SupportQ9 free_rhs=-(model.C+model.G);
  for(int a=0;a<6;++a) {
    const int dof=3+a; const double viscous=a<4?options.leg_viscous:options.wheel_viscous;
    free_mass(dof,dof)+=in.dt*viscous;
    free_rhs[dof]+=in.command[a]-viscous*in.v[dof];
  }
  Eigen::FullPivLU<SupportMatrix9> free_solver(free_mass);
  if(!free_mass.allFinite()||!free_rhs.allFinite()||!free_solver.isInvertible())
    return reject("free_forward_dynamics_singular_or_nonfinite");
  const SupportQ9 free_acceleration=free_solver.solve(free_rhs);
  if(!free_acceleration.allFinite()) return reject("free_forward_dynamics_nonfinite");
  const double free_res=(free_mass*free_acceleration-free_rhs).cwiseAbs().maxCoeff();
  if(free_res>options.max_residual) return reject("free_equation_residual_exceeded");
  if(deadline_expired(started,in.remaining_budget_us,out))
    return reject("fast_certificate_deadline_exceeded");
  const auto fopts=response_options(options,in.dt);
  GroundFrictionResponseInput response_input;
  response_input.q=in.q; response_input.v=in.v; response_input.command=in.command;
  response_input.dt=in.dt; response_input.bilateral_contact=in.bilateral_contact;
  response_input.wheel_ground_friction=in.wheel_ground_friction;
  const auto consider=[&](const std::array<int,4> &mode,
                          std::vector<ground_friction_detail::Candidate> &feasible)->bool {
    if(deadline_expired(started,in.remaining_budget_us,out)) return false;
    ++out.candidate_count;
    ground_friction_detail::Candidate c;
    ground_friction_detail::Reject rejection=ground_friction_detail::kRejectSingularOrNonfinite;
    if(ground_friction_detail::solve_mode(response_input,fopts,model,free_acceleration,mode,c,rejection))
      feasible.push_back(c);
    out.elapsed_us=std::chrono::duration<double,std::micro>(Clock::now()-started).count();
    return out.elapsed_us<=in.remaining_budget_us;
  };

  std::vector<ground_friction_detail::Candidate> feasible;
  std::array<bool,81> scheduled{};
  std::array<bool,81> tested{};
  std::vector<int> order;
  const auto add_mode=[&](const std::array<int,4>&mode) {
    const int code=encode_mode(mode);
    if(code>=0&&code<81&&!scheduled[code]) { scheduled[code]=true; order.push_back(code); }
  };
  if(in.has_priority_mode) add_mode(in.priority_mode);
  add_mode({{0,0,0,0}});
  std::array<int,4> velocity_mode{};
  std::array<int,4> observed_mode{};
  for(int j=0;j<4;++j) {
    const double vn=in.v[3+j]+in.dt*free_acceleration[3+j];
    velocity_mode[j]=(vn>0.0)?1:((vn<0.0)?-1:0);
    observed_mode[j]=(in.v[3+j]>0.0)?1:((in.v[3+j]<0.0)?-1:0);
  }
  add_mode(observed_mode);
  add_mode(velocity_mode);
  add_mode({{1,1,1,1}});
  add_mode({{-1,-1,-1,-1}});
  add_mode({{1,-1,1,-1}});
  add_mode({{-1,1,-1,1}});
  for(int code=0;code<81;++code) if(!scheduled[code]) { scheduled[code]=true; order.push_back(code); }

  bool boundary_case=false;
  std::array<int,4> boundary_joint{{0,0,0,0}};
  for(int code:order) {
    if(tested[code]) continue;
    tested[code]=true;
    const auto mode=decode_mode(code);
    if(!consider(mode,feasible)) return reject("fast_certificate_deadline_exceeded");
    if(feasible.empty()) continue;
    const auto &c=feasible.back();
    for(int j=0;j<4;++j) {
      if(mode[j]!=0) {
        const double vn=in.v[3+j]+in.dt*c.acceleration[3+j];
        if(std::abs(vn)<=fopts.slip_velocity_tolerance) {
          boundary_case=true; boundary_joint[j]=1;
        }
      } else if(std::abs(std::abs(c.friction[j])-fopts.static_friction)<=
                fopts.static_effort_tolerance) {
        boundary_case=true; boundary_joint[j]=2;
      }
    }
    if(!boundary_case) break;
    // Evaluate only adjacent modes at the detected boundary coordinates. A
    // full 81-mode fallback is reserved for cases with no early feasible mode.
    std::vector<std::array<int,4>> neighbors{mode};
    for(int j=0;j<4;++j) if(boundary_joint[j]) {
      std::vector<std::array<int,4>> expanded;
      for(const auto &base:neighbors) {
        if(boundary_joint[j]==1) {
          expanded.push_back(base);
          auto stick=base; stick[j]=0; expanded.push_back(stick);
          auto opposite=base; opposite[j]=-base[j]; expanded.push_back(opposite);
        } else {
          for(int choice:{-1,0,1}) { auto adjacent=base; adjacent[j]=choice; expanded.push_back(adjacent); }
        }
      }
      neighbors.swap(expanded);
    }
    for(const auto &neighbor:neighbors) {
      const int adjacent_code=encode_mode(neighbor);
      if(tested[adjacent_code]) continue;
      tested[adjacent_code]=true;
      const std::size_t prior_feasible=feasible.size();
      if(!consider(neighbor,feasible)) return reject("fast_certificate_deadline_exceeded");
      if(feasible.size()>prior_feasible) {
        const auto &adjacent=feasible.back();
        for(int j=0;j<4;++j) {
          bool newly_boundary=false;
          if(adjacent.mode[j]==0) {
            newly_boundary=std::abs(std::abs(adjacent.friction[j])-fopts.static_friction)<=
                fopts.static_effort_tolerance;
          } else {
            const double vn=in.v[3+j]+in.dt*adjacent.acceleration[3+j];
            newly_boundary=std::abs(vn)<=fopts.slip_velocity_tolerance;
          }
          if(newly_boundary&&boundary_joint[j]==0)
            return reject("fast_boundary_closure_requires_more_modes");
        }
      }
    }
    break;
  }
  if(feasible.empty()) return reject("no_consistent_friction_mode");
  out.feasible_candidate_count=static_cast<int>(feasible.size());

  std::vector<bool> redundant(feasible.size(),false);
  for(std::size_t i=0;i<feasible.size();++i) {
    auto complement=feasible[i].mode; bool has=false;
    for(int j=0;j<4;++j) if(complement[j]!=0) {
      const double vn=in.v[3+j]+in.dt*feasible[i].acceleration[3+j];
      if(std::abs(vn)<=fopts.slip_velocity_tolerance) { complement[j]=0; has=true; }
    }
    if(!has) continue;
    for(std::size_t k=0;k<feasible.size();++k)
      if(k!=i&&same_mode(feasible[k].mode,complement)) { redundant[i]=true; break; }
  }
  std::vector<int> clusters;
  for(int i=0;i<static_cast<int>(feasible.size());++i) {
    if(redundant[i]) continue;
    bool assigned=false;
    for(int chosen:clusters)
      if((feasible[chosen].acceleration-feasible[i].acceleration).cwiseAbs().maxCoeff()<=
         fopts.acceleration_equivalence_tolerance) { assigned=true; break; }
    if(!assigned) clusters.push_back(i);
  }
  out.distinct_accelerations=static_cast<int>(clusters.size());
  if(clusters.empty()) return reject("no_nonredundant_friction_mode");
  if(clusters.size()!=1) return reject("ambiguous_distinct_acceleration_solutions");
  int chosen=clusters.front();
  for(int index=0;index<static_cast<int>(feasible.size());++index)
    if(!redundant[index]&&
       (feasible[chosen].acceleration-feasible[index].acceleration).cwiseAbs().maxCoeff()<=
           fopts.acceleration_equivalence_tolerance&&
       feasible[index].equation_residual<feasible[chosen].equation_residual) chosen=index;
  const auto &c=feasible[chosen];
  // The reference response has both absolute and relative equation tolerance;
  // this fast path additionally preserves the experiment's strict absolute
  // 1e-8 equation/contact gates.
  if(c.equation_residual>options.max_residual||c.contact_residual>options.max_residual)
    return reject("final_response_residual_exceeded");
  out.acceleration=c.acceleration; out.contact_force=c.contact_force;
  out.leg_friction_torque=c.friction; out.leg_mode=c.mode;
  out.kkt_rank=c.rank; out.kkt_dimension=13+static_cast<int>(
      (c.mode[0]==0)+(c.mode[1]==0)+(c.mode[2]==0)+(c.mode[3]==0));
  out.equation_residual=c.equation_residual; out.contact_residual=c.contact_residual;
  out.max_stick_effort=c.stick_effort; out.max_contact_cone_violation=c.cone_violation;
  out.valid=true; out.reason="ok";
  out.elapsed_us=std::chrono::duration<double,std::micro>(Clock::now()-started).count();
  if(out.elapsed_us>in.remaining_budget_us) { out.valid=false; out.reason="fast_certificate_deadline_exceeded"; }
  return out;
}

struct GroundContactMotionFastControlResult : GroundContactMotionControlResult {
  int fast_candidate_count{0};
  int fast_final_kkt_rank{0};
  int fast_final_kkt_dimension{0};
  double fast_final_elapsed_us{0.0};
  double elapsed_us{0.0};
};

// Complete offline control-path wrapper. It preserves the baseline inverse,
 // constrained 2x2 mobility, torque PD, implicit correction, alpha blend, and
 // final state guards; only the final 81-mode enumeration is replaced by the
 // bounded fixed-mode certificate. It rejects any complete call over 1 ms.
inline GroundContactMotionFastControlResult ground_contact_motion_control_fast(
    const GroundContactMotionControlInput &in,
    const GroundContactMotionOptions &options = GroundContactMotionOptions{}) {
  GroundContactMotionFastControlResult out;
  const auto fast_started=std::chrono::steady_clock::now();
  const auto reject = [&](const char *why) {
    out.valid = false; out.reason = why;
    out.elapsed_us=std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-fast_started).count();
    if(out.elapsed_us>1000.0) out.reason="fast_control_deadline_exceeded";
    return out;
  };
  const char *option_reason=nullptr;
  if(!ground_contact_motion_options_valid(options,&option_reason)) return reject(option_reason);
  const auto &state = in.state_and_reference_acceleration;
  if (!in.q_reference.allFinite() || !in.v_reference.allFinite() ||
      !in.joint_anchor.allFinite() || !std::isfinite(in.pitch_anchor) ||
      !std::isfinite(in.horizon) || in.horizon <= 0.0 ||
      !std::isfinite(in.command_blend_alpha) || !in.baseline_leg_torque.allFinite())
    return reject("nonfinite_reference_or_horizon");
  if (in.command_blend_alpha < 0.0 || in.command_blend_alpha > 1.0)
    return reject("command_blend_alpha_out_of_range");
  const double reference_asymmetry = std::max({
      std::abs(in.q_reference[0]-in.q_reference[2]),
      std::abs(in.q_reference[1]-in.q_reference[3]),
      std::abs(in.v_reference[0]-in.v_reference[2]),
      std::abs(in.v_reference[1]-in.v_reference[3]),
      std::abs(in.joint_anchor[0]-in.joint_anchor[2]),
      std::abs(in.joint_anchor[1]-in.joint_anchor[3])});
  if (reference_asymmetry > options.symmetry_tolerance)
    return reject("symmetric_reference_branch_required");
  if (std::abs(in.baseline_leg_torque[0]-in.baseline_leg_torque[2]) > options.symmetry_tolerance ||
      std::abs(in.baseline_leg_torque[1]-in.baseline_leg_torque[3]) > options.symmetry_tolerance)
    return reject("symmetric_baseline_torque_required");
  const double h = std::clamp(in.horizon, 0.025, 0.060);

  ThrustSupportDynamicsInput dynamics;
  dynamics.q = state.q; dynamics.v = state.v;
  dynamics.prediction_dt = state.dt;
  dynamics.friction_coefficient = options.wheel_ground_friction;
  GroundUrdfResponseModel model;
  if (!build_ground_urdf_response_model(dynamics, options.profile, model))
    return reject("fixed_ground_model_invalid");
  if(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-fast_started).count()>1000.0)
    return reject("fast_control_deadline_exceeded");
  SupportMatrix9 damping = SupportMatrix9::Zero();
  for (int j = 0; j < 4; ++j) damping(3 + j, 3 + j) = options.leg_viscous;
  damping(7,7) = damping(8,8) = options.wheel_viscous;
  const SupportMatrix9 free_mass = model.M + state.dt * damping;
  Eigen::FullPivLU<SupportMatrix9> free_lu(free_mass);
  if (!free_lu.isInvertible()) return reject("free_stage_mass_singular");
  const SupportMatrix9 force_map = (free_lu.solve(model.M)).transpose();

  // Constrained incremental dynamics use all four physical contact forces.
  // No equality is imposed on left/right normal or tangent load.
  Eigen::Matrix<double, 13, 13> A = Eigen::Matrix<double, 13, 13>::Zero();
  A.block<9,9>(0,0) = model.M;
  A.block<9,4>(0,9) = -model.contactJacobian.transpose();
  A.block<4,9>(9,0) = model.contactJacobian;
  Eigen::JacobiSVD<Eigen::Matrix<double,13,13>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info()!=Eigen::Success || !svd.singularValues().allFinite())
    return reject("mobility_svd_failed");
  double smallest=std::numeric_limits<double>::infinity();
  int rank=0;
  for(int i=0;i<svd.singularValues().size();++i) {
    const double sigma=svd.singularValues()[i];
    if(sigma>options.rank_relative_threshold*svd.singularValues()[0]) { ++rank; smallest=std::min(smallest,sigma); }
  }
  if(rank!=13) return reject("symmetric_mobility_rank_deficient");
  for(int task=0;task<2;++task) {
    Eigen::Matrix<double,13,1> rhs=Eigen::Matrix<double,13,1>::Zero();
    SupportQ9 force=SupportQ9::Zero();
    if(task==0) { force[3]=1.0; force[5]=1.0; }
    else { force[4]=1.0; force[6]=1.0; }
    rhs.head<9>()=force_map*force;
    const Eigen::Matrix<double,13,1> solution=svd.solve(rhs);
    if(!solution.allFinite() || (A*solution-rhs).cwiseAbs().maxCoeff()>options.max_residual)
      return reject("symmetric_mobility_inconsistent");
    out.acceleration_per_task_torque.col(task)=solution.head<9>();
    out.contact_force_per_task_torque.col(task)=solution.segment<4>(9);
    if(task==0) out.task_mobility(0,0)=0.5*(solution[3]+solution[5]);
    else out.task_mobility(0,1)=0.5*(solution[3]+solution[5]);
    if(task==0) out.task_mobility(1,0)=0.5*(solution[4]+solution[6]);
    else out.task_mobility(1,1)=0.5*(solution[4]+solution[6]);
  }
  if(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-fast_started).count()>1000.0)
    return reject("fast_control_deadline_exceeded");
  if(!out.task_mobility.allFinite() || std::abs(out.task_mobility.determinant())<1e-12)
    return reject("task_mobility_singular");
  out.effective_inertia=out.task_mobility.inverse();
  if(!out.effective_inertia.allFinite()) return reject("effective_inertia_nonfinite");
  Eigen::JacobiSVD<Eigen::Matrix2d> mob_svd(out.task_mobility);
  out.mobility_condition=mob_svd.singularValues()[0]/mob_svd.singularValues()[1];

  Eigen::Matrix<double,2,1> error, velocity_error;
  error << 0.5*((in.q_reference[0]-state.q[3])+(in.q_reference[2]-state.q[5])),
           0.5*((in.q_reference[1]-state.q[4])+(in.q_reference[3]-state.q[6]));
  velocity_error << 0.5*((in.v_reference[0]-state.v[3])+(in.v_reference[2]-state.v[5])),
                    0.5*((in.v_reference[1]-state.v[4])+(in.v_reference[3]-state.v[6]));
  Eigen::Matrix2d kp=Eigen::Matrix2d::Zero(), kd=Eigen::Matrix2d::Zero();
  kp.diagonal()<<25.0,45.0; kd.diagonal()<<3.5,6.0;
  out.pd_torque=kp*error+kd*velocity_error;
  Eigen::Matrix2d implicit=out.effective_inertia+h*kd+h*h*kp;
  Eigen::FullPivLU<Eigen::Matrix2d> implicit_lu(implicit);
  if(!implicit.allFinite() || !implicit_lu.isInvertible())
    return reject("implicit_pd_matrix_singular");
  const Eigen::Matrix<double,2,1> rhs=out.pd_torque+h*kp*velocity_error;
  out.implicit_pd_correction=out.effective_inertia*implicit_lu.solve(rhs);
  if(!out.implicit_pd_correction.allFinite()) return reject("implicit_pd_nonfinite");
  auto reference=ground_contact_motion_inverse(state,options);
  if(!reference.valid) return reject(reference.reason.c_str());
  const double elapsed_before_certificate=std::chrono::duration<double,std::micro>(
      std::chrono::steady_clock::now()-fast_started).count();
  const double remaining_budget=1000.0-elapsed_before_certificate;
  if(remaining_budget<=0.0) return reject("fast_control_deadline_exceeded");
  out.reference_inverse=reference;
  const Eigen::Matrix<double,4,1> unblended=out.reference_inverse.leg_torque+
      (Eigen::Matrix<double,4,1>() << out.implicit_pd_correction[0],out.implicit_pd_correction[1],
                                      out.implicit_pd_correction[0],out.implicit_pd_correction[1]).finished();
  out.leg_torque=(1.0-in.command_blend_alpha)*in.baseline_leg_torque+
      in.command_blend_alpha*unblended;
  for(int j=0;j<4;++j)
    if(std::abs(out.leg_torque[j])>options.leg_effort_limit[j]+1e-9)
      return reject("leg_effort_limit_exceeded");
  // Certify the exact alpha-blended final leg command with the bounded fixed-mode solver.
  GroundContactFastCertificateInput final_input;
  final_input.q=state.q; final_input.v=state.v;
  final_input.command << out.leg_torque[0],out.leg_torque[1],out.leg_torque[2],
      out.leg_torque[3],state.wheel_torque[0],state.wheel_torque[1];
  final_input.dt=state.dt;
  final_input.bilateral_contact=state.bilateral_contact;
  final_input.wheel_ground_friction=options.wheel_ground_friction;
  final_input.remaining_budget_us=remaining_budget;
  const auto forward=ground_contact_motion_fast_certificate(final_input,options);
  if(!forward.valid) return reject(forward.reason.c_str());
  out.predicted_acceleration=forward.acceleration;
  out.predicted_contact_force=forward.contact_force;
  out.predicted_leg_friction_torque=forward.leg_friction_torque;
  out.predicted_leg_friction_mode=forward.leg_mode;
  out.maximum_static_friction_effort=forward.max_stick_effort;
  out.final_equation_residual=forward.equation_residual;
  out.final_contact_residual=forward.contact_residual;
  out.final_distinct_accelerations=forward.distinct_accelerations;
  out.final_candidate_modes=forward.feasible_candidate_count;
  out.final_boundary_redundant_modes=0;
  out.contact_cone_violation=forward.max_contact_cone_violation;
  out.fast_candidate_count=forward.candidate_count;
  out.fast_final_kkt_rank=forward.kkt_rank;
  out.fast_final_kkt_dimension=forward.kkt_dimension;
  out.fast_final_elapsed_us=forward.elapsed_us;
  out.predicted_next_velocity=state.v+state.dt*out.predicted_acceleration;
  out.predicted_next_position=state.q+state.dt*out.predicted_next_velocity;
  if(!out.predicted_acceleration.allFinite() || !out.predicted_next_velocity.allFinite() ||
      !out.predicted_next_position.allFinite())
    return reject("predicted_state_nonfinite");
  out.predicted_pitch_delta=out.predicted_next_position[2]-in.pitch_anchor;
  out.predicted_hip_acceleration_difference=out.predicted_acceleration[3]-out.predicted_acceleration[5];
  out.predicted_knee_acceleration_difference=out.predicted_acceleration[4]-out.predicted_acceleration[6];
  out.predicted_hip_position_difference=out.predicted_next_position[3]-out.predicted_next_position[5];
  out.predicted_knee_position_difference=out.predicted_next_position[4]-out.predicted_next_position[6];
  out.predicted_hip_rate_difference=out.predicted_next_velocity[3]-out.predicted_next_velocity[5];
  out.predicted_knee_rate_difference=out.predicted_next_velocity[4]-out.predicted_next_velocity[6];
  out.predicted_soft_margin=std::numeric_limits<double>::infinity();
  constexpr double qmin[4]={-1.52,-1.56,-1.52,-1.56};
  constexpr double qmax[4]={1.52,1.56,1.52,1.56};
  for(int j=0;j<4;++j) {
    const double qn=out.predicted_next_position[3+j];
    out.predicted_soft_margin=std::min(out.predicted_soft_margin,
        std::min(qn-qmin[j],qmax[j]-qn));
    if(std::abs(qn-in.joint_anchor[j])>0.08)
      return reject("predicted_joint_anchor_guard");
    if(std::min(qn-qmin[j],qmax[j]-qn)<0.30)
      return reject("predicted_joint_soft_limit_guard");
  }
  if(std::abs(out.predicted_pitch_delta)>0.10)
    return reject("predicted_pitch_guard");
  if(std::abs(out.predicted_next_velocity[2])>options.max_pitch_rate)
    return reject("predicted_pitch_rate_guard");
  for(int j=3;j<7;++j)
    if(std::abs(out.predicted_next_velocity[j])>options.max_leg_rate)
      return reject("predicted_leg_rate_guard");
  if(std::abs(out.predicted_next_velocity[7])>options.max_wheel_rate ||
     std::abs(out.predicted_next_velocity[8])>options.max_wheel_rate)
    return reject("predicted_wheel_rate_guard");
  if(state.dt*std::abs(out.predicted_hip_acceleration_difference)>options.symmetry_tolerance ||
     state.dt*std::abs(out.predicted_knee_acceleration_difference)>options.symmetry_tolerance ||
     std::abs(out.predicted_hip_position_difference)>
         options.symmetry_tolerance ||
     std::abs(out.predicted_knee_position_difference)>
         options.symmetry_tolerance ||
     std::abs(out.predicted_hip_rate_difference)>
         options.symmetry_tolerance ||
     std::abs(out.predicted_knee_rate_difference)>
         options.symmetry_tolerance)
    return reject("predicted_pair_synchronization_guard");
  out.elapsed_us=std::chrono::duration<double,std::micro>(
      std::chrono::steady_clock::now()-fast_started).count();
  if(out.elapsed_us>1000.0) return reject("fast_control_deadline_exceeded");
  out.valid=true; out.reason="ok";
  return out;
}

} // namespace bbot_jump
