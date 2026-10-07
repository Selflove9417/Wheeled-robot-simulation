#pragma once

// Offline, fixed-model inverse dynamics for the symmetric bilateral-ground
// motion experiment. This is not connected to the controller.
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "bbot_balance_controller/ground_urdf_response_model.hpp"
#include "bbot_balance_controller/ground_friction_response.hpp"

namespace bbot_jump {

struct GroundContactMotionInput {
  SupportQ9 q = SupportQ9::Zero();
  SupportQ9 v = SupportQ9::Zero();
  Eigen::Matrix<double, 4, 1> leg_acceleration = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::Matrix<double, 2, 1> wheel_torque = Eigen::Matrix<double, 2, 1>::Zero();
  std::array<int, 4> friction_mode_hint{{2,2,2,2}}; // -1/+1 slip, 0 static-compatible, 2 infer.
  double dt = 0.001;
  bool bilateral_contact = true;
};

struct GroundContactMotionOptions {
  GroundUrdfResponseProfile profile = GroundUrdfResponseProfile::URDFGravityAndFixedIMU;
  double static_friction = 0.1;
  double dynamic_friction = 0.1;
  double leg_viscous = 0.5;
  double wheel_viscous = 0.1;
  double wheel_ground_friction = 1.0;
  double symmetry_tolerance = 1e-6;
  double max_residual = 1e-8;
  double rank_relative_threshold = 1e-12;
  double slip_velocity_tolerance = 1e-10;
  double max_leg_rate = 0.10;
  double max_pitch_rate = 0.50;
  double max_wheel_rate = 30.0;
  std::array<double, 4> leg_effort_limit{{75.0, 60.0, 75.0, 60.0}};
  double wheel_effort_limit = 10.0;
};

struct GroundContactMotionResult {
  bool valid{false};
  std::string reason{"uninitialized"};
  SupportQ9 acceleration = SupportQ9::Zero();
  SupportContactVector contact_force = SupportContactVector::Zero();
  Eigen::Matrix<double, 4, 1> leg_torque = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::Matrix<double, 4, 1> leg_friction_torque = Eigen::Matrix<double, 4, 1>::Zero();
  std::array<int, 4> leg_friction_mode{{0, 0, 0, 0}}; // -1/0/+1: slip/zero-friction/stick-compatible.
  Eigen::Matrix<double, 4, 1> final_leg_friction_torque = Eigen::Matrix<double, 4, 1>::Zero();
  std::array<int, 4> final_leg_friction_mode{{0, 0, 0, 0}};
  double final_static_friction_effort{0.0};
  int final_distinct_accelerations{0};
  double equation_residual{std::numeric_limits<double>::infinity()};
  double contact_residual{std::numeric_limits<double>::infinity()};
  double symmetry_residual{std::numeric_limits<double>::infinity()};
  double smallest_kept_singular_value{0.0};
  double largest_singular_value{0.0};
  int rank{0};
};

inline bool ground_contact_motion_options_valid(const GroundContactMotionOptions &o,
                                                const char **reason) {
  const auto fail=[&](const char *why){ if(reason) *reason=why; return false; };
  if(o.profile!=GroundUrdfResponseProfile::URDFGravityAndFixedIMU)
    return fail("requires_verified_urdf_response_profile");
  const double scalars[]={o.static_friction,o.dynamic_friction,o.leg_viscous,
      o.wheel_viscous,o.wheel_ground_friction,o.symmetry_tolerance,o.max_residual,
      o.rank_relative_threshold,o.slip_velocity_tolerance,o.max_leg_rate,
      o.max_pitch_rate,o.max_wheel_rate,o.wheel_effort_limit};
  for(double x:scalars) if(!std::isfinite(x)) return fail("nonfinite_option");
  for(double x:o.leg_effort_limit) if(!std::isfinite(x)) return fail("nonfinite_option");
  if(o.static_friction!=0.1 || o.dynamic_friction!=0.1 || o.leg_viscous!=0.5 ||
     o.wheel_viscous!=0.1 || o.wheel_ground_friction!=1.0)
    return fail("fixed_physics_parameters_required");
  if(o.symmetry_tolerance!=1e-6 || o.max_residual!=1e-8 ||
     o.rank_relative_threshold!=1e-12 || o.slip_velocity_tolerance!=1e-10)
    return fail("physical_or_error_limit_cannot_be_relaxed");
  if(o.max_leg_rate<=0.0 || o.max_leg_rate>0.10 ||
     o.max_pitch_rate<=0.0 || o.max_pitch_rate>0.50 ||
     o.max_wheel_rate<=0.0 || o.max_wheel_rate>30.0 ||
     o.wheel_effort_limit<=0.0 || o.wheel_effort_limit>10.0)
    return fail("actuator_or_rate_limit_cannot_be_relaxed");
  constexpr double limits[4]={75.0,60.0,75.0,60.0};
  for(int i=0;i<4;++i)
    if(o.leg_effort_limit[i]<=0.0 || o.leg_effort_limit[i]>limits[i])
      return fail("actuator_or_rate_limit_cannot_be_relaxed");
  if(reason) *reason="ok";
  return true;
}

inline bool ground_contact_motion_symmetric(const GroundContactMotionInput &in,
                                             double tolerance,
                                             double &residual) {
  residual = std::max({
      std::abs(in.q[3] - in.q[5]), std::abs(in.q[4] - in.q[6]),
      std::abs(in.v[3] - in.v[5]), std::abs(in.v[4] - in.v[6]),
      std::abs(in.leg_acceleration[0] - in.leg_acceleration[2]),
      std::abs(in.leg_acceleration[1] - in.leg_acceleration[3]),
      std::abs(in.wheel_torque[0] - in.wheel_torque[1])});
  return std::isfinite(residual) && residual <= tolerance;
}

// Solve a square 17x17 inverse system. Two mean joint-acceleration task rows
// plus the explicit equal-actuator policy close the system. The bilateral
// contact loads remain free to differ, and the raw left/right q/v are retained
// in M,C,G,J. A rank-deficient or inconsistent system is rejected; no
// projection or minimum-norm regularizer is used.
inline GroundContactMotionResult ground_contact_motion_inverse(
    const GroundContactMotionInput &in,
    const GroundContactMotionOptions &options = GroundContactMotionOptions{}) {
  GroundContactMotionResult out;
  const auto reject = [&](const char *reason) {
    out.valid = false;
    out.reason = reason;
    return out;
  };
  const char *option_reason=nullptr;
  if(!ground_contact_motion_options_valid(options,&option_reason)) return reject(option_reason);
  if (!in.q.allFinite() || !in.v.allFinite() || !in.leg_acceleration.allFinite() ||
      !in.wheel_torque.allFinite() || !std::isfinite(in.dt) ||
      !std::isfinite(options.static_friction) || !std::isfinite(options.dynamic_friction) ||
      !std::isfinite(options.leg_viscous) || !std::isfinite(options.wheel_viscous) ||
      !std::isfinite(options.symmetry_tolerance) || !std::isfinite(options.max_residual))
    return reject("nonfinite_input");
  if (!in.bilateral_contact) return reject("bilateral_contact_required");
  if (std::abs(in.dt - 0.001) > 1e-12) return reject("requires_1ms_physics_model_step");
  if (options.profile != GroundUrdfResponseProfile::URDFGravityAndFixedIMU)
    return reject("requires_verified_urdf_response_profile");
  if (options.static_friction < 0.0 || options.dynamic_friction < 0.0 ||
      options.leg_viscous < 0.0 || options.wheel_viscous < 0.0 ||
      options.wheel_ground_friction < 0.0 || options.wheel_effort_limit <= 0.0)
    return reject("invalid_physical_limits");
  for (int j = 0; j < 4; ++j)
    if (!std::isfinite(options.leg_effort_limit[j]) || options.leg_effort_limit[j] <= 0.0)
      return reject("invalid_leg_effort_limit");
  if (options.static_friction > 0.1 || options.dynamic_friction > 0.1 ||
      options.leg_viscous > 0.5 || options.wheel_viscous > 0.1 ||
      options.wheel_ground_friction > 1.0 || options.wheel_effort_limit > 10.0 ||
      options.symmetry_tolerance > 1e-6 || options.max_residual > 1e-8 ||
      options.rank_relative_threshold > 1e-12 || options.slip_velocity_tolerance > 1e-10)
    return reject("physical_or_error_limit_cannot_be_relaxed");
  if ((in.wheel_torque.array().abs() > options.wheel_effort_limit + 1e-10).any())
    return reject("wheel_effort_limit_exceeded");
  (void)ground_contact_motion_symmetric(in, options.symmetry_tolerance,
                                        out.symmetry_residual);

  ThrustSupportDynamicsInput dynamics;
  dynamics.q = in.q;
  dynamics.v = in.v;
  dynamics.prediction_dt = in.dt;
  dynamics.friction_coefficient = options.wheel_ground_friction;
  GroundUrdfResponseModel model;
  if (!build_ground_urdf_response_model(dynamics, options.profile, model))
    return reject("fixed_ground_model_invalid");

  // DART's first (free) stage uses implicit viscous damping. The subsequent
  // contact/Coulomb stage applies impulses through ordinary M, not the damped
  // matrix. K maps generalized force in the free stage to M*a_free.
  SupportMatrix9 damping = SupportMatrix9::Zero();
  for (int j = 0; j < 4; ++j) damping(3 + j, 3 + j) = options.leg_viscous;
  damping(7, 7) = options.wheel_viscous;
  damping(8, 8) = options.wheel_viscous;
  const SupportMatrix9 free_mass = model.M + in.dt * damping;
  Eigen::FullPivLU<SupportMatrix9> free_lu(free_mass);
  if (!free_mass.allFinite() || !free_lu.isInvertible())
    return reject("free_stage_mass_singular");
  const SupportMatrix9 force_map = (free_lu.solve(model.M)).transpose();

  // For the requested one-step leg motion, select the Coulomb slip direction
  // from the target next velocity. At an exactly stationary zero-acceleration
  // target, use the admissible zero member of the static-friction interval;
  // no hidden torque floor or sign(v~0) convention is applied.
  for (int j = 0; j < 4; ++j) {
    const double next_v = in.v[3 + j] + in.dt * in.leg_acceleration[j];
    int mode=in.friction_mode_hint[j];
    if(mode==2) mode=std::abs(next_v)>options.slip_velocity_tolerance?(next_v>0.0?1:-1):0;
    if(mode < -1 || mode > 1) return reject("invalid_friction_mode_hint");
    if(mode==0) {
      if(std::abs(next_v)>options.slip_velocity_tolerance)
        return reject("static_friction_mode_incompatible_with_target_acceleration");
      out.leg_friction_mode[j]=0;
      out.leg_friction_torque[j]=0.0; // explicit admissible zero member of static set.
    } else {
      out.leg_friction_mode[j]=mode;
      out.leg_friction_torque[j]=-options.dynamic_friction*mode;
    }
  }

  // Unknowns x=[a(9), lambda(4), u_leg(4)]. Contact order is
  // [left tangent, left normal, right tangent, right normal].
  // Two mean-task acceleration constraints plus the explicit equal-actuator
  // policy make this a square system. Raw per-side q/v remain in model M/C/G/J;
  // no state averaging and no contact-load equality are imposed.
  Eigen::Matrix<double, 17, 17> A = Eigen::Matrix<double, 17, 17>::Zero();
  Eigen::Matrix<double, 17, 1> b = Eigen::Matrix<double, 17, 1>::Zero();
  A.block<9, 9>(0, 0) = model.M;
  A.block<9, 4>(0, 9) = -model.contactJacobian.transpose();
  for (int j = 0; j < 4; ++j) A.block<9, 1>(0, 13 + j) = -force_map.col(3 + j);

  SupportQ9 known_force = -model.C - model.G - damping * in.v;
  known_force[7] += in.wheel_torque[0];
  known_force[8] += in.wheel_torque[1];
  const SupportQ9 rhs_dynamic = force_map * known_force +
      SupportQ9((SupportQ9() << 0,0,0,
          out.leg_friction_torque[0], out.leg_friction_torque[1],
          out.leg_friction_torque[2], out.leg_friction_torque[3],0,0).finished());
  b.head<9>() = rhs_dynamic;
  A.block<4, 9>(9, 0) = model.contactJacobian;
  b.segment<4>(9) = -model.contactBias;
  A(13, 3) = 0.5; A(13, 5) = 0.5;
  b[13] = 0.5 * (in.leg_acceleration[0] + in.leg_acceleration[2]);
  A(14, 4) = 0.5; A(14, 6) = 0.5;
  b[14] = 0.5 * (in.leg_acceleration[1] + in.leg_acceleration[3]);
  A(15, 13) = 1.0; A(15, 15) = -1.0;
  A(16, 14) = 1.0; A(16, 16) = -1.0;

  Eigen::JacobiSVD<Eigen::Matrix<double, 17, 17>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success || !svd.singularValues().allFinite())
    return reject("inverse_svd_failed");
  const double largest = svd.singularValues()[0];
  const double cutoff = options.rank_relative_threshold * largest;
  for (int i = 0; i < svd.singularValues().size(); ++i)
    if (svd.singularValues()[i] > cutoff) {
      ++out.rank;
      out.smallest_kept_singular_value = svd.singularValues()[i];
    }
  out.largest_singular_value = largest;
  if (out.rank != 17) return reject("symmetric_inverse_rank_deficient");
  const Eigen::Matrix<double, 17, 1> x = svd.solve(b);
  if (!x.allFinite()) return reject("inverse_solution_nonfinite");
  out.equation_residual = (A * x - b).cwiseAbs().maxCoeff();
  if (out.equation_residual > options.max_residual)
    return reject("symmetric_inverse_inconsistent");
  out.acceleration = x.head<9>();
  out.contact_force = x.segment<4>(9);
  out.leg_torque = x.tail<4>();
  out.contact_residual =
      (model.contactJacobian * out.acceleration + model.contactBias).cwiseAbs().maxCoeff();
  if (!std::isfinite(out.contact_residual) || out.contact_residual > options.max_residual)
    return reject("contact_constraint_residual_exceeded");
  for (int j = 0; j < 4; ++j)
    if (std::abs(out.leg_torque[j]) > options.leg_effort_limit[j] + 1e-9)
      return reject("leg_effort_limit_exceeded");
  for (int side = 0; side < 2; ++side) {
    const double tangent = out.contact_force[2 * side];
    const double normal = out.contact_force[2 * side + 1];
    if (normal < -1e-7 || std::abs(tangent) > options.wheel_ground_friction *
        std::max(0.0, normal) + 1e-7)
      return reject("contact_cone_violated");
  }
  out.valid = true;
  out.reason = "ok";
  return out;
}

struct GroundContactMotionControlInput {
  GroundContactMotionInput state_and_reference_acceleration;
  Eigen::Matrix<double, 4, 1> q_reference = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::Matrix<double, 4, 1> v_reference = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::Matrix<double, 4, 1> joint_anchor = Eigen::Matrix<double, 4, 1>::Zero();
  double pitch_anchor = 0.0; // CAD theta; controller pitch has opposite sign.
  double horizon = 0.025;
  // Optional 2 s handoff is supplied by the caller as a bounded blend from
  // its measured baseline leg command. Alpha=1 preserves the standalone
  // helper output. The baseline must remain on the symmetric task subspace.
  double command_blend_alpha = 1.0;
  Eigen::Matrix<double, 4, 1> baseline_leg_torque = Eigen::Matrix<double, 4, 1>::Zero();
};

struct GroundContactMotionControlResult {
  bool valid{false};
  std::string reason{"uninitialized"};
  GroundContactMotionResult reference_inverse;
  Eigen::Matrix2d task_mobility = Eigen::Matrix2d::Zero(); // (hip,knee) acceleration / (per-leg Nm)
  // Inverse discrete task mobility. It can be nonsymmetric because the
  // implicit free-stage damping map M(M+dtD)^-1 is nonsymmetric.
  Eigen::Matrix2d effective_inertia = Eigen::Matrix2d::Zero();
  Eigen::Matrix<double, 9, 2> acceleration_per_task_torque = Eigen::Matrix<double, 9, 2>::Zero();
  Eigen::Matrix<double, 4, 2> contact_force_per_task_torque = Eigen::Matrix<double, 4, 2>::Zero();
  Eigen::Matrix<double, 2, 1> pd_torque = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 2, 1> implicit_pd_correction = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 4, 1> leg_torque = Eigen::Matrix<double, 4, 1>::Zero();
  SupportQ9 predicted_acceleration = SupportQ9::Zero();
  SupportQ9 predicted_next_position = SupportQ9::Zero();
  SupportQ9 predicted_next_velocity = SupportQ9::Zero();
  SupportContactVector predicted_contact_force = SupportContactVector::Zero();
  Eigen::Matrix<double, 4, 1> predicted_leg_friction_torque = Eigen::Matrix<double, 4, 1>::Zero();
  std::array<int, 4> predicted_leg_friction_mode{{0,0,0,0}};
  double maximum_static_friction_effort{0.0};
  double final_equation_residual{std::numeric_limits<double>::infinity()};
  double final_contact_residual{std::numeric_limits<double>::infinity()};
  int final_distinct_accelerations{0};
  int final_candidate_modes{0};
  int final_boundary_redundant_modes{0};
  double contact_cone_violation{std::numeric_limits<double>::infinity()};
  double predicted_pitch_delta = std::numeric_limits<double>::quiet_NaN();
  double predicted_soft_margin = std::numeric_limits<double>::quiet_NaN();
  double predicted_hip_acceleration_difference = std::numeric_limits<double>::quiet_NaN();
  double predicted_knee_acceleration_difference = std::numeric_limits<double>::quiet_NaN();
  double predicted_hip_position_difference = std::numeric_limits<double>::quiet_NaN();
  double predicted_knee_position_difference = std::numeric_limits<double>::quiet_NaN();
  double predicted_hip_rate_difference = std::numeric_limits<double>::quiet_NaN();
  double predicted_knee_rate_difference = std::numeric_limits<double>::quiet_NaN();
  double mobility_condition = std::numeric_limits<double>::infinity();
};

// Contact-constrained torque feedback for the two symmetric tasks (common hip
// and common knee). Kp/Kd retain their original torque units; aref supplies
// inverse-dynamics feedforward. The 2x2 mobility is measured from constrained
// dynamics, not from the free-flight inertia approximation.
inline GroundContactMotionControlResult ground_contact_motion_control(
    const GroundContactMotionControlInput &in,
    const GroundContactMotionOptions &options = GroundContactMotionOptions{}) {
  GroundContactMotionControlResult out;
  const auto reject = [&](const char *why) {
    out.valid = false; out.reason = why; return out;
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
  if(!reference.valid) { out.reason=reference.reason; return out; }
  out.reference_inverse=reference;
  const Eigen::Matrix<double,4,1> unblended=out.reference_inverse.leg_torque+
      (Eigen::Matrix<double,4,1>() << out.implicit_pd_correction[0],out.implicit_pd_correction[1],
                                      out.implicit_pd_correction[0],out.implicit_pd_correction[1]).finished();
  out.leg_torque=(1.0-in.command_blend_alpha)*in.baseline_leg_torque+
      in.command_blend_alpha*unblended;
  for(int j=0;j<4;++j)
    if(std::abs(out.leg_torque[j])>options.leg_effort_limit[j]+1e-9)
      return reject("leg_effort_limit_exceeded");
  // Certify the actual final command (including the alpha handoff) against all
  // 81 static/sliding friction assignments. Static mode solves its reaction
  // as an unknown set-valued force; it is not assumed to be zero and no
  // epsilon-sign iteration is used.
  GroundFrictionResponseInput final_input;
  final_input.q=state.q; final_input.v=state.v;
  final_input.command << out.leg_torque[0],out.leg_torque[1],out.leg_torque[2],
      out.leg_torque[3],state.wheel_torque[0],state.wheel_torque[1];
  final_input.dt=state.dt;
  final_input.bilateral_contact=state.bilateral_contact;
  final_input.wheel_ground_friction=options.wheel_ground_friction;
  GroundFrictionResponseOptions final_options;
  final_options.response_profile=options.profile;
  final_options.static_friction=options.static_friction;
  final_options.dynamic_friction=options.dynamic_friction;
  final_options.leg_viscous=options.leg_viscous;
  final_options.wheel_viscous=options.wheel_viscous;
  final_options.max_contact_friction=options.wheel_ground_friction;
  // Keep the independent full-81 response's original branch boundary (1e-9).
  // The inverse target's hint threshold does not certify final friction mode.
  final_options.max_abs_equation_residual=options.max_residual;
  final_options.contact_residual_tolerance=options.max_residual;
  for(int i=0;i<4;++i) final_options.input_effort_limits[i]=options.leg_effort_limit[i];
  final_options.input_effort_limits[4]=options.wheel_effort_limit;
  final_options.input_effort_limits[5]=options.wheel_effort_limit;
  const GroundFrictionResponseResult forward=ground_friction_response(final_input,final_options);
  if(!forward.valid) return reject("final_blended_command_friction_rejected");
  if(!std::isfinite(forward.max_equation_residual) ||
     forward.max_equation_residual>options.max_residual ||
     !std::isfinite(forward.max_contact_residual) ||
     forward.max_contact_residual>options.max_residual)
    return reject("final_blended_command_residual_exceeded");
  if(forward.distinct_accelerations!=1) return reject("final_blended_command_response_ambiguous");
  out.predicted_acceleration=forward.acceleration;
  out.predicted_contact_force=forward.contact_force;
  out.predicted_leg_friction_torque=forward.leg_friction_torque;
  out.predicted_leg_friction_mode=forward.leg_mode;
  out.maximum_static_friction_effort=forward.max_stick_effort;
  out.final_equation_residual=forward.max_equation_residual;
  out.final_contact_residual=forward.max_contact_residual;
  out.final_distinct_accelerations=forward.distinct_accelerations;
  out.final_candidate_modes=forward.candidate_modes;
  out.final_boundary_redundant_modes=forward.boundary_redundant_modes;
  out.contact_cone_violation=forward.max_contact_cone_violation;
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
  out.valid=true; out.reason="ok";
  return out;
}

} // namespace bbot_jump
