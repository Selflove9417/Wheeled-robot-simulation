#pragma once

// Offline response-model comparison for the flat_jump_world URDF. This helper
// is deliberately separate from ThrustSupportDynamicsModel's COM/H/task and
// optimizer fields, and is not a controller model.
#include <cmath>
#include <cstdint>

#include "bbot_balance_controller/thrust_support_dynamics.hpp"

namespace bbot_jump {

constexpr double kGroundResponseBaselineGravity = 9.81;
constexpr double kGroundResponseWorldGravity = 9.8;
constexpr double kGroundResponseImuMass = 0.01;
constexpr double kGroundResponseImuIxx = 1e-6;

enum class GroundUrdfResponseProfile : uint8_t {
  OriginalCAD = 0,
  GravityOnly = 1,
  URDFGravityAndFixedIMU = 2,
};

struct GroundUrdfResponseModel {
  bool valid{false};
  SupportMatrix9 M = SupportMatrix9::Zero();
  SupportQ9 C = SupportQ9::Zero();  // Coriolis/centripetal velocity bias; no friction/damping.
  SupportQ9 G = SupportQ9::Zero();
  SupportContactJacobian contactJacobian = SupportContactJacobian::Zero();
  SupportContactVector contactBias = SupportContactVector::Zero();
};

// Build one of three fixed-source profiles, with no fitted physical parameters.
// OriginalCAD is the original 9.81 m/s^2, no-IMU model. GravityOnly changes
// only to 9.8 m/s^2. URDFGravityAndFixedIMU applies the native
// flat_jump_world SDF default gravity (9.8 m/s^2) and the URDF's fixed 0.01 kg
// imu_link at base-link xyz=(0.20, 0.1326, 0.054), Ixx=1e-6 kg m^2. The 9.8
// default is independently consistent with the first free-fall NCS frame in
// the ground-input native trace; this is specific to that SDF/world.
//
// The URDF variant changes only M, C, and G. Contact geometry and rolling
// assumptions remain those of the existing candidate model. It intentionally
// does not expose or certify COM/H, contact adequacy, tasks, or optimizer
// feasibility.
inline bool build_ground_urdf_response_model(
    const ThrustSupportDynamicsInput &input,
    GroundUrdfResponseProfile profile,
    GroundUrdfResponseModel &out)
{
  out = GroundUrdfResponseModel{};
  if (profile != GroundUrdfResponseProfile::OriginalCAD &&
      profile != GroundUrdfResponseProfile::GravityOnly &&
      profile != GroundUrdfResponseProfile::URDFGravityAndFixedIMU) return false;
  ThrustSupportDynamicsInput configured = input;
  configured.gravity = profile != GroundUrdfResponseProfile::OriginalCAD
      ? kGroundResponseWorldGravity : kGroundResponseBaselineGravity;

  ThrustSupportDynamicsModel original;
  if (!thrust_support_dynamics_model(configured, original)) return false;

  out.M = original.mass;
  out.C = original.velocity_bias;
  out.G = original.gravity;
  out.contactJacobian = original.contact_jacobian;
  out.contactBias = original.contact_bias;

  if (profile == GroundUrdfResponseProfile::URDFGravityAndFixedIMU) {
    // The planar coordinates are [world-Y, world-Z, theta, ...]. Xacro xyz is
    // expressed in base_link; only its Y/Z coordinates enter this planar point.
    const Eigen::Vector2d imuLocal(0.1326, 0.054);
    const Eigen::Vector2d imuWorld = support_rotate(input.q[2], imuLocal);
    Eigen::Matrix<double, 2, 9> J = Eigen::Matrix<double, 2, 9>::Zero();
    J(0, 0) = 1.0;
    J(1, 1) = 1.0;
    J.col(2) = support_perp(imuWorld);

    out.M.noalias() += kGroundResponseImuMass * J.transpose() * J;
    SupportQ9 thetaAxis = SupportQ9::Zero();
    thetaAxis[2] = 1.0;
    out.M.noalias() += kGroundResponseImuIxx * thetaAxis * thetaAxis.transpose();

    const Eigen::Vector2d centripetal = -imuWorld * (input.v[2] * input.v[2]);
    out.C.noalias() += kGroundResponseImuMass * J.transpose() * centripetal;
    out.G.noalias() += kGroundResponseImuMass * kGroundResponseWorldGravity *
                       J.row(1).transpose();
  }

  out.valid = out.M.allFinite() && out.C.allFinite() && out.G.allFinite() &&
              out.contactJacobian.allFinite() && out.contactBias.allFinite();
  return out.valid;
}

// Convenience API for callers that only compare the original and full URDF
// source profile. false is OriginalCAD; true is URDFGravityAndFixedIMU.
inline bool build_ground_urdf_response_model(
    const ThrustSupportDynamicsInput &input,
    bool use_urdf_world_and_fixed_imu,
    GroundUrdfResponseModel &out)
{
  return build_ground_urdf_response_model(input,
      use_urdf_world_and_fixed_imu
          ? GroundUrdfResponseProfile::URDFGravityAndFixedIMU
          : GroundUrdfResponseProfile::OriginalCAD,
      out);
}

}  // namespace bbot_jump
