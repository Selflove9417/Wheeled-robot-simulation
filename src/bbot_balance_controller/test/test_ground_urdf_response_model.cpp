// Offline unit coverage for the isolated URDF response correction.
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Eigenvalues>

#include "bbot_balance_controller/ground_urdf_response_model.hpp"

namespace {
using namespace bbot_jump;

Eigen::Matrix<double, 2, 9> imu_jacobian(const SupportQ9 &q)
{
  const Eigen::Vector2d rotated = support_rotate(q[2], {0.1326, 0.054});
  Eigen::Matrix<double, 2, 9> J = Eigen::Matrix<double, 2, 9>::Zero();
  J(0, 0) = 1.0;
  J(1, 1) = 1.0;
  J.col(2) = support_perp(rotated);
  return J;
}

double delta_potential(const SupportQ9 &q, double bodyMass)
{
  const double oldMass = bodyMass + 8.0;
  const Eigen::Vector2d oldCom = thrust_support_com(q, bodyMass);
  const Eigen::Vector2d imu = support_rotate(q[2], {0.1326, 0.054});
  const double base = kGroundResponseBaselineGravity * oldMass * oldCom.y();
  const double urdf = kGroundResponseWorldGravity *
      (oldMass * oldCom.y() + kGroundResponseImuMass * (q[1] + imu.y()));
  return urdf - base;
}

double gravity_only_delta_potential(const SupportQ9 &q, double bodyMass)
{
  const double oldMass = bodyMass + 8.0;
  const Eigen::Vector2d oldCom = thrust_support_com(q, bodyMass);
  return (kGroundResponseWorldGravity - kGroundResponseBaselineGravity) *
         oldMass * oldCom.y();
}
}  // namespace

int main()
{
  using namespace bbot_jump;
  ThrustSupportDynamicsInput input;
  input.q << 0.31, 0.42, -0.071, 0.255, -0.37, 0.256, -0.369, -4.0, 5.0;
  input.v << 0.02, -0.01, 0.37, 0.12, -0.08, -0.10, 0.05, -0.30, 0.20;

  GroundUrdfResponseModel baseline, gravityOnly, urdf;
  assert(build_ground_urdf_response_model(input, false, baseline));
  assert(build_ground_urdf_response_model(input, true, urdf));
  assert(build_ground_urdf_response_model(input,
      GroundUrdfResponseProfile::GravityOnly, gravityOnly));
  assert(baseline.valid && gravityOnly.valid && urdf.valid);
  assert((baseline.M - baseline.M.transpose()).cwiseAbs().maxCoeff() < 1e-12);
  assert((gravityOnly.M - gravityOnly.M.transpose()).cwiseAbs().maxCoeff() < 1e-12);
  assert((urdf.M - urdf.M.transpose()).cwiseAbs().maxCoeff() < 1e-12);
  Eigen::SelfAdjointEigenSolver<SupportMatrix9> eigBase(baseline.M), eigGravity(gravityOnly.M), eigUrdf(urdf.M);
  assert(eigBase.info() == Eigen::Success && eigBase.eigenvalues().minCoeff() > 0.0);
  assert(eigGravity.info() == Eigen::Success && eigGravity.eigenvalues().minCoeff() > 0.0);
  assert(eigUrdf.info() == Eigen::Success && eigUrdf.eigenvalues().minCoeff() > 0.0);
  assert((gravityOnly.M - baseline.M).norm() == 0.0);
  assert((gravityOnly.C - baseline.C).norm() == 0.0);
  assert((gravityOnly.G - urdf.G +
          kGroundResponseImuMass * kGroundResponseWorldGravity * imu_jacobian(input.q).row(1).transpose()).norm() < 1e-12);

  // The baseline is exactly the existing response at its explicit 9.81 default.
  ThrustSupportDynamicsInput oldInput = input;
  oldInput.gravity = kGroundResponseBaselineGravity;
  ThrustSupportDynamicsModel oldModel;
  assert(thrust_support_dynamics_model(oldInput, oldModel));
  assert((baseline.M - oldModel.mass).cwiseAbs().maxCoeff() < 1e-12);
  assert((baseline.C - oldModel.velocity_bias).norm() < 1e-12);
  assert((baseline.G - oldModel.gravity).norm() < 1e-12);

  // Isolated fixed-IMU mass matrix increment: m J'J + Ixx e_theta e_theta'.
  const Eigen::Matrix<double, 2, 9> J = imu_jacobian(input.q);
  SupportQ9 thetaAxis = SupportQ9::Zero();
  thetaAxis[2] = 1.0;
  const SupportMatrix9 expectedDeltaM = kGroundResponseImuMass * J.transpose() * J +
      kGroundResponseImuIxx * thetaAxis * thetaAxis.transpose();
  assert(((urdf.M - baseline.M) - expectedDeltaM).cwiseAbs().maxCoeff() < 1e-12);

  // Independent point-trajectory kinetic energy verifies the mass increment
  // without building the same analytic Jacobian as the implementation.
  const auto imuPosition = [](const SupportQ9 &q) -> Eigen::Vector2d {
    return Eigen::Vector2d(q[0], q[1]) + support_rotate(q[2], {0.1326, 0.054});
  };
  constexpr double energyStep = 1e-5;
  const Eigen::Vector2d measuredPointVelocity =
      (imuPosition(input.q + energyStep * input.v) -
       imuPosition(input.q - energyStep * input.v)) / (2.0 * energyStep);
  const double trajectoryEnergy = 0.5 * kGroundResponseImuMass * measuredPointVelocity.squaredNorm() +
      0.5 * kGroundResponseImuIxx * input.v[2] * input.v[2];
  const double matrixEnergy = 0.5 * input.v.dot((urdf.M - baseline.M) * input.v);
  assert(std::abs(trajectoryEnergy - matrixEnergy) < 1e-10);

  // Verify the C increment with a finite-difference centripetal acceleration
  // of the fixed point along theta(t)=theta0+theta_dot*t.
  const double h = 1e-3;
  const Eigen::Vector2d imuLocal(0.1326, 0.054);
  const Eigen::Vector2d p0 = support_rotate(input.q[2], imuLocal);
  const Eigen::Vector2d pPlus = support_rotate(input.q[2] + input.v[2] * h, imuLocal);
  const Eigen::Vector2d pMinus = support_rotate(input.q[2] - input.v[2] * h, imuLocal);
  const Eigen::Vector2d acceleration = (pPlus - 2.0 * p0 + pMinus) / (h * h);
  const SupportQ9 expectedDeltaC = kGroundResponseImuMass * J.transpose() * acceleration;
  assert(((urdf.C - baseline.C) - expectedDeltaC).norm() < 1e-8);

  // Verify gravity difference against a finite-difference potential gradient.
  const SupportQ9 gravityDeltaG = gravityOnly.G - baseline.G;
  const SupportQ9 deltaG = urdf.G - baseline.G;
  constexpr double eps = 1e-5;
  for (int i = 0; i < 9; ++i) {
    SupportQ9 qp = input.q, qm = input.q;
    qp[i] += eps;
    qm[i] -= eps;
    const double finiteDifference =
        (delta_potential(qp, input.body_mass) - delta_potential(qm, input.body_mass)) /
        (2.0 * eps);
    assert(std::abs(deltaG[i] - finiteDifference) < 2e-7);
    const double gravityFiniteDifference =
        (gravity_only_delta_potential(qp, input.body_mass) -
         gravity_only_delta_potential(qm, input.body_mass)) / (2.0 * eps);
    assert(std::abs(gravityDeltaG[i] - gravityFiniteDifference) < 2e-7);
  }

  // Gravity/IMU do not change the existing contact geometry or acceleration bias.
  assert((urdf.contactJacobian - baseline.contactJacobian).norm() == 0.0);
  assert((urdf.contactBias - baseline.contactBias).norm() == 0.0);

  // Bad state fails closed and leaves a reset, invalid response object.
  ThrustSupportDynamicsInput invalidInput = input;
  invalidInput.q[2] = std::numeric_limits<double>::quiet_NaN();
  GroundUrdfResponseModel invalid;
  assert(!build_ground_urdf_response_model(invalidInput, true, invalid));
  assert(!invalid.valid && invalid.M.isZero() && invalid.C.isZero() && invalid.G.isZero());

  std::cout << "PASS: isolated URDF response M/C/G corrections, SPD, contact invariance, NaN rejection\n";
  return 0;
}
