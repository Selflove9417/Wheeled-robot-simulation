// Golden-value checks for the C++ port of the deployed LQR design model.
//
// Every reference number below was produced by
// src/bbot_balance_controller/scripts/verify_lqr_model_and_sweep.py (which is
// the script that validates the gain tables compiled into
// lqr_gain_scheduled_controller.cpp / adaptive_lqr_balance_controller.cpp) and
// by scipy.linalg.solve_discrete_are. Regenerate with:
//
//   cd src/bbot_balance_controller/scripts && python3 mpc_model_reference.py --dump

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <Eigen/Core>

#include "bbot_balance_controller/lqr_plant_model.hpp"
#include "bbot_kinematics/kinematics.hpp"
#include "bbot_kinematics/robot_params.hpp"

using bbot_balance_controller::lqr_plant::SuspendedBody;
using bbot_balance_controller::lqr_plant::continuous_matrices;
using bbot_balance_controller::lqr_plant::equilibrium_pitch;
using bbot_balance_controller::lqr_plant::zoh_discretize;
using bbot_balance_controller::lqr_plant::leg_angles;
using bbot_balance_controller::lqr_plant::solve_dare;
using bbot_balance_controller::lqr_plant::spectral_radius;
using bbot_balance_controller::lqr_plant::suspended_body;

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

bool near(double value, double reference, double tolerance, const char *what)
{
  const double error = std::abs(value - reference);
  if (error > tolerance)
  {
    std::printf("%s mismatch: got %.12g reference %.12g error %.3e\n", what, value, reference,
      error);
    return false;
  }
  return true;
}
}  // namespace

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  bbot_kinematics::RobotParams parameters;

  // 1. The suspended-body geometry has to reproduce the y_com / z_com columns
  //    of the deployed gain tables (adaptive_lqr_balance_controller.cpp:299-303,
  //    tolerance copied from verify_lqr_model_and_sweep.py:244).
  const double table[5][3] = {
    {0.3000, -0.0276264, 0.3592154},
    {0.3500, -0.0258066, 0.4016316},
    {0.4000, -0.0234056, 0.4443432},
    {0.4500, -0.0203401, 0.4872441},
    {0.5000, -0.0164269, 0.5302655}};
  double worst_geometry = 0.0;
  for (const auto &row : table)
  {
    const SuspendedBody body = suspended_body(row[0], parameters);
    worst_geometry = std::max(worst_geometry, std::abs(body.y_com - row[1]));
    worst_geometry = std::max(worst_geometry, std::abs(body.z_com - row[2]));
  }
  std::printf("worst |y_com|/|z_com| error against the deployed table = %.3e m\n",
    worst_geometry);
  require(worst_geometry < 1.0e-6, "suspended COM geometry does not match the deployed table");

  // 2. Continuous matrices at H = 0.40 m.
  Eigen::Matrix4d A;
  Eigen::Vector4d B;
  SuspendedBody body;
  continuous_matrices(0.40, parameters, A, B, &body);
  require(near(A(1, 2), -13.010066585554, 1.0e-9, "A(1,2)"), "A(1,2)");
  require(near(A(3, 2), 43.631425961657, 1.0e-9, "A(3,2)"), "A(3,2)");
  require(near(B(1), 1.870368272195, 1.0e-9, "B(1)"), "B(1)");
  require(near(B(3), -3.894393813347, 1.0e-9, "B(3)"), "B(3)");
  require(A.rows() == 4 && A(0, 1) == 1.0 && A(2, 3) == 1.0, "state ordering [x, x_dot, theta, theta_dot]");
  require(near(body.length, 0.4449592250653897, 1.0e-12, "pendulum length"), "length");
  require(near(body.inertia, 0.46890137827131695, 1.0e-9, "suspended inertia"), "inertia");
  require(near(body.effective_mass, 20.14530612244898, 1.0e-9, "effective mass"), "M1");

  // 3. Exact zero-order-hold discretisation at the deployed 200 Hz period.
  Eigen::Matrix4d Ad;
  Eigen::Vector4d Bd;
  zoh_discretize(0.005, A, B, Ad, Bd);
  const double ad_reference[16] = {
    1.000000000000e+00, 5.000000000000e-03, -1.626406153506e-04, -2.710578367433e-07,
    0.0, 1.000000000000e+00, -6.506215956769e-02, -1.626406153506e-04,
    0.0, 0.0, 1.000545442402e+00, 5.000909037618e-03,
    0.0, 0.0, 2.181967924158e-01, 1.000545442402e+00};
  const double bd_reference[4] = {2.338092288590e-05, 9.352896966936e-03, -4.868434777448e-05,
    -1.947550921721e-02};
  for (int row = 0; row < 4; ++row)
  {
    for (int column = 0; column < 4; ++column)
    {
      char label[32];
      std::snprintf(label, sizeof(label), "Ad(%d,%d)", row, column);
      require(near(Ad(row, column), ad_reference[row * 4 + column], 1.0e-11, label), label);
    }
    char label[16];
    std::snprintf(label, sizeof(label), "Bd(%d)", row);
    require(near(Bd(row), bd_reference[row], 1.0e-12, label), label);
  }

  // 4. The deployed fixed gain must stabilise this model (it is the seed of
  //    the Riccati policy iteration, so the check is a precondition).
  Eigen::RowVector4d deployed_gain;
  deployed_gain << -5.931603, -45.087129, -181.972969, -43.390225;
  const double deployed_radius = spectral_radius(Ad - Bd * deployed_gain);
  std::printf("spectral radius of the deployed fixed gain = %.8f\n", deployed_radius);
  require(deployed_radius < 1.0, "deployed fixed gain does not stabilise the model");

  // 5. DARE solution and LQR gain for the Chapter-2 weighting used by the
  //    repository scripts (Q = diag(100, 5000, 3000, 1200), R = 1).
  Eigen::Matrix4d Q;
  Q.setZero();
  Q(0, 0) = 100.0;
  Q(1, 1) = 5000.0;
  Q(2, 2) = 3000.0;
  Q(3, 3) = 1200.0;
  Eigen::Matrix4d P;
  Eigen::RowVector4d K;
  int iterations = 0;
  double residual = 0.0;
  const bool solved = solve_dare(Ad, Bd, Q, 1.0, deployed_gain, P, K, 50, iterations, residual);
  std::printf("Kleinman iterations=%d residual=%.3e\n", iterations, residual);
  require(solved, "Riccati policy iteration did not converge");
  require(iterations <= 12, "Riccati policy iteration needs too many passes");
  const double k_reference[4] = {-6.230876172057, -47.540093775299, -214.496292448661,
    -54.771166018161};
  for (int index = 0; index < 4; ++index)
  {
    char label[24];
    std::snprintf(label, sizeof(label), "K[%d]", index);
    require(near(K(index), k_reference[index], 1.0e-8, label), label);
  }
  // scipy.linalg.solve_discrete_are(Ad, Bd, Q, 1) entries.
  require(near(P(0, 0), 152595.21281570703, 1.0e-4, "P(0,0)"), "P(0,0)");
  require(near(P(2, 2), 3325097.9585884362, 1.0e-2, "P(2,2)"), "P(2,2)");
  require(near(P(3, 3), 146405.41938674354, 1.0e-4, "P(3,3)"), "P(3,3)");
  require(std::abs(P(0, 2) - P(2, 0)) < 1.0e-9, "P must be symmetric");
  require(spectral_radius(Ad - Bd * K) < 1.0, "DARE gain must stabilise the model");

  // 6. The ported leg angles must agree with bbot_kinematics for the same
  //    hip-axle height, otherwise the MPC would fly a different posture than
  //    the one the leg position controller is commanded to.
  double q_hip = 0.0;
  double q_knee = 0.0;
  leg_angles(0.40, parameters, q_hip, q_knee);
  bbot_kinematics::Kinematics kinematics;
  const double base_to_hip = 0.07;  // height.base_to_hip of the deployed nodes
  const auto ik = kinematics.inverse_kinematics(
    0.40 + base_to_hip + parameters.wheel_radius, 0.0);
  std::printf("leg angles at H=0.40: port hip=%.8f knee=%.8f, kinematics hip=%.8f knee=%.8f\n",
    q_hip, q_knee, ik.theta_hip, ik.theta_knee);
  require(near(q_hip, ik.theta_hip, 1.0e-9, "hip angle"), "ported IK hip differs from bbot_kinematics");
  require(near(q_knee, ik.theta_knee, 1.0e-9, "knee angle"), "ported IK knee differs from bbot_kinematics");

  // 7. equilibrium_pitch() must equal the geometric column used by
  //    adaptive_lqr_balance_controller.cpp:603 (-atan2(y_com, z_com)).
  const double geometric_pitch = equilibrium_pitch(body);
  require(near(geometric_pitch, -std::atan2(body.y_com, body.z_com), 1.0e-15, "theta_eq"),
    "theta_eq formula");
  std::printf("theta_eq(H=0.40) geometric = %.6f rad, GS-LQR table = 0.087823 rad\n",
    geometric_pitch);

  std::printf("test_lqr_plant_model: all checks passed\n");
  return 0;
}
