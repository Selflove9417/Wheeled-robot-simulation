#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "bbot_balance_controller/reference_jump_policy.hpp"
#include "bbot_balance_controller/effort_allocation.hpp"
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/ground_launch_reference.hpp"
#include "bbot_balance_controller/rolling_jump_control.hpp"
#include "bbot_kinematics/kinematics.hpp"

void check(bool ok, const char *message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  using namespace bbot_jump;
  constexpr double support = 17.5 * 9.81 / 2.0;
  ReferenceJumpPulse pulse;
  double previous = 0.0;
  for (int i=0; i<90; ++i) {
    const double f = pulse.extra_force(i*.001, true, true, 1.0, 2.43, support);
    check(f>=previous-1e-10 && f<=3.0*support, "impulse must rise smoothly within its budget");
    previous=f;
  }
  check(pulse.extra_force(.090, false, true, 9.0, 2.43, support)==0.0 &&
        !pulse.released(), "stale high speed must neither release nor add impulse");
  check(pulse.extra_force(.091, true, false, 9.0, 2.43, support)==0.0 &&
        !pulse.released(), "closed attitude/mode gate must not arm the impulse");
  previous=pulse.extra_force(.100, true, true, 2.44, 2.43, support);
  check(pulse.released(), "fresh COM target must latch release");
  for (int i=1; i<=100; ++i) {
    const double f = pulse.extra_force(.100+i*.001, true, true, -.5, 2.43, support);
    check(f>=0.0 && f<=previous+1e-10, "released impulse must unload monotonically despite falling speed");
    previous=f;
  }
  check(previous==0.0, "impulse may not restart after unload");
  pulse.reset();
  pulse.extra_force(.320, false, false, 0.0, 2.43, support);
  check(pulse.released() && pulse.extra_force(.361, true, true, .5, 2.43, support)==0.0,
        "finite pulse must expire even with missing observations");
  pulse.reset();
  check(!pulse.released() && pulse.extra_force(.04,true,true,0.0,2.43,support)>0.0,
        "second jump needs a fresh impulse state");
  check(!reference_deploy_due(true,true,1.0,.5,.12,.055), "finished tuck must hold while ascending");
  check(!reference_deploy_due(true,false,-1.0,.5,.12,.055), "stale velocity must not manufacture an apex");
  check(reference_deploy_due(true,true,-.1,.5,.12,.055), "fresh apex must begin deployment");
  check(reference_deploy_due(false,false,2.0,.15,.12,.055), "landing deadline must override unfinished tuck");
  check(reference_deploy_due(false,false,0.0,std::numeric_limits<double>::quiet_NaN(),.12,.055),
        "invalid remaining flight budget must not keep holding tucked legs");
  check(reference_travel_scale(-1.25,-15.0,.020)==0.0,
        "joint must unload before stale high-speed extension hits the hard stop");
  check(reference_travel_scale(-.50,-12.0,0.0)>reference_travel_scale(-.50,-12.0,.030),
        "fresh and delayed samples must have different remaining travel budgets");
  check(reference_travel_scale(1.25,15.0,.020)==0.0,
        "both physical end stops need symmetric predictive protection");
  check(reference_travel_scale(0.0,2.0,.081)==0.0 &&
        reference_travel_scale(-.7,-5.0,.020)==1.0,
        "stale joint samples must unload, while ordinary stroke retains impulse");
  check(reference_travel_scale(-.85,-18.0,0.0)==0.0,
        "braking distance must cut impulse before a fresh fast joint reaches its stop");
  bbot_kinematics::Kinematics kin;
  const auto takeoff=kin.inverse_kinematics(.66,0.0);
  const auto tuck=kin.inverse_kinematics(.52,0.0);
  const auto land=kin.inverse_kinematics(.60,0.085);
  check(tuck.theta_knee>takeoff.theta_knee+.5 && land.theta_knee>takeoff.theta_knee+.2,
        "CAD offset poses must genuinely flex the knee relative to long-leg takeoff");
  for (double q : {tuck.theta_hip,tuck.theta_knee,land.theta_hip,land.theta_knee})
    check(std::abs(q)<1.52,"new flight poses must fit unchanged physical joint limits");
  const std::array<double,4> start{takeoff.theta_hip,takeoff.theta_knee,takeoff.theta_hip,takeoff.theta_knee};
  const std::array<double,4> mid{tuck.theta_hip,tuck.theta_knee,tuck.theta_hip,tuck.theta_knee};
  const std::array<double,4> end{land.theta_hip,land.theta_knee,land.theta_hip,land.theta_knee};
  const auto plan=plan_flight_round_trip(start,{}, {},mid,end,.14,.12,.5,11,13,450,500,1.52,1.5708);
  check(plan.valid,"full flex/deploy trajectory must fit normal flight bounds");
  for (double pd : {-22., 0., 22.}) {
    const double limit=signed_force_limit(-.25,pd,57.0);
    check(std::abs(-.25*std::min(4.0*support,limit)+pd)<=57.0+1e-10,
          "pulse force must retain the signed final knee torque budget");
  }

  // The final SQUAT reference and first THRUST reference use the same CAD
  // mapping at the shared height/pitch. This guards against a phase-boundary
  // IK reseed that would create an artificial joint-rate pulse.
  bbot_kinematics::Kinematics launch_kin;
  const auto & launch_params = launch_kin.get_params();
  constexpr double launch_height = 0.34;
  constexpr double squat_start_height = 0.474;
  constexpr double squat_start_pitch = 0.05555;
  constexpr double launch_pitch = 0.050;
  constexpr double launch_com_forward = 0.010;
  constexpr double body_mass = 9.5;
  const auto squat_end = ground_launch_pose(
      launch_params, launch_height, launch_pitch, launch_com_forward,
      body_mass, ground_launch_blend(1.0));
  const auto thrust_start = ground_launch_pose(
      launch_params, launch_height, launch_pitch, launch_com_forward,
      body_mass, 1.0);
  check(squat_end.valid && thrust_start.valid,
        "shared SQUAT/THRUST launch pose must be geometrically valid");
  check(squat_end.q == thrust_start.q &&
        squat_end.target_x == thrust_start.target_x,
        "THRUST seed must exactly continue the final SQUAT pose");
  check(std::abs(squat_end.com_forward - launch_com_forward) < 1e-8 &&
        std::abs(thrust_start.com_forward - launch_com_forward) < 1e-8,
        "phase handoff pose must meet the requested CAD COM-forward target");
  check(std::abs(squat_end.q[0]) <= 1.52 && std::abs(squat_end.q[1]) <= 1.5708,
        "shared launch pose must remain inside the existing joint bounds");

  const auto neutral_start = ground_launch_pose(
      launch_params, squat_start_height, squat_start_pitch, launch_com_forward,
      body_mass, ground_launch_blend(0.0));
  const auto neutral_ik = launch_kin.inverse_kinematics(squat_start_height, 0.0);
  check(neutral_start.valid &&
        neutral_start.q[0] == neutral_ik.theta_hip &&
        neutral_start.q[1] == neutral_ik.theta_knee,
        "zero launch blend at the real SQUAT start must preserve neutral IK");

  // Exercise the actual SQUAT height trajectory. The neutral .474 m start is
  // valid, then the shape blend and height move together to the shared .34 m
  // THRUST seed; do not test the unreachable combination (.34 m, blend=0).
  QuinticTrajectory squat_height;
  constexpr double squat_duration = 0.50;
  constexpr int squat_samples = 501;
  squat_height.init(0.0, squat_duration, squat_start_height, 0.0, 0.0,
                    launch_height, 0.0, 0.0);
  std::array<std::array<double, 2>, squat_samples> squat_q{};
  for (int i = 0; i < squat_samples; ++i)
  {
    const double t = squat_duration * static_cast<double>(i) / (squat_samples - 1);
    double h = 0.0, hdot = 0.0, hddot = 0.0;
    squat_height.evaluate(t, h, hdot, hddot);
    const double progress = t / squat_duration;
    const double blend = ground_launch_blend(progress);
    const double pitch_ref = rolling_reference_blend(
        squat_start_pitch, launch_pitch, progress);
    const auto pose = ground_launch_pose(
        launch_params, h, pitch_ref, launch_com_forward, body_mass, blend);
    check(pose.valid,
          "all samples on the real .474-to-.34 m SQUAT path must remain valid");
    squat_q[static_cast<std::size_t>(i)] = pose.q;
  }
  double max_squat_hip_rate = 0.0;
  double max_squat_knee_rate = 0.0;
  const double squat_dt = squat_duration / (squat_samples - 1);
  for (int i = 1; i + 1 < squat_samples; ++i)
  {
    const double hip_rate = (squat_q[static_cast<std::size_t>(i + 1)][0] -
                             squat_q[static_cast<std::size_t>(i - 1)][0]) /
                            (2.0 * squat_dt);
    const double knee_rate = (squat_q[static_cast<std::size_t>(i + 1)][1] -
                              squat_q[static_cast<std::size_t>(i - 1)][1]) /
                             (2.0 * squat_dt);
    max_squat_hip_rate = std::max(max_squat_hip_rate, std::abs(hip_rate));
    max_squat_knee_rate = std::max(max_squat_knee_rate, std::abs(knee_rate));
  }
  double max_hip_lookahead_rate = 0.0;
  double max_knee_lookahead_rate = 0.0;
  double max_hip_position_step_rate = 0.0;
  double max_knee_position_step_rate = 0.0;
  const int lookahead_steps = static_cast<int>(std::lround(0.010 / squat_dt));
  const int control_steps = static_cast<int>(std::lround(0.005 / squat_dt));
  for (int i = 0; i + lookahead_steps < squat_samples; i += control_steps)
  {
    for (std::size_t joint = 0; joint < 2; ++joint)
    {
      const double rate =
          (squat_q[static_cast<std::size_t>(i + lookahead_steps)][joint] -
           squat_q[static_cast<std::size_t>(i)][joint]) / 0.010;
      if (joint == 0)
        max_hip_lookahead_rate = std::max(max_hip_lookahead_rate, std::abs(rate));
      else
        max_knee_lookahead_rate = std::max(max_knee_lookahead_rate, std::abs(rate));
      const double position_step_rate =
          (squat_q[static_cast<std::size_t>(i + control_steps)][joint] -
           squat_q[static_cast<std::size_t>(i)][joint]) / 0.005;
      if (joint == 0)
        max_hip_position_step_rate = std::max(
            max_hip_position_step_rate, std::abs(position_step_rate));
      else
        max_knee_position_step_rate = std::max(
            max_knee_position_step_rate, std::abs(position_step_rate));
    }
  }
  check(max_squat_hip_rate <= 2.0 && max_squat_knee_rate <= 2.0 &&
        max_hip_lookahead_rate <= 2.0 && max_knee_lookahead_rate <= 2.0 &&
        max_hip_position_step_rate <= 2.0 && max_knee_position_step_rate <= 2.0,
        "actual .474 m/.05555-to-.05 rad SQUAT path must fit the 2 rad/s position slew");
  check(std::abs(squat_q.front()[0] - neutral_start.q[0]) < 1e-10 &&
        std::abs(squat_q.front()[1] - neutral_start.q[1]) < 1e-10 &&
        std::abs(squat_q.back()[0] - squat_end.q[0]) < 1e-10 &&
        std::abs(squat_q.back()[1] - squat_end.q[1]) < 1e-10,
        "SQUAT trajectory endpoints must match neutral seed and THRUST handoff");

  check(!ground_launch_pose(launch_params, 0.95, launch_pitch,
                            launch_com_forward, body_mass, 1.0).valid,
        "unreachable high launch geometry must remain rejected");
  const auto rejected_pose = ground_launch_pose(
      launch_params, launch_height, launch_pitch, launch_com_forward,
      0.0, 1.0);
  const auto fallback_ik = launch_kin.inverse_kinematics(launch_height, 0.0);
  check(!rejected_pose.valid && std::isfinite(fallback_ik.theta_hip) &&
        std::isfinite(fallback_ik.theta_knee) &&
        std::abs(fallback_ik.theta_hip) <= 1.52 &&
        std::abs(fallback_ik.theta_knee) <= 1.5708,
        "invalid shaped pose must permit the controller's bounded neutral-IK fallback");
  std::cout << "reference jump policy passed\n";
}
