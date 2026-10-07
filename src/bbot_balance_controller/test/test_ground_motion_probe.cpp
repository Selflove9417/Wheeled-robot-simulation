#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include "bbot_balance_controller/ground_motion_probe.hpp"
#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace {
void require(bool value, const char *message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b, double eps=1e-10) {
  return std::abs(a-b) <= eps;
}
}

int main() {
  using namespace bbot_jump;
  const std::array<double,4> anchor{{.2558,-.3693,.2558,-.3693}};
  const auto start=ground_motion_probe_reference(anchor,1,0.0);
  require(!ground_motion_probe_reference(anchor,1,-1e-3).valid,
          "negative elapsed time was silently clamped");
  const auto end=ground_motion_probe_reference(anchor,1,kGroundMotionDuration);
  const auto reverse=ground_motion_probe_reference(anchor,-1,kGroundMotionDuration);
  require(start.valid && end.valid && reverse.valid, "reference invalid");
  for (int i=0;i<4;++i) {
    require(near(start.q[i],anchor[i]) && near(start.v[i],0.0) && near(start.a[i],0.0),
            "start is not anchored and stopped");
    require(near(end.q[i]-anchor[i], (i%2 ? -1.0 : 1.0) * .01 * (i%2 ? 2.0 : 1.0)),
            "forward pulse has wrong integrated displacement");
    require(near(reverse.q[i]-anchor[i], -(end.q[i]-anchor[i])),
            "return pulse is not symmetric");
    require(near(end.v[i],0.0) && near(end.a[i],0.0), "normal stop is not C2");
  }
  for (double boundary : {.5,1.0}) {
    const auto left=ground_motion_probe_reference(anchor,1,boundary-1e-7);
    const auto right=ground_motion_probe_reference(anchor,1,boundary+1e-7);
    for (int i=0;i<4;++i) {
      require(std::abs(left.q[i]-right.q[i])<1e-8, "position discontinuity");
      require(std::abs(left.v[i]-right.v[i])<1e-7, "velocity discontinuity");
      require(std::abs(left.a[i]-right.a[i])<1e-5, "acceleration discontinuity");
    }
  }

  GroundMotionGuardInput guard;
  guard.effort_active=true; guard.fresh_state=true; guard.bilateral_contact=true;
  guard.anchor=anchor; guard.q=anchor; guard.v={{0,0,0,0}};
  guard.pitch_delta=.01; guard.pitch_rate=.1;
  std::string reason;
  require(ground_motion_probe_guard(guard,reason), "valid guard rejected");
  guard.switch_pending=true;
  require(!ground_motion_probe_guard(guard,reason) && reason=="effort_mode_unavailable",
          "pending mode was accepted");
  guard.switch_pending=false; guard.q[0]=anchor[0]+.081;
  require(!ground_motion_probe_guard(guard,reason) && reason=="joint_anchor_or_rate_guard",
          "anchor excursion was accepted");

  // The motion branch uses a full 4x4 implicit PD solve. With a symmetric
  // robot and symmetric hip error, common-mode hip feedback must survive.
  const JointVector qSym((JointVector() << .24, -.36, .24, -.36).finished());
  const JointVector vSym=JointVector::Zero();
  const JointVector qdSym((JointVector() << .25, -.36, .25, -.36).finished());
  const JointVector vdSym=JointVector::Zero();
  const JointVector kpSym((JointVector() << 25.0,45.0,25.0,45.0).finished());
  const JointVector kdSym((JointVector() << 3.5,6.0,3.5,6.0).finished());
  const JointMatrix massSym=flight_joint_inertia(qSym,14.0);
  const JointVector commonHip=discrete_flight_pd(massSym,qSym,vSym,qdSym,vdSym,
      kpSym,kdSym,JointVector::Zero(),.04);
  require(commonHip.allFinite() && commonHip[0] > 0.0 && commonHip[2] > 0.0,
          "symmetric hip reference lost common-mode feedback");
  require(near(commonHip[0],commonHip[2],1e-8),
          "symmetric hip response was not symmetric");

  GroundMotionEffortInput effort;
  effort.support={{12.0,-8.0,14.0,-9.0}};
  effort.torso_per_hip=-1.5;
  const auto direct=ground_motion_direct_full_support(effort);
  require(direct.valid, "full support rejected");
  require(near(direct.torque[0],10.5) && near(direct.torque[2],12.5),
          "direct torso correction replaced common hip support");
  require(near(direct.torque[1],-8.0) && near(direct.torque[3],-9.0),
          "knee support changed unexpectedly");
  effort.torso_per_hip=100.0;
  require(near(ground_motion_direct_full_support(effort).torque[0],75.0),
          "hip torque cap was not enforced");
  effort.support[1]=std::numeric_limits<double>::quiet_NaN();
  require(!ground_motion_direct_full_support(effort).valid,
          "nonfinite support torque was accepted");
  effort.support[1]=-8.0; effort.hip_limit=100.0;
  require(!ground_motion_direct_full_support(effort).valid,
          "ground probe accepted a hip cap above the legacy 75 Nm bound");
  std::cout << "ground motion probe checks passed\n";
  return 0;
}
