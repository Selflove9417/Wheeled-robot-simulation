#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/thrust_support_allocator.hpp"
using namespace bbot_jump;
void need(bool value,const char*why){if(!value)throw std::runtime_error(why);}
int main(){
 // Real read-only native state at 8.790 s, and separately recorded controller
 // tasks at 8.795 s. This is a numerical regression, not a jump success.
 ThrustSupportAllocationInput in;auto &d=in.dynamics;
 d.q<<1.0311584259908806,.3505205423603289,-.11401976032403524,
  -.15370081569708008,.1991303522490579,-.1537076307900914,.19916035883337038,
  -14.458273018329708,-14.45832954016099;
 d.v<<.4025480697475047,-.11718313665835993,-.3510904096400806,
  .07731078857519759,.5577371721482463,.07761947289445684,.5574570692621221,
  -6.553503343000882,-6.553525161021889;
 d.prediction_dt=.010;d.actuator_limits={150,150,150,150,10,10};
 in.wheel_mode=SupportWheelMode::BoundedWheelServo;
 in.tasks.desired_com_vertical_acceleration=.914286;
 in.tasks.body_pitch_enabled=true;in.tasks.desired_body_pitch_acceleration=-10.1656;
 in.tasks.desired_centroidal_hdot=.548161;in.tasks.desired_com_axle_relative_forward_acceleration=1.26484;
 const auto before=allocate_thrust_support(in);
 in.enforce_hip_difference_acceleration=true;
 in.enforce_equal_normal_loads=true;
 in.desired_hip_difference_acceleration=-400.*(d.q[3]-d.q[5])-40.*(d.v[3]-d.v[5]);
 const auto after=allocate_thrust_support(in);
 need(after.valid,after.reason);
 need(std::abs(after.decision[3]-after.decision[5]-in.desired_hip_difference_acceleration)<1e-6,"anti-symmetric acceleration leaked");
 need(after.maximum_equality_residual<1e-6&&after.residuals.max_inequality_violation<1e-6,"physical constraints violated");
 need(std::abs(after.decision[16]-after.decision[18])<1e-6,"unequal normal load allocation leaked");
 need(std::abs(after.decision[9]-after.decision[11])<1.,"large opposing hip inputs survived");
 // Invalid synchronization request must fail closed.
 in.desired_hip_difference_acceleration=std::numeric_limits<double>::quiet_NaN();
 need(!allocate_thrust_support(in).valid,"nonfinite symmetry equality accepted");
 std::cout<<"before_valid="<<before.valid<<" before_hip="<<before.decision[9]<<'/'<<before.decision[11]
  <<" after_hip="<<after.decision[9]<<'/'<<after.decision[11]
  <<" eq_residual="<<after.maximum_equality_residual<<" PASS\n";
}
