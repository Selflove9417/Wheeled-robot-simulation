#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/allocator_flight_plan.hpp"
using namespace bbot_jump;
void need(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
int main(){
 // Replay of recorded joint q/v; zero acceleration is explicitly a synthetic
 // boundary for this trajectory test, not an observed acceleration or jump.
 JointVector q(.6033357827,-1.1442232273,.6033357827,-1.1442232273);
 JointVector v(7.1602836456,-12.843899596,7.1602836456,-12.843899596);
 JointVector a(3.,-8.,3.,-8.);
 const auto p=make_allocator_flight_plan(3.29,q,v,{0.,0.},.088,0.,.8,2.,0.,9.5,bbot_kinematics::RobotParams(),a,true);
 // The whole-body gate may reject this synthetic COM/H/wheel combination.
 // The test separately proves physically bounded leg braking with no fixed
 // position reversal, preservation of all three initial derivatives, and rest.
 need(p.tuck_duration==.06,"braking time changed");
 for(int j=0;j<4;++j){
  double qq,vv,aa;p.tuck[j].evaluate(p.start,qq,vv,aa);
  need(std::abs(qq-q[j])<1e-9&&std::abs(vv-v[j])<1e-9&&std::abs(aa-a[j])<1e-8,"initial q/v/a lost");
  need(flight_trajectory_admissible(p.tuck[j],j%2?15.:11.,j%2?500.:450.,j%2?1.56:1.52),"bounded braking rejected");
  p.tuck[j].evaluate(p.start+.06,qq,vv,aa);
  need(std::abs(vv)<1e-8&&std::abs(aa)<1e-7,"leg did not brake to rest");
  need((qq-q[j])*v[j]>0.,"brake endpoint prematurely reversed movement");
 }
 JointVector bad=a;bad[0]=std::numeric_limits<double>::quiet_NaN();
 need(!make_allocator_flight_plan(3.29,q,v,{0.,0.},.088,0.,.8,2.,0.,9.5,bbot_kinematics::RobotParams(),bad,true).valid,"invalid initial acceleration accepted");
 std::cout<<"PASS: 60 ms bounded braking preserves initial q/v/a and ends at rest; no whole-jump claim\n";
}
