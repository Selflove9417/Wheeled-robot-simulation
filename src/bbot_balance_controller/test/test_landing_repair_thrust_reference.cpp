#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/landing_repair_thrust_reference.hpp"
using namespace bbot_jump;
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
double height(const JointVector & q,double pitch){
    auto g=centroidal_geometry({q[0],q[1],q[2],q[3]},9.5);
    return rotate_about_hip(-pitch,g.com-g.axle).z();
}
int main(){
    JointVector q(.45,-.70,.46,-.71),v(4,-8,4,-8);
    constexpr double pitch=.068,rate=.05,eps=1e-7;
    const std::array<double,2> absolute_wheels{-6,-6};
    const std::array<double,2> relative_wheels{
        absolute_wheels[0]+rate-v[0]-v[1],absolute_wheels[1]+rate-v[2]-v[3]};
    // Independent finite-difference of supported world COM height, plus the
    // seven-body momentum helper already checked against orbital+axial sums.
    const double vz=(height(q+eps*v,pitch+eps*rate)-height(q-eps*v,pitch-eps*rate))/(2*eps);
    const double H=landing_momentum(q,v,relative_wheels,rate,9.5).total;
    auto ref=momentum_thrust_reference(q,pitch,rate,absolute_wheels,9.5,vz,H);
    require(ref.valid,"feasible two-task reference rejected");
    require(std::abs(ref.hip_velocity-4)<1e-7 && std::abs(ref.knee_velocity+8)<1e-7,
            "inverse does not preserve both physical tasks or wheel-spin convention");
    require(std::abs(ref.predicted_com_velocity-vz)<1e-10 &&
            std::abs(ref.predicted_momentum-H)<1e-10,"reference equality residual");
    require(!momentum_thrust_reference(q,pitch,rate,absolute_wheels,9.5,vz,H,1,15).valid,
            "speed limit clipped into a falsely feasible reference");
    require(!momentum_thrust_reference(q,pitch,rate,absolute_wheels,9.5,-1,H).valid,
            "negative launch speed accepted");
    require(!momentum_thrust_reference(q,pitch,rate,absolute_wheels,-1,vz,H).valid,
            "invalid mass accepted");

    MomentumThrustScope scope{true,true,true,false,true,false,true,true,true,true,
                             1.0,.998,.99,.12};
    require(momentum_thrust_blend(scope)==1,"fully active phase rejected");
    for(int i=0;i<10;i++){
        auto x=scope;
        if(i==0)x.enabled=false;if(i==1)x.thrust=false;if(i==2)x.gate_open=false;
        if(i==3)x.blocked=true;if(i==4)x.effort_active=false;if(i==5)x.switch_pending=true;
        if(i==6)x.contact_valid=false;if(i==7)x.contact_continuous=false;
        if(i==8)x.bilateral=false;if(i==9)x.momentum_valid=false;
        require(momentum_thrust_blend(x)==0,"unsupported phase can apply reference");
    }
    auto x=scope;x.contact_stamp=.98;require(momentum_thrust_blend(x)==0,"stale contact");
    x=scope;x.momentum_stamp=.97;require(momentum_thrust_blend(x)==0,"stale velocity data");
    x=scope;x.momentum_stamp=1.001;require(momentum_thrust_blend(x)==0,"future data");
    x=scope;x.motion_time=.08;require(momentum_thrust_blend(x)==0,"nonzero onset");
    x.motion_time=.18;require(momentum_thrust_blend(x)==0,"nonzero end");
    x.motion_time=.09;require(std::abs(momentum_thrust_blend(x)-.5)<1e-10,"entry ramp");
    std::cout<<"supported dual reference and phase/contact guards passed\n";
}
