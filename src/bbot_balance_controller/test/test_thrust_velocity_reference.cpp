#include "bbot_balance_controller/thrust_velocity_reference.hpp"
#include <iostream>
#include <stdexcept>

using namespace bbot_jump;
void require(bool ok,const char * message) { if (!ok) throw std::runtime_error(message); }

// Independently sum sagittal URDF link masses with both wheel axles fixed at
// ground height. This position-only calculation contains no controller Jacobian.
double stance_com_height(const std::array<double,4> & q,double pitch,double mass) {
    auto z=[](double y,double z,double angle) { return std::sin(angle)*y+std::cos(angle)*z; };
    double axle=0, weighted=mass*z(.13261282,.05396677,-pitch);
    for (int h:{0,2}) {
        const double hip_z=z(.125,-.07,-pitch);
        const double knee_z=z(-.29348091,-.06220095,q[h]-pitch);
        const double wheel_z=hip_z+knee_z+z(.28210870,-.19553796,q[h]+q[h+1]-pitch);
        weighted+=1.2*(hip_z+z(-.13690699,-.02116697,q[h]-pitch));
        weighted+=.8*(hip_z+knee_z+z(.11538205,-.08532288,q[h]+q[h+1]-pitch));
        weighted+=2*wheel_z;
        axle+=.5*wheel_z;
    }
    return .07+weighted/(mass+8)-axle;
}

int main() {
    // Late supported extension resembling both v6.12 jumps. Nominal IK asks
    // for -3 rad/s while COM force feedback still requests 1.98 m/s.
    const std::array<double,4> pose{.671123,-.909174,.671126,-.909166};
    const auto ref=thrust_velocity_reference(pose,.132348,9.5,1.98091,1.949,1.949,0,30);
    require(ref.valid && ref.knee_velocity<-12 && ref.knee_velocity>-16,
            "late thrust reference still follows slow nominal IK");
    const double observed=-10.0014;
    const double old_d=3*(-2.93299-observed);
    const double new_d=3*(ref.knee_velocity-observed);
    require(old_d>21 && new_d<0,"underspeed extension still receives braking D");
    require(3*(ref.knee_velocity-(ref.knee_velocity-2))>0,"overspeed damping removed");
    require(ref.knee_velocity==ref.knee_velocity_raw && !ref.speed_limited,
            "ordinary target artificially limited");
    const auto limited=thrust_velocity_reference(pose,.14,9.5,5,3,3,0,10);
    require(limited.valid && limited.speed_limited && limited.knee_velocity==-10,
            "configured motor speed limit ignored");
    require(!thrust_velocity_reference(pose,NAN,9.5,2,0,0,0,30).valid,"invalid pose accepted");
    require(!thrust_velocity_reference(pose,0,-1,2,0,0,0,30).valid,"invalid mass accepted");
    require(!thrust_velocity_reference(pose,0,9.5,2,0,0,0,0).valid,"invalid speed limit accepted");
    auto bad=pose;bad[3]=NAN;
    require(!thrust_velocity_reference(bad,0,9.5,2,0,0,0,30).valid,"invalid joint accepted");
    require(!thrust_velocity_reference(pose,3,9.5,2,0,0,0,30).valid,"fallen geometry inverted");
    // A straight leg has no usable vertical knee lever; never divide through it.
    const double a=(1.6*.11538205+4*.28210870)/17.5-.28210870;
    const double b=(1.6*(-.08532288)+4*(-.19553796))/17.5+.19553796;
    const double k_straight=std::atan2(a,b);
    const std::array<double,4> singular{0,k_straight,0,k_straight};
    require(!thrust_velocity_reference(singular,0,9.5,2,0,0,0,30).valid,
            "singular/reversed extension geometry inverted");

    int geometries=0;
    for (int i=0;i<160;++i) {
        const double pitch=.1+.04*std::sin(i), mass=(i%2)?9.5:14.;
        const std::array<double,4> q{.25+.2*std::sin(i),-.45+.2*std::cos(i),
                                    .28+.2*std::sin(2*i),-.48+.2*std::cos(2*i)};
        const double target=.4+.01*i, hl=.8,hr=.5,rate=.1;
        const auto cmd=thrust_velocity_reference(q,pitch,mass,target,hl,hr,rate,30);
        require(cmd.valid && !cmd.speed_limited,"regular geometry rejected");
        const std::array<double,4> speed{hl,cmd.knee_velocity,hr,cmd.knee_velocity};
        auto plus=q,minus=q;
        constexpr double eps=1e-6;
        for(int j=0;j<4;++j){plus[j]+=eps*speed[j];minus[j]-=eps*speed[j];}
        const double numerical=(stance_com_height(plus,pitch+eps*rate,mass)-
                                stance_com_height(minus,pitch-eps*rate,mass))/(2*eps);
        require(std::abs(numerical-target)<1e-7,"reference does not produce requested COM velocity");
        // No measured speed enters reference generation. Fixed reference D
        // dissipates kinetic error for either sign of velocity perturbation.
        for(double error:{-3.,-1.,1.,3.}) require(3*(-error)*error<0,"D is not dissipative");
        ++geometries;
    }
    require(protected_thrust_knee_velocity(-14,0,0)==0,"travel stop permits extension");
    require(std::abs(protected_thrust_knee_velocity(-14,.4,0)+5.6)<1e-12,"travel taper bypassed");
    require(protected_thrust_knee_velocity(-14,1,1)==-3.5,"terminal braking bypassed");

    // Independent effective-mass velocity tracking with 10/20 ms samples and
    // a one-frame actuator delay; holding force is in equilibrium. This tests
    // damping of speed error, not flight/contact transitions or Gazebo height.
    for (int period:{10,20}) for (double inertia_scale:{.7,1.,1.3}) {
        const double j=-.18, inertia=inertia_scale*17.5*j*j;
        const double desired=1.98091/j;
        double velocity=-3, sensed=velocity,applied=0,pending=0,peak=0;
        for (int ms=0;ms<2000;++ms) {
            if (ms%period==0) { applied=pending;sensed=velocity; }
            if (ms%5==0) pending=2*std::clamp(3*(desired-sensed),-22.,22.);
            velocity+=.001*applied/inertia;
            peak=std::max(peak,std::abs(velocity));
        }
        require(std::abs(j*velocity-1.98091)<.001 && peak<15,
                "sampled speed damping diverged or cannot represent launch speed");
    }
    std::cout<<"PASS: "<<geometries<<" independent stance derivatives, underspeed/overspeed D, "
             <<"motor/travel limits and delayed velocity tracking (not Gazebo validation)\n";
}
