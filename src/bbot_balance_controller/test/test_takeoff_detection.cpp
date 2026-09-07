#include "bbot_balance_controller/takeoff_detection.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
}
int main() {
    using namespace bbot_jump;
    JointPoseHistory history;
    history.push(1.00,{0,0,0,0});
    history.push(1.02,{0.2,-0.4,0.2,-0.4});
    std::array<double,4> q{};
    check(history.interpolate(1.01,q),"bracketed pose unavailable");
    check(std::abs(q[0]-0.1)<1e-10 && std::abs(q[1]+0.2)<1e-10,"interpolation incorrect");
    // Synthetic grounded extension at 2 m/s: a 10 ms skew creates 20 mm
    // false penetration with the latest pose, while matched timestamps cancel.
    const double world_height=0.4+0.2*0.1;
    check(std::abs(world_height-(0.4+0.2*q[0]))<1e-10,"aligned grounded clearance");
    check(world_height-(0.4+0.2*0.2)<-0.019,"skew regression scenario");
    check(!history.interpolate(1.03,q),"future extrapolation allowed");
    check(!history.interpolate(0.99,q),"past extrapolation allowed");
    history.push(1.10,{1,1,1,1});
    check(!history.interpolate(1.06,q),"sensor gap interpolated");
    history.push(0.10,{2,2,2,2});
    check(!history.interpolate(1.01,q),"clock reset retained old poses");
    history.push(0.10,{3,3,3,3});
    check(history.interpolate(0.10,q) && q[0]==3,"duplicate replacement");

    TakeoffConfirmation detector;
    auto update = [&](double stamp,double left,double right,bool unloaded=true,bool rise=true) {
        return detector.update(stamp,stamp+0.005,true,left,right,unloaded,rise);
    };
    check(!update(1,.014,.014) && detector.count()==1,"first frame confirmed");
    check(!detector.update(1,1.010,true,.014,.014,true,true) && detector.count()==1,
          "repeated frame counted");
    check(update(1.02,.010,.011),"clearance hysteresis lost airborne evidence");
    detector.reset();
    check(!update(2,.020,0) && detector.count()==0,"single wheel triggered");
    check(!update(2.02,0,0),"body ascent with grounded wheels triggered");
    check(!update(2.04,.020,.020,false),"loaded wheels triggered");
    check(!update(2.06,.020,.020,true,false),"no lift motion triggered");
    check(!update(2.08,.014,.014),"first frame triggered after rejection");
    check(!update(2.10,.005,.020) && detector.count()==0,"renewed contact did not reset");
    check(!update(2.12,.014,.014),"first frame triggered after contact");
    check(!update(2.14,.015,.015,false) && detector.count()==0,"acceleration recovery did not reset");
    check(!update(2.16,.010,.010),"hysteresis admitted initial subthreshold frame");
    check(!update(2.18,.020,.020),"first frame after reset");
    check(!update(2.25,.020,.020) && detector.count()==1,"gap kept old evidence");
    check(!detector.update(2.27,2.40,true,.020,.020,true,true) && detector.count()==0,"stale frame accepted");
    check(!detector.update(2.50,2.40,true,.020,.020,true,true),"future frame accepted");
    check(!detector.update(2.40,2.405,false,.020,.020,true,true),"unaligned pose accepted");
    check(!update(3,.020,.020),"first frame after stale data");
    check(!update(3.005,.020,.020),"confirmation shorter than 10ms");
    check(update(3.015,.020,.020),"independent frames did not confirm");
    check(!update(0.1,.020,.020) && detector.count()==1,"clock reset retained evidence");
    check(!update(0.12,std::numeric_limits<double>::quiet_NaN(),.02),"NaN clearance accepted");
    detector.reset();
    check(!detector.update(7.44,7.445,true,.0274,.0274,true,true),"first clear frame accepted");
    check(detector.update(7.46,7.465,true,.0384,.0384,false,true),
          "internal IMU impulse vetoed two clearly airborne wheel samples");
    detector.reset();
    check(!detector.update(8,8.005,true,.013,.014,false,true) && detector.count()==0,
          "ordinary clearance bypassed load check");
    check(!detector.update(8.02,8.025,true,.030,0,false,true),"one clear wheel bypassed contact");
    TouchdownConfirmation contact;
    check(!contact.update(4,4.005,true,.020,true,9.8),"near ground is not contact");
    check(!contact.update(4.02,4.025,true,0,false,9.8),"ascending touchdown accepted");
    check(!contact.update(4.04,4.045,true,0,true,0),"unloaded trajectory accepted as contact");
    // Latest run: wheels have reached ground while deploy is unfinished and
    // reported motor effort is zero. Geometry + loading persists across frames.
    check(!contact.update(8.12,8.125,true,-.00034,true,11.8),"single contact frame accepted");
    check(!contact.update(8.12,8.13,true,-.00034,true,11.8),"duplicate contact counted");
    check(contact.update(8.14,8.145,true,-.00028,true,9.17),"supported wheels did not enter buffer");
    contact.reset();
    check(!contact.update(9,9.1,true,0,true,9.8),"stale contact accepted");
    check(!contact.update(9.2,9.205,false,0,true,9.8),"unsynchronized contact accepted");
    check(!contact.update(9.22,9.225,true,0,true,9.8),"first frame after invalid contact");
    check(!contact.update(9.28,9.285,true,0,true,9.8),"contact gap retained confirmation");
    std::cout << "Takeoff and touchdown timestamp confirmation regressions passed\n";
}
