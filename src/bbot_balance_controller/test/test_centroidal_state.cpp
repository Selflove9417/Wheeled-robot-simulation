#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <limits>

void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
int main() {
    using namespace bbot_jump;
    const std::array<double,4> zero{};
    // Actual standing fixture: COM should be almost above the axle despite a
    // nonzero box pitch. A neutral COM must not inherit the old +0.05 catch bias.
    const auto standing=centroidal_balance_state(
        {.258634,-.406939,.258634,-.406939},zero,.0458078,0,9.5);
    require(standing.valid && std::abs(standing.forward)<.001,"standing COM not over axle");
    require(standing.height>.32 && standing.height<.33,"COM confused with base_link height");

    // Actual 11.094 s fixture: the box leans forward but the mass lies behind
    // the wheels. Replaying the feedback checks direction, not closed-loop stability.
    const std::array<double,4> q{.890934,-.247101,.891152,-.247134};
    const std::array<double,4> v{-4.07261,2.26863,-4.07193,2.26793};
    const auto landing=centroidal_balance_state(q,v,.363404,-1.60915,9.5);
    require(landing.valid && landing.forward<-.07 && landing.angle<-.25,
            "forward box hides backward COM displacement");
    const double new_cmd=catch_wheel_target(landing.angle,landing.rate,1.12909,
        -179.6985,-42.8109,.043,1.5);
    require(new_cmd>0,"wheels still chase the forward box while COM is behind");
    std::cout<<"landing replay: COM offset="<<landing.forward<<" lean="<<landing.angle
             <<" rate="<<landing.rate<<" target="<<new_cmd<<"\n";

    // Numerical derivatives cover asymmetric poses, link redistribution and
    // body rotation. A changing knee with constant box pitch must change lean rate.
    for (int i=0;i<150;++i) {
        std::array<double,4> a{.2+.25*std::sin(i),-.45+.3*std::cos(i),
                               .25+.2*std::sin(2*i),-.4+.2*std::cos(2*i)};
        const std::array<double,4> speed{.7,-1.1,-.3,.9};
        auto plus=a,minus=a;
        constexpr double eps=1e-6;
        for (int j=0;j<4;++j) { plus[j]+=eps*speed[j]; minus[j]-=eps*speed[j]; }
        const double pitch=.15*std::sin(.5*i), rate=.4;
        const auto now=centroidal_balance_state(a,speed,pitch,rate,9.5);
        const auto next=centroidal_balance_state(plus,speed,pitch+eps*rate,rate,9.5);
        const auto prev=centroidal_balance_state(minus,speed,pitch-eps*rate,rate,9.5);
        require(now.valid && next.valid && prev.valid,"valid pose rejected");
        require(std::abs(now.rate-(next.angle-prev.angle)/(2*eps))<1e-7,"COM lean derivative ignores joint motion");
        const auto fixed_leg=centroidal_balance_state(a,zero,pitch,rate,9.5);
        require(std::abs(fixed_leg.rate-rate)<1e-12,"pitch sign does not match COM lean sign");
    }

    // No external impulse: let the legs reshape while choosing the base height
    // to keep world COM on a prescribed trajectory. Odometry and q are sampled
    // at different rates; the observer must align them, then recover COM velocity.
    JointPoseHistory history;
    CentroidalHeightObserver observer;
    std::array<double,4> joints{.25,-.4,.25,-.4};
    double old_base=0, last_base_speed=0;
    for (int i=0;i<25;++i) {
        const double t=1+.02*i;
        joints={.25+3*(t-1),-.4-6*(t-1),.25+3*(t-1),-.4-6*(t-1)};
        // Joint samples bracket each odometry timestamp, never extrapolate.
        auto before=joints,after=joints;
        for (int j=0;j<4;++j) { const double speed=j%2?-6:3; before[j]-=.005*speed; after[j]+=.005*speed; }
        history.push(t-.005,before); history.push(t+.005,after);
        const Eigen::Matrix3d orientation=(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(-.1,Eigen::Vector3d::UnitX())).toRotationMatrix();
        const Eigen::Vector3d vertical=orientation.row(2);
        const double base=.5+1.15*(t-1)-vertical.dot(centroidal_geometry(joints,9.5).com);
        observer.update(t,t+.006,base,vertical,history,9.5);
        if (i) {
            require(observer.valid(t+.006),"aligned COM velocity missing");
            require(std::abs(observer.velocity()-1.15)<1e-10,"leg motion leaked into world COM velocity");
            last_base_speed=(base-old_base)/.02;
            require(!observer.update(t,t+.007,base,vertical,history,9.5),"duplicate odometry counted twice");
            require(std::abs(observer.velocity()-1.15)<1e-10,"duplicate changed momentum estimate");
        }
        old_base=base;
    }
    require(!observer.valid(observer.stamp()+.081),"stale COM velocity allowed launch release");
    require(!observer.update(5,5,0,Eigen::Vector3d::UnitZ(),history,9.5),"unaligned pose extrapolated");
    require(!observer.update(0,0,0,Eigen::Vector3d::Zero(),history,9.5),"invalid orientation accepted");
    std::array<double,4> bad=q; bad[0]=std::numeric_limits<double>::quiet_NaN();
    require(!centroidal_balance_state(bad,v,0,0,9.5).valid,"invalid joint state accepted");
    require(!centroidal_balance_state(q,v,0,0,-1).valid,"invalid mass accepted");
    require(std::abs(last_base_speed-1.15)>.2,"fixture did not distinguish body and COM velocity");
    // Old release saw 1.94 m/s at 10.633 s. With mass redistribution the
    // synchronized COM speed, not the box speed, decides pulse completion.
    ThrustRelease release;
    release.update(10.633,1,1.15,1.98,185.5);
    require(!release.active(),"low momentum still releases thrust");
    release.update(10.7,1,1.95,1.98,185.5);
    require(release.active(),"real target COM speed cannot release thrust");
    std::cout<<"PASS: COM geometry, joint/pitch derivatives, aligned momentum and landing direction\n";
}
