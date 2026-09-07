#include "bbot_balance_controller/centroidal_state.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace bbot_jump;
void require(bool ok,const char *msg) { if (!ok) throw std::runtime_error(msg); }
int main() {
    // Independently prescribe a world COM path while the base/legs rotate.
    // A rotated heading must not reverse forward motion, including backward travel.
    for (double speed:{-1.9,0.0,1.9}) {
        JointPoseHistory history;
        CentroidalWorldObserver observer;
        const Eigen::Vector3d heading(-std::sin(.7),std::cos(.7),0);
        for (int i=0;i<60;++i) {
            const double stamp=1+.01*i;
            const std::array<double,4> q{.2+.01*i,-.4-.005*i,.3+.008*i,-.5};
            history.push(stamp,q);
            const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(-.002*i,Eigen::Vector3d::UnitX())).toRotationMatrix();
            const Eigen::Vector3d base=heading*(speed*(stamp-1))+Eigen::Vector3d(0,0,.4)-
                rotation*centroidal_geometry(q,9.5).com;
            observer.update(stamp,stamp+.005,base,rotation,history,9.5);
            if (i) require(observer.valid(stamp+.005) &&
                std::abs(observer.forward_velocity(heading)-speed)<1e-10,"base/leg motion corrupted COM velocity");
        }
        require(!observer.valid(1.68),"stale COM velocity remained valid");
        require(!observer.valid(.5),"future COM sample accepted");
    }
    const double target=centroidal_catch_target(1.9,.15,.3,-2,.07,5);
    require(target<-2,"forward COM still outruns catch target");
    require(std::abs(target+centroidal_catch_target(-1.9,-.15,.3,2,.07,5))<1e-12,"asymmetric capture");
    require(centroidal_catch_target(0,0,.3,0,.07,5)==0,"stationary state creates walking");
    require(centroidal_catch_target(NAN,0,.3,0,.07,5)==0,"invalid input creates command");
    require(centroidal_catch_target(20,1,.3,0,.07,5)==-5,"controller speed ceiling exceeded");
    // Relative wheel spin compensation must yield the requested world axle speed.
    for (double rate:{-5.,0.,5.}) {
        const double u=centroidal_catch_target(.4,.03,.3,rate,.07,5);
        require(std::abs(-u-.07*rate-(1.25*.4+std::sqrt(9.81/.3)*.03))<1e-12,"shank spin compensation sign wrong");
    }
    // Independent linear inverted pendulum with 100/50 Hz sensing, one frame
    // actuator delay and the unchanged 8 m/s² wheel-command slew limit.
    int cases=0;
    std::vector<std::array<double,4>> scenarios;
    for (double height:{.25,.4,.5}) for (double speed:{-1.9,-.8,0.,.8,1.9})
    for (double initial:{-.08,0.,.08}) scenarios.push_back({height,speed,initial,-speed});
    // Approximate observed first-contact state, including opposing wheel spin.
    // Unlike arbitrary high-speed opposite-spin states, this is within the
    // capture basin with the existing 8 m/s² actuator slew bound.
    scenarios.push_back({.326,.55,-.024,.94});
    scenarios.push_back({.326,-.55,.024,-.94});
    scenarios.push_back({.326,.55,-.024,centroidal_catch_target(.55,-.024,.326,0,.07,5)});
    scenarios.push_back({.326,-.55,.024,centroidal_catch_target(-.55,.024,.326,0,.07,5)});
    for (const auto & scenario:scenarios) for (int period:{10,20}) {
        const auto [height,speed,initial,initial_command]=scenario;
        const double omega2=9.81/height;
        double com=initial,axle=0,v=speed,applied=initial_command,pending=applied;
        double sampled_com=com,sampled_axle=axle,sampled_v=v,cmd=applied,peak=0;
        for (int ms=0;ms<8000;++ms) {
            if (ms%period==0) {
                applied=pending;sampled_com=com;sampled_axle=axle;sampled_v=v;
            }
            if (ms%5==0) {
                const double goal=centroidal_catch_target(sampled_v,sampled_com-sampled_axle,height,0,.07,5);
                cmd+=std::clamp(goal-cmd,-.04,.04);pending=cmd;
            }
            v+=.001*omega2*(com-axle);
            com+=.001*v;axle-=.001*applied;
            peak=std::max(peak,std::abs(com-axle));
        }
        // Regression boundary: replaying the OLD opposing incoming command
        // with 50 Hz + a frame of delay is unrecoverable in this limited model.
        // The new pre-contact target must pass at both sampling rates below.
        if (period==20 && std::abs(initial_command)==.94) {
            require(peak>.3,"old-handoff boundary fixture no longer excites failure");
            continue;
        }
        if (!(peak<.3 && std::abs(com-axle)<.002 && std::abs(v)<.01))
            std::cerr<<"capture failure h="<<height<<" v0="<<speed<<" r0="<<initial
                     <<" u0="<<initial_command<<" period="<<period<<" peak="<<peak<<" final_v="<<v<<"\n";
        require(peak<.3 && std::abs(com-axle)<.002 && std::abs(v)<.01,
                "sampled capture did not stop and center COM");
        ++cases;
    }
    // Regression: the torso may be nearly level while COM translation diverges.
    require(!centroidal_hold_ready(true,-.0593344,-.0858444,-.23517),
            "11-second drifting recovery marked stable");
    require(!centroidal_hold_ready(false,0,0,0),"stale COM permits handoff");
    require(centroidal_hold_ready(true,.005,.01,.02),"quiet COM rejected");
    require(!centroidal_hold_ready(true,0,NAN,0),"invalid rate permits handoff");

    // 30-second recovery hold: raise COM from buffer to standing height over
    // 0.8 s, keep the controller running after settling, then disturb it twice.
    // Include vertical acceleration in the independent pendulum plant. This
    // checks sustained control, not just convergence at the end of CATCH.
    for (int period:{10,20}) for (double direction:{-1.,1.}) {
        double com=.005*direction, axle=0, v=.02*direction;
        double applied=0,pending=0,cmd=0,sc=com,sa=0,sv=v,sh=.21;
        double late_peak=0;
        for (int ms=0;ms<30000;++ms) {
            const double t=.001*ms, u=std::clamp(t/.8,0.,1.);
            const double height=.21+.12*(10*u*u*u-15*std::pow(u,4)+6*std::pow(u,5));
            const double az=(t<.8) ? .12/(.8*.8)*(60*u-180*u*u+120*u*u*u) : 0.;
            if (ms==5000 || ms==12000) v+=direction*.12;
            if (ms%period==0) { applied=pending;sc=com;sa=axle;sv=v;sh=height; }
            if (ms%5==0) {
                const double goal=centroidal_catch_target(sv,sc-sa,sh,0,.07,5);
                cmd+=std::clamp(goal-cmd,-.04,.04);pending=cmd;
            }
            v+=.001*(9.81+az)/height*(com-axle);
            com+=.001*v;axle-=.001*applied;
            require(std::abs(com-axle)<.08,"recovery height change lost capture");
            if (ms>20000) late_peak=std::max(late_peak,std::abs(v));
        }
        require(late_peak<.001 && std::abs(com-axle)<.001 && std::abs(cmd)<.001,
                "long hold resumed creeping after initial stabilization");
    }
    // User drive must preserve the existing zero-command balance equilibrium.
    for (double requested:{-.5,0.,.5}) {
        require(std::abs(centroidal_catch_target(requested,0,.33,0,.07,5,requested)+requested)<1e-12,
                "drive equilibrium reverses travel");
    }
    double reference=0;
    reference=ground_drive_reference(reference,.5,.005,.5,1.0);
    require(std::abs(reference-.005)<1e-12,"drive command jumps at enable");
    require(ground_drive_reference(reference,NAN,.005,.5,1.0)==0,"nonfinite drive request retained");
    // Delayed forward -> reverse -> stop, followed by squat/stand perturbation
    // with the same controller (repeat-jump ground preparation interface).
    for (int period:{10,20}) {
        double com=0,axle=0,v=0,applied=0,pending=0,cmd=0,ref=0;
        double sc=0,sa=0,sv=0,sh=.33;
        for (int ms=0;ms<24000;++ms) {
            const double t=.001*ms;
            const double request=t<2 ? 0. : t<7 ? .5 : t<12 ? -.5 : 0.;
            const double h=t<16 ? .33 : t<17 ? .33-.12*(t-16) : t<18 ? .21+.12*(t-17) : .33;
            if (ms%period==0) { applied=pending;sc=com;sa=axle;sv=v;sh=h; }
            if (ms%5==0) {
                ref=ground_drive_reference(ref,request,.005,.5,1.0);
                const double goal=centroidal_catch_target(sv,sc-sa,sh,0,.07,5,ref);
                cmd+=std::clamp(goal-cmd,-.04,.04);pending=cmd;
            }
            v+=.001*9.81/h*(com-axle);com+=.001*v;axle-=.001*applied;
            require(std::abs(com-axle)<.08,"driving lost centroidal balance");
            if (ms==6500) require(std::abs(v-.5)<.01,"forward command ignored");
            if (ms==11500) require(std::abs(v+.5)<.01,"reverse command ignored");
        }
        require(std::abs(v)<.001 && std::abs(com-axle)<.001,"stop did not restore original hold");
    }
    // Actual 8.154 s sample: position servo pulls backward before launch speed.
    // Unilateral P removes only that pull; the existing D still brakes overspeed.
    const double damping=3.0*(-2.82941-(-7.97836));
    const double old_knee=std::clamp(22.0*(-.0879087-(-.398485))+damping,-22.,22.);
    const double new_knee=std::clamp(22.0*thrust_knee_position_error(-.0879087,-.398485,true)+damping,-22.,22.);
    require(new_knee>15 && new_knee<old_knee-6,"position pull remains or velocity damping was removed");
    require(thrust_knee_position_error(-.4,-.2,true)==-.2,"extension tracking removed");
    require(thrust_knee_position_error(-.2,-.4,false)==.2,"protective position braking removed");
    std::cout<<"PASS: aligned COM translation, wheel kinematics, "<<cases
             <<" delayed capture/stop cases (not full robot simulation)\n";
}
