#include "bbot_balance_controller/jump_phase_control.hpp"
#include "bbot_balance_controller/control_timing.hpp"
#include <iostream>
#include <stdexcept>

void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
int main() {
    using namespace bbot_jump;
    double clock_previous=8.880, control_dt=0.0;
    int controls=0;
    for (int i=0;i<=100;++i) {
        if (advance_control_time(8.880+0.0001*i,clock_previous,control_dt)) {
            ++controls;
            require(std::abs(control_dt-.005)<1e-9,"synthetic controller dt");
        }
    }
    require(controls==2,"slow simulation runs control more than 200Hz");
    require(!advance_control_time(clock_previous,clock_previous,control_dt),"paused clock advances control");
    require(!advance_control_time(0,clock_previous,control_dt) && clock_previous==0,"clock reset not handled");
    ThrustRelease release;
    release.update(1,.80,1.2,1.98,120);
    require(!release.active(),"low-speed stroke prematurely released");
    release.update(1.01,.50,2.0,1.98,120);
    require(!release.active(),"early stroke prematurely released");
    release.update(1.02,.95,1.91,1.98,120);
    require(release.active() && release.force_limit(1.02)==120,"release boundary not continuous");
    require(release.brake_blend(1.02,.65)==.65,"initial brake not preserved");
    require(release.brake_blend(1.025,0)==.65,"speed loss removed terminal braking");
    double previous_force=120;
    for (int i=1;i<=60;++i) {
        const double t=1.02+.001*i;
        release.update(t,1.0,.7,1.98,240);
        require(release.force_limit(t)<=previous_force+1e-10,"speed loss restarted the propulsive pulse");
        require(release.force_limit(t)>=-1e-10,"release force reversed");
        previous_force=release.force_limit(t);
    }
    require(previous_force==0,"release failed to unload");
    release.reset();
    require(!release.active(),"next jump inherited release latch");
    // 8.355s: near upright but rolling backward at 0.496 m/s. Old 2-frame
    // criterion released CATCH here and PREPARE never recaptured the next fall.
    require(!touchdown_release_ready(-.0577667-.038,.224383,-.0501292,-.495928,.588),
            "moving support released capture");
    require(touchdown_release_ready(.02,.05,.03,.04,-.02),"settled capture rejected");
    require(touchdown_capture_lost(-.20,-1.0,-.40),"backward fall stranded in PREPARE");
    require(touchdown_capture_lost(.20,1.0,.40),"forward fall stranded in BRAKE");
    require(!touchdown_capture_lost(-.20,1.0,-.01),"recovering body triggered recapture");
    require(!touchdown_capture_lost(.01,.02,.015),"noise triggered recapture");
    require(touchdown_catch_limit(-.30,-1.5,.4)==1.5,"diverging catch keeps insufficient fixed ceiling");
    require(touchdown_catch_limit(-.30,1.5,.4)==.9,"recovering body widens wheel ceiling");
    const double start=.113, v=.45, duration=.07;
    auto initial=thrust_pitch_reference(start,v,duration,0);
    require(initial.angle==start && initial.rate==v,"initial reference handoff changed");
    const auto end=thrust_pitch_reference(start,v,duration,duration);
    require(std::abs(end.angle-(start+.5*v*duration))<1e-12 && end.rate==0,
            "finite reference still requests forward rotation");
    for (int i=1;i<200;++i) {
        const double t=.001*i, eps=1e-6;
        const auto ref=thrust_pitch_reference(start,v,duration,t);
        const double derivative=(thrust_pitch_reference(start,v,duration,t+eps).angle-
                                 thrust_pitch_reference(start,v,duration,t-eps).angle)/(2*eps);
        require(std::abs(derivative-ref.rate)<1e-8,"angle and rate references disagree");
        require(ref.angle>=start && ref.angle<=end.angle+1e-12,"reference overshoot");
    }
    require(thrust_pitch_reference(start,v,0,.1).rate==0,"zero lead creates continuing rotation");
    require(thrust_extension_velocity(-7.0,6.0,-1.0,1.0)==-6.0,
            "force stroke reverts to zero speed or disables overspeed damping");
    require(thrust_extension_velocity(-7.0,6.0,-1.0,0.0)==0.0,"travel stop bypassed");
    require(thrust_extension_velocity(2.0,6.0,-1.0,1.0)==0.0,"collapsing knee treated as extension");
    require(thrust_extension_velocity(2.0,3.0,1.0,.5)==1.0,"hip extension safety scale ignored");
    const double kp=-233.4004, kd=-62.6391;
    // Recorded nominal-stroke end: positive lean/rate while legacy wheel command
    // switches towards positive/reverse. Ground feedback must move support forward.
    const double cmd=thrust_ground_wheel_target(.50,.204832-end.angle,.645681,kp,kd);
    require(cmd<-.50 && cmd>=-1.50,"forward falling thrust asks wheels to retreat");
    require(thrust_ground_wheel_target(.50,0,0,kp,kd)==-.50,"steady rolling target changed");
    require(thrust_ground_wheel_target(.50,-.10,-.5,kp,kd)>-.50,"backward disturbance not corrected");
    double blend=landing_wheel_blend(0,true,false,.01);
    require(blend==0,"ascending wheel clearance triggered landing handoff");
    blend=landing_wheel_blend(blend,true,true,.05);
    require(std::abs(blend-.5)<1e-12,"midpoint handoff incorrect");
    require(landing_wheel_blend(blend,true,true,.07)==blend,"clearance noise reversed handoff");
    require(landing_wheel_blend(blend,false,true,0)==blend,"invalid geometry changed blend");
    blend=landing_wheel_blend(blend,true,true,.02);
    require(blend==1,"ground wheel command not ready near contact");
    // Actual landing row: pitch .262641, rate 1.05284, wheels retreating.
    // This is divergent, even though the old catch_release condition accepted it.
    const double pitch=.262641-.038, rate=1.05284;
    const double capture=pitch+rate/std::sqrt(9.81/.5);
    require(!touchdown_capture_settled(pitch,rate,capture),"forward fall releases CATCH");
    require(!touchdown_capture_settled(-pitch,-rate,-capture),"backward fall releases CATCH");
    require(touchdown_capture_settled(.02,.05,.03),"stable capture never releases");
    const double landing_cmd=catch_wheel_target(.262641-(.038+.050),rate,.30,kp,kd,.043);
    require(landing_cmd<0,"landing forward fall has backward wheel target");
    const double airborne_cmd=.659194;
    const double blended=(1-blend)*airborne_cmd+blend*landing_cmd;
    require(blended==landing_cmd,"airborne reversal leaks through at contact");
    require(std::abs(catch_wheel_target(0,0,.02,kp,kd,.043))<1e-12,"velocity deadband removed");
    std::cout<<"PASS: consistent pitch references, grounded support direction, landing handoff, capture release\n";
}
