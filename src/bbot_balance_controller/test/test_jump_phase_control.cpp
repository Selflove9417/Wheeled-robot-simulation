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
    release.update(1,.80,1.37,1.98,120);
    require(!release.active(),"low-speed stroke prematurely released");
    release.update(1.01,.50,1.39,1.98,120);
    require(release.active(),"predictive unload did not lead target speed");
    release.reset();
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
    require(thrust_brake_factor(false, false, .8)==0.0,
            "propulsive stroke braked before speed release");
    require(std::abs(thrust_brake_factor(false, true, .8)-.8)<1e-12,
            "speed release did not brake during airborne confirmation gap");
    require(std::abs(thrust_brake_factor(true, false, .6)-.6)<1e-12,
            "airborne leg did not retain terminal brake");
    require(thrust_terminal_hip_command(-5.85,false,true,true,.015,true,1.27)==-5.85,
            "default hip path changed");
    require(thrust_terminal_hip_command(-5.85,true,true,true,.015,true,1.27)==0.0,
            "late negative hip effort was not isolated");
    require(thrust_terminal_hip_command(9.22,true,true,true,.015,true,.71)==9.22,
            "positive posture effort was reduced");
    require(thrust_terminal_hip_command(-5.85,true,false,true,.015,true,1.27)==-5.85,
            "early thrust was modified");
    require(thrust_terminal_hip_command(-5.85,true,true,false,.015,true,1.27)==-5.85,
            "invalid geometry enabled the diagnostic");
    require(thrust_terminal_hip_command(-5.85,true,true,true,.002,true,1.27)==-5.85,
            "near-ground request was modified");
    require(thrust_terminal_hip_command(-5.85,true,true,true,.015,false,1.27)==-5.85,
            "stale IMU enabled the diagnostic");
    require(std::abs(ballistic_time_to_height(.674, .211, .394)-.2614)<.001,
            "COM ballistic deadline changed by retracting base motion");
    require(ballistic_time_to_height(NAN, 0.0, .4)==0.0,
            "invalid COM height produced a landing deadline");
    // Recorded first EXTEND sample: the old geometric projection changed the
    // knee target by -0.60 rad and reversed +4.32 to -0.95 rad/s in one cycle.
    // The bounded projection must move toward wheel-first without recreating
    // that body impulse.
    const auto wheel_first = bounded_wheel_first_target(
        .680, -.599, -1.575, 4.321, -.301, -2.520,
        std::atan2(.28210870, .19553796), .750);
    require(wheel_first.active, "unsafe landing shank was not projected");
    require(std::abs(wheel_first.knee_position - (-.629)) < 1e-12,
            "wheel-first position projection is still discontinuous");
    require(std::abs(wheel_first.knee_velocity - 3.321) < 1e-12,
            "wheel-first velocity projection still reverses in one sample");
    const auto inward = bounded_wheel_first_target(
        .680, -.599, -1.575, -2.0, -.301, -2.520,
        std::atan2(.28210870, .19553796), .750);
    require(inward.knee_velocity == -2.0,
            "wheel-first projection slowed an already-safe inward motion");
    // 8.355s: near upright but rolling backward at 0.496 m/s. Old 2-frame
    // criterion released CATCH here and PREPARE never recaptured the next fall.
    require(!touchdown_release_ready(-.0577667-.038,.224383,-.0501292,-.495928,.588),
            "moving support released capture");
    require(touchdown_release_ready(.02,.05,.03,.04,-.02),"settled capture rejected");
    require(touchdown_capture_lost(-.20,-1.0,-.40),"backward fall stranded in PREPARE");
    require(touchdown_capture_lost(.20,1.0,.40),"forward fall stranded in BRAKE");
    require(!touchdown_capture_lost(-.20,1.0,-.01),"recovering body triggered recapture");
    require(!touchdown_capture_lost(.01,.02,.015),"noise triggered recapture");
    // Logged failure case: torso pitch appeared recovering, but COM was severely backward
    // (com_lean = -0.562, capture_state = -0.997). Must NOT trigger reverse brake!
    require(!touchdown_reverse_brake_ready(-0.339, -0.236, -0.997, -0.562, 1.641),
            "logged backward COM lean must not trigger reverse braking");
    require(!touchdown_reverse_brake_ready(-0.45, 0.39, -0.05, -0.04, 0.10),
            "reverse braking started before support velocity crossed zero");
    require(!touchdown_reverse_brake_ready(0.05, -0.09, -0.05, -0.04, 0.10),
            "reverse braking started before com velocity crossed zero");
    require(!touchdown_reverse_brake_ready(-0.75, -0.09, -0.25, -0.04, 0.10),
            "reverse braking started while capture state still too negative");
    require(!touchdown_reverse_brake_ready(-0.75, -0.09, -0.05, -0.15, 0.10),
            "reverse braking started while balance angle still too negative");
    require(!touchdown_reverse_brake_ready(-0.75, -0.09, -0.05, -0.04, -0.40),
            "reverse braking started while balance rate still too negative");
    // Valid zero-crossing case near neutral point:
    require(touchdown_reverse_brake_ready(-0.339, -0.236, -0.05, -0.04, 0.10),
            "near-neutral recovered COM state must start reverse braking");
    require(std::abs(thrust_knee_brake_reaction_compensation(22.0, 22.0) - 8.0) < 1e-12,
            "thrust knee-brake reaction compensation did not respect its limit");
    require(thrust_knee_brake_reaction_compensation(-20.0, -18.0) == 0.0,
            "ordinary knee extension incorrectly activated brake compensation");
    require(std::abs(thrust_knee_brake_reaction_compensation(10.0, 6.0) - 3.2) < 1e-12,
            "asymmetric knee braking was not averaged before hip compensation");
    require(landing_pitch_unrecoverable(-.70, -.80),
            "diverging landing pitch bypassed the fall guard");
    require(!landing_pitch_unrecoverable(-.70, .20),
            "recovering landing pitch was terminated by the fall guard");
    require(landing_pitch_unrecoverable(-.92, .20),
            "extreme landing pitch bypassed the hard guard");
    require(landing_pitch_unrecoverable(-.224, -1.188, .22, .65),
            "reverse-brake recapture did not detect the recorded early torso divergence");
    // reverse_brake_capture_diverged unit tests:
    // 1. Early trigger with recorded 174545 / 174454 values (-0.173, fresh observation):
    require(reverse_brake_capture_diverged(true, 0.009, -0.173, -0.16),
            "failed to trigger recapture on recorded early capture divergence");
    // 2. Normal non-trigger when capture is healthy (> -0.16):
    require(!reverse_brake_capture_diverged(true, 0.009, +0.069, -0.16),
            "falsely triggered recapture on positive capture state");
    require(!reverse_brake_capture_diverged(true, 0.009, -0.120, -0.16),
            "falsely triggered recapture on moderate negative capture state");
    // 3. Invalid capture observation must NOT trigger:
    require(!reverse_brake_capture_diverged(false, 0.009, -0.250, -0.16),
            "falsely triggered recapture when capture validity was false");
    // 4. Stale observation (> 80 ms) must NOT trigger:
    require(!reverse_brake_capture_diverged(true, 0.085, -0.250, -0.16),
            "falsely triggered recapture on stale observation age");
    require(!reverse_brake_capture_diverged(true, -0.001, -0.250, -0.16),
            "falsely triggered recapture on negative observation age");
    require(!reverse_brake_capture_diverged(true, 0.009, std::numeric_limits<double>::quiet_NaN(), -0.16),
            "falsely triggered recapture on NaN capture state");
    require(landing_knee_extension_guard(-1.39, -8.2, -20.0) > 40.0,
            "fast touchdown rebound was still driven into the knee hard stop");
    require(landing_knee_extension_guard(-1.18, -8.2, -20.0) == -20.0,
            "knee guard acted outside its extension warning zone");
    require(landing_knee_extension_guard(-1.39, 4.0, -20.0) == -20.0,
            "knee guard opposed normal landing compression");
    require(landing_common_hip_damping(16.0, 14.0) == -12.0,
            "touchdown common hip speed was not damped at the configured limit");
    require(landing_common_hip_damping(-3.0, -5.0) == 4.0,
            "touchdown common hip damping has the wrong sign");
    const auto long_leg_buffer = landing_buffer_profile(.670, .340, .500);
    require(std::abs(long_leg_buffer.target_height - .500) < 1e-12,
            "long landing collapsed directly into the deep crouch");
    require(std::abs(long_leg_buffer.duration - .510) < 1e-12,
            "long landing ignored the smoothstep height-rate limit");
    const auto folded_buffer = landing_buffer_profile(.450, .340, .500);
    require(std::abs(folded_buffer.target_height - .340) < 1e-12,
            "short landing lost its configured minimum buffer height");
    require(folded_buffer.duration >= .18,
            "landing buffer duration fell below the legacy minimum");
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

    // Asymmetric rate limiter unit tests:
    // dt = 0.005s: max_step_accel = 8.0 * 0.005 = 0.040, max_step_decel = 24.0 * 0.005 = 0.120
    const double cur_cmd = -0.50;
    // 1. Accelerating forward: target = -0.80 (< -0.50). delta = -0.30, clamped to -0.040 -> next = -0.540
    const double next_accel = thrust_wheel_rate_limit(-0.80, cur_cmd, 0.005, 8.0, 24.0);
    require(std::abs(next_accel - (-0.540)) < 1e-9, "forward acceleration not limited to max_accel step");

    // 2. Decelerating towards zero/positive: target = 0.0 (> -0.50). delta = +0.50, clamped to +0.120 -> next = -0.380
    const double next_decel = thrust_wheel_rate_limit(0.0, cur_cmd, 0.005, 8.0, 24.0);
    require(std::abs(next_decel - (-0.380)) < 1e-9, "deceleration/braking not limited to max_decel step");

    // 3. Small change within limits: target = -0.52 (delta = -0.02, within 0.040)
    const double next_small = thrust_wheel_rate_limit(-0.52, cur_cmd, 0.005, 8.0, 24.0);
    require(std::abs(next_small - (-0.520)) < 1e-9, "small acceleration delta altered");

    // 4. Small decel change within limits: target = -0.45 (delta = +0.05, within 0.120)
    const double next_small_decel = thrust_wheel_rate_limit(-0.45, cur_cmd, 0.005, 8.0, 24.0);
    require(std::abs(next_small_decel - (-0.450)) < 1e-9, "small decel delta altered");

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
