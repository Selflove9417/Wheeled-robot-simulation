#include "bbot_balance_controller/jump_phase_control.hpp"
#include "bbot_balance_controller/control_timing.hpp"
#include "bbot_balance_controller/thrust_forward_speed_predictor.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
int main() {
    using namespace bbot_jump;
    // Recorded protected smoke sample at ~30 ms: torso stable for seven
    // samples, safe joint positions, but knee speed still -9.46025 rad/s.
    // Time remained available, so ARREST must continue until the original
    // joint-speed gate passes rather than prematurely deploying protection.
    require(stable_arrest_action(.030, 7, false, true, false, false, false) ==
                StableArrestAction::continue_arrest,
            "stable torso with remaining time must keep arresting unsafe joint speed");
    require(stable_arrest_action(.040, 8, false, true, true, false, false) ==
                StableArrestAction::start_checked_tuck,
            "safe joint speed after arrest must still enter the checked tuck planner");
    require(stable_arrest_action(.040, 8, false, false, false, false, false) ==
                StableArrestAction::protective_deploy,
            "insufficient remaining flight time must still choose protective deployment");
    require(stable_arrest_action(.080, 8, false, true, false, true, false) ==
                StableArrestAction::protective_deploy,
            "normal 80 ms ARREST timeout must still protect unsafe joint speed");
    require(stable_arrest_action(.080, 8, false, true, true, true, true) ==
                StableArrestAction::start_checked_tuck,
            "existing safe late-tuck exception must retain full trajectory validation");
    require(stable_arrest_action(.029, 7, false, true, true, false, false) ==
                StableArrestAction::continue_arrest &&
            stable_arrest_action(.040, 1, false, true, true, false, false) ==
                StableArrestAction::continue_arrest,
            "stable ARREST handoff must still require its original time and count gates");

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

    StableDurationNs stable_duration;
    const int64_t thrust_gate_ns = 20'000'000;
    for (const int64_t stamp : {3'075'000'000LL, 3'080'000'000LL,
                                3'085'000'000LL, 3'090'000'000LL}) {
        require(stable_duration.update(true, stamp) < thrust_gate_ns,
                "3.075 through 3.090 provides only 15 ms of observed stability");
    }
    require(stable_duration.update(false, 3'095'000'000LL) == 0,
            "out-of-gate 3.095 sample resets the continuous stable interval");
    require(stable_duration.update(true, 3'100'000'000LL) == 0 &&
            stable_duration.update(true, 3'105'000'000LL) == 5'000'000 &&
            stable_duration.update(true, 3'120'000'000LL) == thrust_gate_ns,
            "only distinct stable simulation stamps accumulate the required 20 ms");
    stable_duration.reset();
    require(stable_duration.update(true, 1'000'000'000LL) == 0 &&
            stable_duration.update(true, 1'006'000'000LL) == 6'000'000 &&
            stable_duration.update(true, 1'014'000'000LL) == 14'000'000 &&
            stable_duration.update(true, 1'020'000'000LL) == thrust_gate_ns,
            "irregular sample intervals accumulate real timestamp span");
    require(stable_duration.update(true, 1'020'000'000LL) == thrust_gate_ns,
            "duplicate timestamp does not manufacture additional elapsed time");
    require(stable_duration.update(true, 900'000'000LL) == 0 &&
            stable_duration.update(true, 920'000'000LL) == thrust_gate_ns,
            "clock rollback starts a fresh interval without bridging epochs");
    require(stable_duration.update(false, 930'000'000LL) == 0 &&
            stable_duration.update(true, 935'000'000LL) == 0,
            "failed gate sample resets interval and next valid sample seeds it");
    require(stable_duration.update(true, -1) == 0 && stable_duration.elapsed_ns() == 0,
            "invalid simulation stamp clears stable evidence");

    // SQUAT and THRUST trajectories have distinct IK seeds. The first THRUST
    // sample is not 5 ms of joint motion, even if its planned IK differs from
    // the final SQUAT target; later samples must retain the ordinary rate.
    const double squat_hip_target = -0.163064;
    const double thrust_seed_hip_target = -0.155870;
    require(sampled_joint_reference_rate(thrust_seed_hip_target, squat_hip_target,
                                         0.005, 1.0, 3.0, true) == 0.0,
            "new THRUST IK seed must not create a cross-phase finite-difference pulse");
    require(std::abs(sampled_joint_reference_rate(-0.150870, thrust_seed_hip_target,
                                                  0.005, 1.0, 3.0, false) - 1.0) < 1e-12,
            "same-trajectory IK motion retains the actual finite-difference rate");
    require(std::abs(sampled_joint_reference_rate(0.187796, 0.195562,
                                                  0.005, 1.0, 6.0, true)) < 1e-12,
            "new THRUST knee seed also suppresses cross-phase reference motion");
    require(sampled_joint_reference_rate(0.005, 0.0, 0.005, 1.0, 3.0, false) == 1.0 &&
            sampled_joint_reference_rate(0.005, 0.0, 0.005, 0.5, 3.0, false) == 0.5 &&
            sampled_joint_reference_rate(1.0, 0.0, 0.005, 1.0, 3.0, false) == 3.0,
            "same-trajectory finite difference keeps travel scaling and speed limits");

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
    ThrustRelease default_ratio_release, explicit_baseline_release;
    default_ratio_release.update(2.0, .5, 1.40, 2.0, 100.0);
    explicit_baseline_release.update(2.0, .5, 1.40, 2.0, 100.0, .70);
    require(default_ratio_release.active() && explicit_baseline_release.active() &&
            default_ratio_release.blend(2.02) == explicit_baseline_release.blend(2.02) &&
            default_ratio_release.force_limit(2.02) == explicit_baseline_release.force_limit(2.02),
            "explicit 0.70 release ratio changed the legacy default behavior");
    ThrustRelease ratio074_release;
    ratio074_release.update(3.0, .5, 1.40, 1.98, 120.0, .74);
    require(!ratio074_release.active(), "0.74 release activated at the old 0.70 threshold");
    ratio074_release.update(3.01, .5, 1.466, 1.98, 120.0, .74);
    require(ratio074_release.active() &&
            std::abs(ratio074_release.blend(3.01)) < 1e-12,
            "0.74 release did not begin at its configured speed threshold");
    double ratio074_previous_force = ratio074_release.force_limit(3.01);
    for (int i = 1; i <= 40; ++i) {
        const double t = 3.01 + .001 * i;
        ratio074_release.update(t, .5, .1, 1.98, 500.0, .78);
        const double force_now = ratio074_release.force_limit(t);
        require(force_now <= ratio074_previous_force + 1e-10,
                "active 0.74 release restarted or unloaded non-monotonically");
        ratio074_previous_force = force_now;
    }
    require(ratio074_previous_force < 1e-8,
            "0.74 release did not complete the unchanged 40 ms unload");
    for (double invalid_ratio : {.69, .79, std::numeric_limits<double>::quiet_NaN()}) {
        ThrustRelease invalid_release;
        invalid_release.update(4.0, .5, 1.99, 2.0, 100.0, invalid_ratio);
        require(!invalid_release.active(), "invalid release ratio activated unload");
    }
    ThrustRelease invalid_force_release;
    invalid_force_release.update(4.0, .5, 1.99, 2.0,
                                 std::numeric_limits<double>::quiet_NaN(), .74);
    require(!invalid_force_release.active(), "non-finite release force activated unload");
    require(thrust_brake_factor(false, false, .8)==0.0,
            "propulsive stroke braked before speed release");
    require(std::abs(thrust_brake_factor(false, true, .8)-.8)<1e-12,
            "speed release did not brake during airborne confirmation gap");
    require(std::abs(thrust_brake_factor(true, false, .6)-.6)<1e-12,
            "airborne leg did not retain terminal brake");
    require(std::abs(thrust_hip_extension_velocity_scale(false, 1.0)-.65)<1e-12,
            "Position-first hip release behavior changed");
    require(std::abs(thrust_hip_extension_velocity_scale(true, .5)-.5)<1e-12 &&
            thrust_hip_extension_velocity_scale(true, 1.0)==0.0,
            "Effort continuation hip extension reference did not follow release ramp to zero");
    require(std::abs(thrust_knee_reference_velocity_scale(1.0)-.65)<1e-12 &&
            std::abs(thrust_knee_reference_velocity_scale(.5)-.825)<1e-12,
            "knee inverse release reference changed while isolating the hip brake");
    require(!preserve_effort_support_handoff(false, false) &&
            !preserve_effort_support_handoff(false, true) &&
            !preserve_effort_support_handoff(true, false) &&
            preserve_effort_support_handoff(true, true),
            "handoff must skip prewrite only for an already active Effort continuation");
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
    const double target_speed=.45;
    require(thrust_forward_attitude_scale(.35,target_speed,true)==1.0,
            "speed-taper changed attitude correction below its entry speed");
    require(std::abs(thrust_forward_attitude_scale(.39,target_speed,true)-.5)<1e-12,
            "speed-taper midpoint is not continuous smoothstep");
    require(thrust_forward_attitude_scale(.43,target_speed,true)==0.0,
            "speed-taper left forward acceleration at its exit speed");
    require(thrust_forward_attitude_scale(.44,target_speed,true)==0.0,
            "speed-taper re-enabled above its exit speed");
    require(thrust_forward_attitude_scale(.39,target_speed,false)==1.0,
            "stale COM data did not preserve the old attitude law");
    require(thrust_forward_attitude_scale(std::numeric_limits<double>::quiet_NaN(),target_speed,true)==1.0,
            "invalid COM speed did not fall back to the old attitude law");
    const double attenuated_accel=apply_thrust_forward_attitude_taper(-.8,.5,true);
    const double preserved_brake=apply_thrust_forward_attitude_taper(+.8,.0,true);
    require(std::abs(attenuated_accel+.4)<1e-12,
            "negative attitude acceleration was not tapered");
    require(preserved_brake==.8,
            "positive attitude braking was attenuated");
    require(apply_thrust_forward_attitude_taper(-.8,.5,false)==-.8,
            "disabled taper changed the baseline attitude term");
    require(thrust_ground_wheel_target(.85,-.65)==-1.50 &&
            thrust_ground_wheel_target(0.0,+1.50)==1.50,
            "existing +/-1.50 wheel target limits changed");

    // Position-based THRUST speed prediction: adjacent intervals are
    // midpoint velocities and only the three latest distinct aligned frames
    // contribute. This predicts a feedback input; it does not rewrite the
    // measured COM velocity or prove closed-loop jump performance.
    const std::array<double,2> forward_y{0.0,1.0};
    ThrustForwardSpeedPredictor speed_predictor;
    require(speed_predictor.observe(0.0,{0.0,0.0,0.0},forward_y) &&
            speed_predictor.observe(0.02,{0.0,0.0084,0.0},forward_y) &&
            speed_predictor.observe(0.04,{0.0,0.0176,0.0},forward_y),
            "constant-acceleration position samples were not accepted");
    auto prediction=speed_predictor.predict(0.05,.75,true,true);
    require(prediction.active && std::abs(prediction.raw_speed-.75)<1e-12 &&
            std::abs(prediction.base_speed-.46)<1e-12 &&
            std::abs(prediction.acceleration-2.0)<1e-12 &&
            std::abs(prediction.horizon-.02)<1e-12 &&
            std::abs(prediction.delta_speed-.04)<1e-12 &&
            std::abs(prediction.predicted_speed-.50)<1e-12,
            "constant-acceleration midpoint prediction did not use the frozen-axis interval speed base");
    require(!speed_predictor.observe(.04,{0.0,.0176,0.0},forward_y) &&
            speed_predictor.sample_count()==3 &&
            speed_predictor.predict(.05,.46,true,true).active,
            "identical duplicate timestamp added evidence or invalidated valid history");
    require(!speed_predictor.observe(.04,{.1,.0176,0.0},forward_y) &&
            speed_predictor.sample_count()==0,
            "same-stamp orthogonal world-position conflict was not rejected");

    // Unequal sample cadence still uses the elapsed interval between velocity
    // midpoints, rather than assuming a fixed 20 ms odometry period.
    ThrustForwardSpeedPredictor uneven_predictor;
    require(uneven_predictor.observe(0.0,{0.0,0.0,0.0},forward_y) &&
            uneven_predictor.observe(.02,{0.0,.0084,0.0},forward_y) &&
            uneven_predictor.observe(.05,{0.0,.0225,0.0},forward_y),
            "uneven-interval positions were rejected");
    prediction=uneven_predictor.predict(.06,.47,true,true);
    require(prediction.active && std::abs(prediction.acceleration-2.0)<1e-12 &&
            std::abs(prediction.horizon-.025)<1e-12 &&
            std::abs(prediction.delta_speed-.05)<1e-12 &&
            std::abs(prediction.predicted_speed-.52)<1e-12,
            "uneven-interval midpoint timing was ignored");

    // Negative acceleration stays visible for diagnosis but cannot lower the
    // prediction below the frozen-axis interval mean and re-enable forward
    // attitude drive through the speed feedback/taper inputs.
    ThrustForwardSpeedPredictor deceleration_predictor;
    require(deceleration_predictor.observe(0.0,{0.0,0.0,0.0},forward_y) &&
            deceleration_predictor.observe(.02,{0.0,.0100,0.0},forward_y) &&
            deceleration_predictor.observe(.04,{0.0,.0184,0.0},forward_y),
            "deceleration position samples were rejected");
    prediction=deceleration_predictor.predict(.05,.50,true,true);
    const double decel_feedback_speed = .45 + 1.20*(.45-prediction.predicted_speed);
    const double raw_feedback_speed = .45 + 1.20*(.45-prediction.base_speed);
    require(prediction.active && std::abs(prediction.acceleration+4.0)<1e-12 &&
            prediction.delta_speed==0.0 &&
            std::abs(prediction.predicted_speed-.42)<1e-12 &&
            decel_feedback_speed==raw_feedback_speed &&
            thrust_forward_attitude_scale(prediction.predicted_speed,.45,true)==
                thrust_forward_attitude_scale(prediction.base_speed,.45,true) &&
            decel_feedback_speed < .45+1.20*(.45-.34),
            "negative acceleration lead lowered speed feedback and re-enabled forward drive");

    // Replay the failed campaign J1 samples at 8.140/8.160/8.180 s. The
    // result is only the proposed feedback value, never the measured/fitted
    // takeoff speed or evidence of a successful closed-loop intervention.
    ThrustForwardSpeedPredictor failed_trace_replay;
    require(failed_trace_replay.observe(8.140,{.200006,1.083580,.50},forward_y) &&
            failed_trace_replay.observe(8.160,{.200006,1.094060,.50},forward_y) &&
            failed_trace_replay.observe(8.180,{.200006,1.105480,.50},forward_y),
            "failed-trace COM frames did not seed the predictor");
    prediction=failed_trace_replay.predict(8.185,.571293,true,true);
    require(prediction.active && prediction.acceleration>2.0 &&
            prediction.delta_speed>0.0 && prediction.delta_speed<=.08 &&
            prediction.predicted_speed>prediction.raw_speed,
            "failed-trace offline replay did not yield a bounded forward prediction");

    // Acceleration and delta clamps, freshness boundaries, invalid data,
    // rollback, long-gap reset, and explicit per-jump/phase reset.
    ThrustForwardSpeedPredictor bounded_predictor;
    require(bounded_predictor.observe(0.0,{0.0,0.0,0.0},forward_y) &&
            bounded_predictor.observe(.02,{0.0,.0016,0.0},forward_y) &&
            bounded_predictor.observe(.04,{0.0,.0064,0.0},forward_y),
            "bounded acceleration samples were rejected");
    prediction=bounded_predictor.predict(.07,.24,true,true);
    require(prediction.active && prediction.acceleration==4.0 &&
            std::abs(prediction.horizon-.04)<1e-12 &&
            std::abs(prediction.delta_speed-.08)<1e-12,
            "acceleration, horizon, or delta upper bound failed");
    require(!bounded_predictor.predict(.0701,.24,true,true).active &&
            bounded_predictor.sample_count()==0,
            "overlong prediction horizon was not rejected/reset");
    require(bounded_predictor.observe(1.0,{0.0,0.0,0.0},forward_y),
            "predictor could not rebuild after horizon expiry");
    require(!bounded_predictor.predict(1.041,.2,true,true).active &&
            bounded_predictor.sample_count()==0,
            "stale one-frame history was retained while candidate source remained nominally fresh");
    require(bounded_predictor.observe(1.05,{0.0,0.0,0.0},forward_y),
            "predictor could not rebuild after stale one-frame history");
    require(!bounded_predictor.predict(1.06,.2,false,true).active &&
            bounded_predictor.sample_count()==0,
            "stale COM source did not clear prediction history");
    require(!bounded_predictor.observe(1.02,{0.0,.01,0.0},
                                      {std::numeric_limits<double>::quiet_NaN(),0.0}) &&
            bounded_predictor.sample_count()==0,
            "invalid frozen axis did not clear prediction history");
    require(bounded_predictor.observe(1.0,{0.0,0.0,0.0},forward_y) &&
            bounded_predictor.observe(1.02,{0.0,.01,0.0},forward_y) &&
            bounded_predictor.observe(1.04,{0.0,.02,0.0},forward_y),
            "predictor could not seed after invalid-axis rejection");
    require(!bounded_predictor.observe(1.03,{0.0,.015,0.0},forward_y) &&
            bounded_predictor.sample_count()==0,
            "timestamp rollback did not clear prediction history");
    require(bounded_predictor.observe(2.0,{0.0,0.0,0.0},forward_y) &&
            bounded_predictor.observe(2.02,{0.0,.01,0.0},forward_y) &&
            !bounded_predictor.observe(2.2,{0.0,.10,0.0},forward_y) &&
            bounded_predictor.sample_count()==1,
            "long interval did not reset and seed a fresh history");
    require(!bounded_predictor.observe(2.22,{0.0,std::numeric_limits<double>::quiet_NaN(),0.0},forward_y) &&
            bounded_predictor.sample_count()==0,
            "NaN COM position did not clear prediction history");
    require(speed_predictor.observe(3.0,{0.0,0.0,0.0},forward_y) &&
            speed_predictor.observe(3.02,{0.0,.01,0.0},forward_y) &&
            speed_predictor.observe(3.04,{0.0,.02,0.0},forward_y),
            "predictor could not seed before phase-reset check");
    speed_predictor.reset();
    require(speed_predictor.sample_count()==0 &&
            !speed_predictor.predict(3.05,.5,true,true).active,
            "explicit new-jump/phase reset did not clear prediction history");

    // G4 J1 release replay, active reference zero, KD=12. At 3.495s the
    // legacy +13.4672 Nm body command over-damped a rate the fast filter saw
    // earlier; the signed correction backs off by 3.5020 Nm. At 3.535s the
    // legacy -9.68503 Nm command continued braking after the true rate reversed;
    // the signed correction backs off by 3.7871 Nm. These are command replays,
    // not measured actuator torque or proof of closed-loop improvement.
    const double excess_positive_damping = thrust_release_fast_rate_correction(
        true, true, true, true, 1.0, -0.7587, -0.175037, 12.0, 8.0);
    const double excess_negative_damping = thrust_release_fast_rate_correction(
        true, true, true, true, 1.0, 1.12581, 0.494626, 12.0, 8.0);
    require(std::abs(excess_positive_damping + 3.5020) < 1e-3 &&
            std::abs(excess_negative_damping - 3.7871) < 1e-3,
            "G4 release replay did not back off over-damping with the signed lead");
    const auto replay_positive = apply_thrust_rate_correction(
        13.4672, excess_positive_damping, 20.0);
    const auto replay_negative = apply_thrust_rate_correction(
        -9.68503, excess_negative_damping, 20.0);
    require(std::abs(replay_positive.command - 9.9652) < 1e-3 &&
            std::abs(replay_positive.applied + 3.5020) < 1e-3 &&
            std::abs(replay_negative.command + 5.89793) < 1e-3 &&
            std::abs(replay_negative.applied - 3.7871) < 1e-3,
            "G4 signed-lead commands or applied increments were incorrect");
    require(std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .083, 1.510, 12.0, 8.0) + 8.0) < 1e-12,
            "previous fast-forward-rate replay lost its bounded missing damping");
    require(std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, -0.0162426, -1.14857, 12.0, 8.0) - 6.7940) < 1e-3,
            "previous failed backward-rate replay changed under signed-lead strategy");
    require(std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, .25, 1.12581, .494626, 12.0, 8.0) -
                excess_negative_damping * .25) < 1e-12,
            "release blend did not scale the signed correction");
    require(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .125, .125, 12.0, 8.0) == 0.0,
            "equal legacy and fast rates produced a nonzero correction");
    require(std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .10, 0.0, 12.0, 8.0) - .6) < 1e-12 &&
            std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .10, -.001, 12.0, 8.0) - .606) < 1e-12 &&
            std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .10, .001, 12.0, 8.0) - .594) < 1e-12,
            "signed correction was discontinuous through zero fast rate");
    require(std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, .10, .05, 12.0, 8.0) - .3) < 1e-12 &&
            std::abs(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, -.10, -.05, 12.0, 8.0) + .3) < 1e-12,
            "stronger same-sign legacy damping was not reduced in both directions");
    require(thrust_release_fast_rate_correction(
                true, true, true, true, 1.0, -10.0, 10.0, 12.0, 8.0) == -8.0 &&
            thrust_release_fast_rate_correction(
                true, true, true, true, .25, 10.0, -10.0, 12.0, 8.0) == 2.0,
            "signed correction did not obey symmetric +/-8 Nm clamp and blend");
    require(thrust_release_fast_rate_correction(
                false, true, true, true, 1.0, -0.0162426, -1.14857, 12.0, 8.0) == 0.0 &&
            thrust_release_fast_rate_correction(
                true, effort_release_rate_correction_eligible(false, false),
                true, true, 1.0, -0.0162426, -1.14857, 12.0, 8.0) == 0.0,
            "disabled or Position-supported PRE/SQUAT activated the correction");
    require(!effort_release_rate_correction_eligible(false, false) &&
            !effort_release_rate_correction_eligible(true, true) &&
            effort_release_rate_correction_eligible(true, false),
            "release eligibility ignored confirmed Effort mode or a pending switch");
    const auto gate_blend = [](double elapsed, bool enabled = true,
                               bool in_thrust = true, bool effort = true,
                               bool pending = false, bool gate_open = true,
                               bool imu_fresh = true) {
        return thrust_gate_fast_rate_correction_blend(
            enabled, in_thrust, effort, pending, gate_open, imu_fresh,
            elapsed, 0.020);
    };
    require(gate_blend(0.0) == 0.0 &&
            std::abs(gate_blend(.005) - .15625) < 1e-12 &&
            std::abs(gate_blend(.010) - .5) < 1e-12 &&
            gate_blend(.020) == 1.0 && gate_blend(.030) == 1.0,
            "gate-scope correction did not smoothstep over exactly 20 ms");
    require(gate_blend(.020, false) == 0.0 &&
            gate_blend(.020, true, false) == 0.0 &&
            gate_blend(.020, true, true, false) == 0.0 &&
            gate_blend(.020, true, true, true, true) == 0.0 &&
            gate_blend(.020, true, true, true, false, false) == 0.0 &&
            gate_blend(.020, true, true, true, false, true, false) == 0.0,
            "gate-scope correction ignored phase, Effort, pending, gate, or freshness guard");
    require(gate_blend(-.001) == 0.0 &&
            gate_blend(std::numeric_limits<double>::quiet_NaN()) == 0.0 &&
            thrust_gate_fast_rate_correction_blend(
                true, true, true, false, true, true, .020, 0.0) == 0.0 &&
            gate_blend(0.0) == 0.0,
            "invalid elapsed/duration or hop-reset blend did not stay inactive");
    const double gate_scope_request = thrust_release_fast_rate_correction(
        true, true, true, true, gate_blend(.010), 1.12581, .494626, 12.0, 8.0);
    require(std::abs(gate_scope_request - excess_negative_damping * .5) < 1e-12,
            "gate-scope blend was not applied once to the bounded signed lead");
    require(thrust_release_fast_rate_correction(
                true, true, true, true, gate_blend(.020), -10.0, 10.0, 12.0, 8.0) == -8.0 &&
            thrust_release_fast_rate_correction(
                true, true, true, true, gate_blend(.020), 10.0, -10.0, 12.0, 8.0) == 8.0,
            "gate-scope signed correction exceeded its symmetric +/-8 Nm limit");
    const auto gate_scope_capped = apply_thrust_rate_correction(
        -18.0, thrust_release_fast_rate_correction(
                   true, true, true, true, gate_blend(.020), 10.0, -10.0, 12.0, 8.0),
        20.0);
    require(gate_scope_capped.command == -10.0 && gate_scope_capped.command >= -20.0 &&
            gate_scope_capped.applied == 8.0,
            "gate-scope correction bypassed the existing +/-20 Nm body cap");
    require(thrust_release_fast_rate_correction(
                true, effort_release_rate_correction_eligible(true, false),
                true, true, .5, -.0162426, -1.14857, 12.0, 8.0) > 0.0,
            "confirmed Effort THRUST after the first Position handoff did not activate correction");
    require(thrust_release_fast_rate_correction(
                true, true, false, true, 1.0, -0.0162426, -1.14857, 12.0, 8.0) == 0.0 &&
            thrust_release_fast_rate_correction(
                true, true, true, false, 1.0, -0.0162426, -1.14857, 12.0, 8.0) == 0.0,
            "inactive release or stale IMU activated the correction");
    const auto capped_rate_correction = apply_thrust_rate_correction(18.0, 8.0, 20.0);
    require(capped_rate_correction.command == 20.0 && capped_rate_correction.applied == 2.0,
            "supplemental torque exceeded the existing +/-20 Nm body cap");
    const auto lower_capped_rate_correction = apply_thrust_rate_correction(-19.0, 8.0, 20.0);
    require(lower_capped_rate_correction.command == -11.0 &&
            lower_capped_rate_correction.applied == 8.0,
            "positive correction failed to unwind an opposing saturated body command");
    const auto mirrored_lower_cap = apply_thrust_rate_correction(-18.0, -8.0, 20.0);
    require(mirrored_lower_cap.command == -20.0 && mirrored_lower_cap.applied == -2.0,
            "negative symmetric correction exceeded the existing -20 Nm body cap");
    require(apply_thrust_rate_correction(3.0, 0.0, 20.0).command == 3.0,
            "zero correction changed the baseline body command");

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

    // Candidate setting: keep the forward-acceleration side at 8 m/s^2 while
    // allowing only the positive/braking side to slew at 16 m/s^2. These are
    // command slopes in m/s^2, not measured wheel speed or motor limits.
    const double accel_at_16_decel =
        thrust_wheel_rate_limit(-0.80,cur_cmd,0.005,8.0,16.0);
    const double brake_at_16_decel =
        thrust_wheel_rate_limit(0.50,cur_cmd,0.005,8.0,16.0);
    require(std::abs(accel_at_16_decel-(-.540))<1e-12,
            "increased braking slew also increased forward-acceleration slew");
    require(std::abs(brake_at_16_decel-(-.420))<1e-12 &&
            brake_at_16_decel <= .50 && brake_at_16_decel > cur_cmd,
            "16 m/s^2 braking command overshot or ignored its directional rate limit");
    require(thrust_ground_wheel_target(.85,-.65)==-1.50 &&
            thrust_ground_wheel_target(0.0,+1.50)==1.50,
            "higher command slew changed the existing +/-1.50 m/s target clamp");

    // Recorded initial-BALANCE command reversals from the failed startup trace.
    // The controller keeps each legacy +/-2.5 target but must slew the
    // Position-support wheel command by no more than 8 m/s^2.
    const double failed_startup_targets[] = {
        0.266709, 0.276966, -0.254756, 0.422701, -0.413098,
        1.18705, 1.12717, 1.44535, -0.0363338, 2.5, -2.5};
    double startup_cmd = 0.0; // last_wheel_cmd_x_ at startup
    for (double target : failed_startup_targets)
    {
        const double applied = initial_balance_wheel_command(
            target, startup_cmd, 0.005, true, 8.0);
        require(std::abs(applied-startup_cmd) <= 8.0*0.005 + 1e-12,
                "initial BALANCE slew exceeded 8 m/s^2 on recorded rate reversals");
        require((target >= startup_cmd && applied <= target) ||
                (target <= startup_cmd && applied >= target),
                "initial BALANCE slew overshot its raw target");
        require(std::abs(applied) <= 2.5,
                "rate-limited command escaped the existing +/-2.5 target bound");
        startup_cmd=applied;
    }
    require(std::abs(initial_balance_wheel_command(2.5,0.0,0.0,true,8.0)-0.008) < 1e-12 &&
            std::abs(initial_balance_wheel_command(-2.5,0.0,-.1,true,8.0)+0.008) < 1e-12,
            "zero/negative dt must use the existing 1 ms minimum step without a command jump");
    const double near_target=initial_balance_wheel_command(.300001,.3,.005,true,8.0);
    require(std::abs(near_target-.300001)<1e-12 &&
            std::abs(initial_balance_wheel_command(.299999,.3,.005,true,8.0)-.299999)<1e-12,
            "initial BALANCE slew must remain continuous near zero target error");
    require(initial_balance_wheel_command(1.7,-.2,.005,false,8.0)==1.7,
            "post-landing/noninitial BALANCE must not activate the initial Position slew helper");

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
