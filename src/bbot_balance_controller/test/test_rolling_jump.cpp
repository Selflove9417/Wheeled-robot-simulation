#include <cmath>
#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/rolling_jump_control.hpp"

void require(bool condition, const char * message) {
    if (!condition) throw std::runtime_error(message);
}
int main() {
    using namespace bbot_jump;
    // The reported failure is still accelerating, just outside the speed gate.
    require(!rolling_prepare_ready(.128,.20,.20,.070-.074,-.011,.06,.035,.30),
            "must not start thrust at the logged below-threshold speed");
    require(kDefaultPreJumpTimeout>2.08,"default deadline leaves no settling budget");
    require(rolling_prepare_ready(.15,.20,.20,.004,.01,.06,.035,.30),"valid rolling state rejected");
    require(!rolling_prepare_ready(.15,.20,.17,.004,.01,.06,.035,.30),"ramp must finish first");
    require(!rolling_prepare_ready(.20,.20,.20,.08,.01,.06,.035,.30),"unsafe pitch accepted");
    require(!rolling_prepare_ready(.20,.20,.20,.004,.5,.06,.035,.30),"unsafe rate accepted");
    // PI-generated pitch reference .074 must survive PRE_JUMP -> SQUAT.
    require(std::abs(rolling_reference_blend(.074,.098,0)-.074)<1e-12,"pitch reference jumped");
    require(std::abs(rolling_reference_blend(.074,.098,1)-.098)<1e-12,"wrong squat terminal pitch");
    require(std::abs(rolling_reference_blend(.043,.037,0)-.043)<1e-12,"control scale jumped");
    for (double initial : {.038,.074,.112}) {
        for (int i=0;i<=100;++i) {
            const double value=rolling_reference_blend(initial,.098,i/100.0);
            require(value>=std::min(initial,.098)-1e-12 && value<=std::max(initial,.098)+1e-12,
                    "reference overshoot");
        }
    }
    // Logged abort command step -.135 -> -.568 must take several control ticks.
    double command=-.135;
    command=rolling_abort_wheel_command(-.568,command,.005);
    require(std::abs(command+.160)<1e-12,"abort command contains a step");
    for (int i=0;i<30;++i) {
        const double next=rolling_abort_wheel_command(-.568,command,.005);
        require(std::abs(next-command)<=.025+1e-12,"abort slew bound violated");
        command=next;
    }
    require(std::abs(command+.568)<1e-12,"abort cannot converge to balance output");
    require(rolling_abort_wheel_command(1,-.2,0)==-.2,"paused clock must hold command");

    // Flight wheel target tests:
    // 1. Baseline wheel speed preserved when within deadband
    require(std::abs(flight_wheel_target(0.40, 0.0, 0.0, 0.0, 0.0, false, -1.0) - 0.40) < 1e-12,
            "baseline wheel speed must be preserved");
    require(std::abs(flight_wheel_target(0.40, -0.015, -0.05, 0.0, 0.0, false, -1.0) - 0.40) < 1e-12,
            "within-deadband pitch error/rate must not perturb baseline");

    // 2. Reaction wheel direction: air_wheel_sign=-1.0 must accelerate backward (cmd > baseline) for negative pitch/rate
    const double cmd_neg_pitch = flight_wheel_target(0.0, -0.10, -0.50, 0.0, 0.0, false, -1.0);
    require(cmd_neg_pitch > 0.0,
            "negative pitch/rate with air_wheel_sign=-1.0 must produce positive wheel velocity cmd");
    const double cmd_pos_pitch = flight_wheel_target(0.0, 0.10, 0.50, 0.0, 0.0, false, -1.0);
    require(cmd_pos_pitch < 0.0,
            "positive pitch/rate with air_wheel_sign=-1.0 must produce negative wheel velocity cmd");

    // 3. ATTITUDE_ARREST knee braking feedforward sign mapping
    double arrest_ff_neg = 0.0;
    double cmd_arrest_neg = flight_wheel_target(0.0, 0.0, 0.0, 10.0, 0.0, true, -1.0, 0.50, 0.45, 1.40, &arrest_ff_neg);
    require(std::abs(arrest_ff_neg - 0.35) < 1e-12, "arrest_ff must be +0.35 for air_wheel_sign=-1.0");
    require(std::abs(cmd_arrest_neg - 0.35) < 1e-12, "cmd must equal arrest_ff when error is zero");

    double arrest_ff_pos = 0.0;
    double cmd_arrest_pos = flight_wheel_target(0.0, 0.0, 0.0, 10.0, 0.0, true, +1.0, 0.50, 0.45, 1.40, &arrest_ff_pos);
    require(std::abs(arrest_ff_pos - (-0.35)) < 1e-12, "arrest_ff must be -0.35 for air_wheel_sign=+1.0 (legacy)");
    require(std::abs(cmd_arrest_pos - (-0.35)) < 1e-12, "cmd must equal -0.35 under legacy sign");

    // 4. Feedforward and feedback cooperate (both positive, reinforcing)
    const double cmd_both = flight_wheel_target(0.0, 0.0, -0.50, 10.0, 0.0, true, -1.0);
    const double cmd_fb_only = flight_wheel_target(0.0, 0.0, -0.50, 0.0, 0.0, false, -1.0);
    require(cmd_both > cmd_fb_only + 0.30, "feedforward and feedback must reinforce each other");

    // 5. Clamping at speed limits
    const double cmd_sat_pos = flight_wheel_target(0.0, -2.0, -10.0, 0.0, 0.0, false, -1.0, 0.50, 0.45, 1.40);
    require(std::abs(cmd_sat_pos - 1.40) < 1e-12, "must clamp to positive speed limit");
    const double cmd_sat_neg = flight_wheel_target(0.0, 2.0, 10.0, 0.0, 0.0, false, -1.0, 0.50, 0.45, 1.40);
    require(std::abs(cmd_sat_neg - (-1.40)) < 1e-12, "must clamp to negative speed limit");

    std::cout << "PASS: rolling readiness, continuous squat reference, bounded abort handoff, flight wheel control\n";
}
