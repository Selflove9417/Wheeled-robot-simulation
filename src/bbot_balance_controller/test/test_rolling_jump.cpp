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
    std::cout << "PASS: rolling readiness, continuous squat reference, bounded abort handoff\n";
}
