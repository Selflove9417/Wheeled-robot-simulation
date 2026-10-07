#include <iostream>
#include <stdexcept>
#include <cmath>
#include <array>
#include "bbot_balance_controller/jump_phase_control.hpp"
#include "bbot_balance_controller/takeoff_detection.hpp"
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/thrust_velocity_reference.hpp"

void check(bool condition, const char * msg) {
    if (!condition) {
        std::cerr << "FAILED: " << msg << std::endl;
        throw std::runtime_error(msg);
    }
}

int main() {
    using namespace bbot_jump;
    std::cout << "Running Comprehensive Jump Regression Suite..." << std::endl;

    // 1. Takeoff detection: dual sample, hysteresis, stale timestamps, immediate trigger
    {
        TakeoffConfirmation confirm;
        // First frame: clearance > 12mm
        check(!confirm.update(1.00, 1.005, true, 0.015, 0.015, true, true),
              "First sample alone must not trigger takeoff");
        check(confirm.count() == 1, "First sample should set count=1");

        // Second frame < 10ms: reject
        check(!confirm.update(1.005, 1.008, true, 0.015, 0.015, true, true),
              "Frame under 10ms interval must not trigger takeoff");

        // Second frame >= 10ms, clearance > 6mm (hysteresis threshold): success
        check(confirm.update(1.015, 1.018, true, 0.008, 0.008, true, true),
              "Second sample with hysteresis > 6mm must confirm takeoff");
        check(confirm.count() >= 2, "Confirmed takeoff must have count >= 2");

        // Stale timestamp rejection (> 80ms)
        confirm.reset();
        check(!confirm.update(1.00, 1.10, true, 0.020, 0.020, true, true),
              "Stale sample must be rejected");
        check(confirm.count() == 0, "Stale sample count must be 0");
    }

    // 2. COM velocity latching & ThrustRelease monotonic decay
    {
        TakeoffSpeedLatch speed_latch;
        check(!speed_latch.update(1.00, 1.10, true, 2.00, 1.98),
              "Stale COM velocity must not latch launch speed");
        check(!speed_latch.update(1.00, 1.01, false, 2.00, 1.98),
              "Speed reached before thrust is armed must not latch");
        check(!speed_latch.update(1.00, 1.01, true, 1.87, 1.98),
              "Velocity below 95 percent target must not latch");
        check(speed_latch.update(1.02, 1.025, true, 1.89, 1.98),
              "First fresh velocity crossing must latch independently of clearance");
        check(std::abs(speed_latch.velocity()-1.89)<1e-12,
              "Latched velocity must preserve the first crossing sample");
        check(speed_latch.update(1.03, 1.035, true, 1.50, 1.98),
              "A latched speed event must survive later velocity decay");
        check(std::abs(speed_latch.velocity()-1.89)<1e-12,
              "Later samples must not overwrite first crossing velocity");

        ThrustRelease release;
        check(!release.active(), "ThrustRelease initially inactive");

        // Pre-threshold update: should not trigger
        release.update(1.00, 0.70, 1.37, 1.98, 120.0);
        check(!release.active(), "Low phase / low velocity must not trigger release");

        // Unloading leads the target because the synchronized COM estimate
        // and the finite force ramp otherwise add one more full impulse.
        release.update(1.05, 0.20, 1.39, 1.98, 150.0);
        check(release.active(), "Predictive unload must lead the ballistic target");
        check(std::abs(release.force_limit(1.05) - 150.0) < 1e-6, "Initial release force preserved");

        // Monotonic unloading: subsequent speed drop must not increase force limit
        double prev_limit = release.force_limit(1.05);
        for (int i = 1; i <= 50; ++i) {
            double t = 1.05 + 0.001 * i;
            // Velocity drops, but force limit must monotonically decrease towards 0
            release.update(t, 0.95, 1.50, 1.98, 200.0);
            double cur_limit = release.force_limit(t);
            check(cur_limit <= prev_limit + 1e-12, "Force limit must monotonically unload");
            check(cur_limit >= -1e-12, "Force limit must stay non-negative");
            prev_limit = cur_limit;
        }
        check(prev_limit == 0.0, "Force limit must fully reach 0 at end of blend");
    }

    // 3. Flight round-trip trajectory: admissibility & time budget
    {
        std::array<double, 4> q{0.4, -0.6, 0.4, -0.6};
        std::array<double, 4> v{0.5, -1.0, 0.5, -1.0};
        std::array<double, 4> a{0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> mid{0.9, -1.1, 0.9, -1.1}; // tuck
        std::array<double, 4> end{0.3, -0.5, 0.3, -0.5}; // landing deploy

        // With plenty of time available (0.35s), plan must be valid
        auto plan_valid = plan_flight_round_trip(q, v, a, mid, end, 0.10, 0.12, 0.35);
        check(plan_valid.valid, "Valid round-trip flight trajectory must be admitted");
        check(plan_valid.tuck_duration >= 0.10, "Tuck duration must meet minimum nominal");
        check(plan_valid.extend_duration >= 0.12, "Extend duration must meet minimum nominal");

        // With insufficient time (e.g. 0.15s for 0.10+0.12=0.22s need), plan must fail
        auto plan_tight = plan_flight_round_trip(q, v, a, mid, end, 0.10, 0.12, 0.15);
        check(!plan_tight.valid, "Insufficient flight time budget must reject round trip");

        // Excessive incoming velocity must reject
        std::array<double, 4> v_excess{15.0, -25.0, 15.0, -25.0};
        auto plan_excess = plan_flight_round_trip(q, v_excess, a, mid, end, 0.10, 0.12, 0.40);
        check(!plan_excess.valid, "Excessive incoming velocity must reject round trip plan");
    }

    // 4. Catch wheel target direction test: forward & backward mirroring
    {
        // Positive com_velocity (moving forward) must produce negative (forward-driving) or deceleration command
        // Note: Signed wheel command convention in bbot_jump: negative command rolls the robot forward.
        // To decelerate a positive com_velocity (moving forward), wheel command must accelerate axle forward or apply back-torque.
        // Let us test centroidal_catch_target symmetry:
        // centroidal_catch_target(v, forward, height, shank_rate, R, limit, v_ref)
        const double v_fwd = 0.30;
        const double v_bwd = -0.30;
        const double r = 0.02;
        const double h = 0.34;
        const double cmd_fwd = centroidal_catch_target(v_fwd, r, h, 0.0, 0.07, 5.0, 0.0);
        const double cmd_bwd = centroidal_catch_target(v_bwd, -r, h, 0.0, 0.07, 5.0, 0.0);
        check(std::abs(cmd_fwd + cmd_bwd) < 1e-10, "Forward and backward catch targets must mirror identically");
        check(cmd_fwd != 0.0, "Catch target must be active when moving");
    }

    // 5. Touchdown capture settled criteria
    {
        // Settled state: small pitch error, small rate, small capture
        check(touchdown_capture_settled(0.02, 0.05, 0.03), "Quiet attitude must pass settled criteria");
        // Diverging forward fall
        check(!touchdown_capture_settled(0.25, 1.00, 0.40), "Diverging attitude must reject settled criteria");
        // Diverging backward fall
        check(!touchdown_capture_settled(-0.25, -1.00, -0.40), "Backward divergence must reject settled criteria");
    }

    // 6. Centroidal hold ready criteria
    {
        check(centroidal_hold_ready(true, 0.01, 0.05, 0.03), "Stable COM must satisfy centroidal hold");
        check(centroidal_hold_ready(true, -0.0381, 0.001, 0.001),
              "Measured flat-ground standing equilibrium must satisfy centroidal hold");
        check(!centroidal_hold_ready(false, 0.01, 0.05, 0.03), "Invalid state must reject centroidal hold");
        check(!centroidal_hold_ready(true, 0.08, 0.05, 0.03), "Large lean must reject centroidal hold");
        check(!centroidal_hold_ready(true, 0.01, 0.05, 0.20), "High translation velocity must reject centroidal hold");
    }


    return 0;
}
