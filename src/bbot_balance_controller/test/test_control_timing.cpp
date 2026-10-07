#include <cmath>
#include <cstdlib>
#include <iostream>

#include "bbot_balance_controller/control_timing.hpp"

namespace {
void require(bool condition, const char * message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
}

int main() {
    // A CSV row must retain the control callback's accepted sample even if
    // /clock advances (or another queued clock message is processed) before
    // the logger writes the row.
    const double control_sample_sec = 24.655;
    const double start_sec = 0.012;
    const double frozen_log_stamp = bbot_jump::control_sample_log_timestamp(
        control_sample_sec, start_sec);
    const double later_clock_sec = 24.705;
    require(std::abs(frozen_log_stamp - 24.643) < 1e-12,
            "log timestamp is computed from the accepted control sample");
    require(std::abs(frozen_log_stamp - (later_clock_sec - start_sec)) > 0.049,
            "later /clock value cannot replace the frozen update timestamp");
    require(bbot_jump::control_sample_log_timestamp(NAN, start_sec) == -1.0,
            "non-finite control stamp is rejected");
    require(bbot_jump::control_sample_log_timestamp(control_sample_sec, INFINITY) == -1.0,
            "non-finite start stamp is rejected");

    // Accepted control samples are strictly later than last_time_; timestamp
    // serialization must therefore preserve their order and duplicate steps.
    const double first = bbot_jump::control_sample_log_timestamp(10.100, 10.000);
    const double second = bbot_jump::control_sample_log_timestamp(10.105, 10.000);
    require(second > first, "separate accepted control samples remain ordered");
    std::cout << "control timing/log timestamp tests passed\n";
    return 0;
}
