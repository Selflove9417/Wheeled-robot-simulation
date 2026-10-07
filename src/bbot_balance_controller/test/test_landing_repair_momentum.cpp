#include <iostream>
#include <limits>
#include <stdexcept>
#include "bbot_balance_controller/landing_repair_momentum.hpp"

using namespace bbot_jump;
void require(bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
}
int main() {
    // Independent seven-body orbital+axial calculation from a strictly paired
    // physical trial sample (IMU and joint headers both 3.320s).
    JointVector q(.887944, -1.29756, .887933, -1.29756);
    JointVector v(.873531, -4.00572, .873127, -4.00557);
    const std::array<double, 2> wheel{11.4154, 11.4154};
    const auto h = landing_momentum(q, v, wheel, -1.53574, 9.5);
    require(h.valid && std::abs(h.total - 1.2465104812424854) < 1e-10,
            "mass-matrix momentum differs from independent seven-body result");
    require(std::abs(landing_pitch_rate_for_momentum(q, v, wheel, h.total, 9.5) +
                     1.53574) < 1e-12, "inverse momentum has wrong pitch sign");
    const auto stopped = landing_momentum(q, JointVector::Zero(), wheel, 0.0, 9.5);
    require(stopped.total < h.total &&
            landing_pitch_rate_for_momentum(q, JointVector::Zero(), wheel, h.total, 9.5) < -.5,
            "stopping joints must not falsely cancel total momentum");
    require(!landing_momentum(q, v, wheel, 0.0, -1.0).valid,
            "invalid mass accepted");
    auto invalid_q = q; invalid_q[0] = std::numeric_limits<double>::quiet_NaN();
    require(!landing_momentum(invalid_q, v, wheel, 0.0, 9.5).valid,
            "nonfinite state accepted");

    LandingJointHistory history;
    LandingJointSample sample{3.32, q, v, wheel}, out;
    require(history.push(sample), "valid history sample rejected");
    require(history.exact(3.32, 3.322, out), "exact sensor headers rejected");
    require(!history.exact(3.319, 3.322, out), "nearest sensor stamp mistaken for exact");
    require(!history.exact(3.32, 3.341, out), "stale state accepted");
    require(!history.exact(3.32, 3.319, out), "future state accepted");
    sample.stamp = .01; history.push(sample);
    require(!history.exact(3.32, 3.322, out), "clock reset retained previous jump");
    sample.stamp = 0.0;
    require(!history.push(sample), "zero header stamp accepted");
    std::cout << "landing-repair momentum and sensor-time guards passed\n";
}
