#include "bbot_balance_controller/world_pose_velocity.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
}

int main() {
    using bbot_jump::WorldPoseVelocity;
    WorldPoseVelocity estimate;
    check(!estimate.update(1.0, {0, 0, .50}) && !estimate.valid(),
          "single pose claimed velocity");
    check(estimate.update(1.02, {0, .01, .54}), "regular odometry rejected");
    check(std::abs(estimate.velocity()[1] - .50) < 1e-10 &&
          std::abs(estimate.velocity()[2] - 2.0) < 1e-10, "world velocity wrong");
    check(!estimate.update(1.02, {0, 3, 5}) && estimate.valid() &&
          std::abs(estimate.velocity()[2] - 2.0) < 1e-10,
          "duplicate pose altered velocity or baseline");
    check(estimate.update(1.04, {0, .02, .58}) &&
          std::abs(estimate.velocity()[2] - 2.0) < 1e-10,
          "duplicate pose corrupted next interval");

    // Recorded apex: legacy filtered twist still said +0.472 m/s, while the
    // world height had already fallen. Event decisions must see descent now.
    estimate.reset();
    estimate.update(8.04, {0, 0, .752786});
    check(estimate.update(8.06, {0, .010, .743255}) &&
          estimate.velocity()[2] < -.47, "apex descent delayed by old twist");

    // Heading/lean cannot project horizontal speed into world vertical speed.
    estimate.reset();
    estimate.update(2.0, {0, 0, .30});
    check(estimate.update(2.02, {.02, .03, .30}) && estimate.velocity()[2] == 0,
          "horizontal motion became vertical motion");

    check(!estimate.update(2.021, {5, 5, 5}) && !estimate.valid(),
          "teleport became takeoff velocity");
    check(estimate.update(2.041, {5, 5, 5}) && estimate.velocity()[2] == 0,
          "teleport rejection prevented recovery");
    check(!estimate.update(2.20, {5, 5, 5}) && !estimate.valid(),
          "long odometry gap accepted");
    check(estimate.update(2.22, {5, 5, 5}), "gap rebase did not recover");
    check(!estimate.update(.10, {0, 0, .4}) && !estimate.valid(),
          "clock reset retained stale velocity");
    check(estimate.update(.12, {0, 0, .42}), "clock reset failed to recover");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    check(!estimate.update(.14, {0, nan, .4}) && !estimate.valid(),
          "non-finite position accepted");
    check(!estimate.update(nan, {0, 0, .4}), "non-finite timestamp accepted");
    check(!estimate.update(-1, {0, 0, .4}), "negative timestamp accepted");
    estimate.update(3.0, {0, 0, .4});
    check(!estimate.update(3.0001, {0, 0, .4001}), "tiny interval accepted");
    check(estimate.update(3.001, {0, 0, .401}) &&
          std::abs(estimate.velocity()[2] - 1.0) < 1e-9,
          "tiny interval corrupted baseline");
    std::cout << "World pose velocity timing, frame, apex and reset regressions passed\n";
}
