#pragma once

#include <array>
#include <cmath>

namespace bbot_jump {

// Odometry.pose is in the world/odom frame; Odometry.twist need not be. Use
// successive position samples for jump decisions without introducing another
// slow filter on top of the simulator's velocity filtering. This estimate is
// the mean world velocity over the most recent odometry interval.
class WorldPoseVelocity {
    std::array<double, 3> previous_position_{};
    std::array<double, 3> velocity_{};
    double previous_stamp_ = -1.0;
    double sample_stamp_ = -1.0;
    bool valid_ = false;
public:
    void reset() {
        previous_stamp_ = sample_stamp_ = -1.0;
        previous_position_ = velocity_ = {};
        valid_ = false;
    }

    bool update(double stamp, const std::array<double, 3> & world_position) {
        if (!std::isfinite(stamp) || stamp < 0.0) {
            reset();
            return false;
        }
        for (double position : world_position) {
            if (!std::isfinite(position)) {
                reset();
                return false;
            }
        }
        if (previous_stamp_ < 0.0 || stamp < previous_stamp_) {
            reset();
            previous_stamp_ = stamp;
            previous_position_ = world_position;
            return false;
        }
        // A repeated sensor frame contributes no new evidence. Keep the last
        // valid estimate and its timestamp, so callers can apply freshness.
        const double dt = stamp - previous_stamp_;
        if (dt < 0.001 - 1e-9) return false;

        std::array<double, 3> candidate{};
        bool usable = dt <= 0.080 + 1e-9;
        for (std::size_t axis = 0; axis < candidate.size(); ++axis) {
            candidate[axis] = (world_position[axis] - previous_position_[axis]) / dt;
            // Teleport/reset discontinuities are not takeoff velocity. Keep
            // this well above the 1.98 m/s jump target and expected impacts.
            usable = usable && std::isfinite(candidate[axis]) &&
                std::abs(candidate[axis]) <= 12.0;
        }
        previous_stamp_ = stamp;
        previous_position_ = world_position;
        valid_ = usable;
        if (!usable) {
            velocity_ = {};
            sample_stamp_ = -1.0;
            return false;
        }
        velocity_ = candidate;
        sample_stamp_ = stamp;
        return true;
    }

    bool valid() const { return valid_; }
    double sample_stamp() const { return sample_stamp_; }
    const std::array<double, 3> & velocity() const { return velocity_; }
};

} // namespace bbot_jump
