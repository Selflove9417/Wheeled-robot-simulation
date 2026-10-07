#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace bbot_jump {

struct ThrustForwardSpeedPrediction {
    double raw_speed = 0.0;
    double base_speed = 0.0;
    double predicted_speed = 0.0;
    double acceleration = 0.0;
    double horizon = 0.0;
    double delta_speed = 0.0;
    bool active = false;
};

// Predict the THRUST wheel-feedback speed from three distinct, time-aligned
// world COM positions. The velocity between adjacent positions is treated as
// a midpoint measurement; extrapolation is tightly bounded and never changes
// the observer's raw velocity or any takeoff/flight measurement.
class ThrustForwardSpeedPredictor {
public:
    static constexpr double kMaxInterval = 0.080;
    static constexpr double kMaxFreshAge = 0.040;
    static constexpr double kMaxHorizon = 0.040;
    static constexpr double kMaxAcceleration = 4.0;
    static constexpr double kMaxDeltaSpeed = 0.080;

    void reset() {
        samples_.clear();
        axis_ = {};
        have_axis_ = false;
    }

    // `forward_axis` is the unit horizontal axis frozen when /jump_cmd was
    // accepted. A repeated identical position is ignored; conflicting data
    // at one timestamp clears history instead of creating fake acceleration.
    bool observe(double stamp, const std::array<double, 3> & world_com,
                 const std::array<double, 2> & forward_axis) {
        if (!std::isfinite(stamp) || stamp < 0.0 ||
            !std::isfinite(world_com[0]) || !std::isfinite(world_com[1]) ||
            !std::isfinite(world_com[2]) ||
            !std::isfinite(forward_axis[0]) || !std::isfinite(forward_axis[1])) {
            reset();
            return false;
        }
        const double axis_norm = std::hypot(forward_axis[0], forward_axis[1]);
        if (!std::isfinite(axis_norm) || std::abs(axis_norm - 1.0) > 1e-3) {
            reset();
            return false;
        }
        if (have_axis_ &&
            (std::abs(axis_[0] - forward_axis[0]) > 1e-6 ||
             std::abs(axis_[1] - forward_axis[1]) > 1e-6)) {
            reset();
            return false;
        }
        if (!have_axis_) {
            axis_ = forward_axis;
            have_axis_ = true;
        }

        const double along = world_com[0] * axis_[0] + world_com[1] * axis_[1];
        if (!std::isfinite(along)) {
            reset();
            return false;
        }
        if (!samples_.empty()) {
            const double dt = stamp - samples_.back().stamp;
            if (dt == 0.0) {
                if (std::abs(world_com[0] - samples_.back().world[0]) <= 1e-12 &&
                    std::abs(world_com[1] - samples_.back().world[1]) <= 1e-12 &&
                    std::abs(world_com[2] - samples_.back().world[2]) <= 1e-12)
                    return false;
                reset();
                return false;
            }
            if (dt < 0.0 || dt < 0.001 || dt > kMaxInterval) {
                reset();
                // A discontinuity is not a usable interval, but this valid
                // current point can seed the next two distinct observations.
                if (dt > 0.0 && dt > kMaxInterval) {
                    axis_ = forward_axis;
                    have_axis_ = true;
                    samples_.push_back({stamp, along, world_com});
                }
                return false;
            }
        }
        samples_.push_back({stamp, along, world_com});
        while (samples_.size() > 3) samples_.pop_front();
        return true;
    }

    ThrustForwardSpeedPrediction predict(double now, double raw_speed,
                                         bool source_fresh, bool enabled) {
        ThrustForwardSpeedPrediction out;
        out.raw_speed = raw_speed;
        out.base_speed = raw_speed;
        out.predicted_speed = raw_speed;
        if (!enabled) return out;
        if (!source_fresh || !std::isfinite(now) || !std::isfinite(raw_speed)) {
            reset();
            return out;
        }
        if (samples_.empty()) return out;
        const double newest_age = now - samples_.back().stamp;
        if (!std::isfinite(newest_age) || newest_age < 0.0 ||
            newest_age > kMaxFreshAge) {
            reset();
            return out;
        }
        if (samples_.size() != 3) return out;

        const auto & a = samples_[0];
        const auto & b = samples_[1];
        const auto & c = samples_[2];
        const double dt01 = b.stamp - a.stamp;
        const double dt12 = c.stamp - b.stamp;
        if (!(dt01 >= 0.001 && dt01 <= kMaxInterval &&
              dt12 >= 0.001 && dt12 <= kMaxInterval)) {
            return out;
        }

        const double v01 = (b.along - a.along) / dt01;
        const double v12 = (c.along - b.along) / dt12;
        const double midpoint01 = 0.5 * (a.stamp + b.stamp);
        const double midpoint12 = 0.5 * (b.stamp + c.stamp);
        const double midpoint_dt = midpoint12 - midpoint01;
        if (!std::isfinite(v01) || !std::isfinite(v12) ||
            !std::isfinite(midpoint_dt) || midpoint_dt <= 0.0 ||
            std::abs(v01) > 12.0 || std::abs(v12) > 12.0) {
            reset();
            return out;
        }

        out.horizon = now - midpoint12;
        if (!std::isfinite(out.horizon) || out.horizon < 0.0 ||
            out.horizon > kMaxHorizon + 1e-9) {
            reset();
            out.horizon = 0.0;
            return out;
        }
        out.horizon = std::min(out.horizon, kMaxHorizon);
        // The measured COM interval velocity and this predictor use the
        // accepted-J frozen axis, even if the body has since yawed slightly.
        out.base_speed = v12;
        out.predicted_speed = v12;
        out.acceleration = std::clamp((v12 - v01) / midpoint_dt,
                                      -kMaxAcceleration, kMaxAcceleration);
        // Only lead into positive acceleration. A negative estimate can be a
        // transient consequence of interval timing; using it to lower the
        // speed feedback input would increase forward drive. Keep the signed
        // acceleration diagnostic, but never apply a negative lead.
        out.delta_speed = std::clamp(
            std::max(0.0, out.acceleration * out.horizon), 0.0, kMaxDeltaSpeed);
        out.predicted_speed = out.base_speed + out.delta_speed;
        if (!std::isfinite(out.predicted_speed)) {
            out = {};
            out.raw_speed = raw_speed;
            out.base_speed = raw_speed;
            out.predicted_speed = raw_speed;
            return out;
        }
        out.active = true;
        return out;
    }

    std::size_t sample_count() const { return samples_.size(); }

private:
    struct Sample {
        double stamp;
        double along;
        std::array<double, 3> world;
    };
    std::deque<Sample> samples_;
    std::array<double, 2> axis_{};
    bool have_axis_ = false;
};

}  // namespace bbot_jump
