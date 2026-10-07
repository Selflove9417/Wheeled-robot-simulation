#pragma once

#include <array>
#include <algorithm>
#include <cmath>

namespace bbot_jump {

// These are opt-in reference constraints, not historical THRUST acceleration
// limits or a prediction of physical joint acceleration/contact dynamics.
struct LandingHandoffSample {
    bool valid = false;
    bool limited = false;
    std::array<double, 4> velocity{};
    std::array<double, 4> rate{};
    std::array<double, 2> hip_feedback{};
};

inline bool handoff_support_allowed(bool contact_valid, bool contact_continuous,
                                    unsigned wheel_mask, double now,
                                    double contact_stamp) {
    return contact_valid && contact_continuous && wheel_mask == 3 &&
        std::isfinite(now) && std::isfinite(contact_stamp) && contact_stamp > 0 &&
        now >= contact_stamp && now - contact_stamp <= .010;
}

inline bool handoff_actual_travel_safe(const std::array<double,4> & q,
                                       const std::array<double,4> & velocity) {
    for (size_t i=0;i<4;++i)
        if (!std::isfinite(q[i]) || !std::isfinite(velocity[i]) ||
            std::abs(q[i]) > (i%2 ? 1.56 : 1.52) ||
            std::abs(q[i]+.030*velocity[i]) > (i%2 ? 1.56 : 1.52)) return false;
    return true; // Constant-velocity preflight guard, not a braking guarantee.
}

class LandingReferenceHandoff {
public:
    void reset() { seeded_ = false; sample_ = {}; stamp_ = 0.; }
    const LandingHandoffSample & sample() const { return sample_; }
    double stamp() const { return stamp_; }

    LandingHandoffSample update(double now, const std::array<double, 4> & target,
                                const std::array<double, 4> & measured_q,
                                const std::array<double, 2> & feedback_target,
                                bool data_fresh, bool effort_ready,
                                bool travel_ok, bool inverse_active, double knee_speed_limit = 15.) {
        LandingHandoffSample out;
        if (!data_fresh || !effort_ready || !travel_ok || (!std::isfinite(knee_speed_limit) || knee_speed_limit <= 0.) || !std::isfinite(now)) {
            reset(); return out;
        }
        for (size_t i = 0; i < 4; ++i)
            if (!std::isfinite(target[i]) || !std::isfinite(measured_q[i]) ||
                std::abs(target[i]) > (i%2 ? knee_speed_limit : 11.) ||
                std::abs(measured_q[i]) > (i%2 ? 1.56 : 1.52)) {
                reset(); return out;
            }
        for (double f : feedback_target)
            if (!std::isfinite(f)) { reset(); return out; }
        const double dt = seeded_ ? now - stamp_ : 0.;
        if (seeded_ && dt == 0.) {
            // THRUST and FLIGHT can share one control tick, but the latter
            // has a stricter knee speed budget. Recheck without advancing.
            for (size_t i=0;i<4;++i)
                if (std::abs(sample_.velocity[i]) > (i%2 ? knee_speed_limit : 11.) ||
                    std::abs(measured_q[i]+.030*sample_.velocity[i]) > (i%2 ? 1.56 : 1.52)) {
                    reset(); return out;
                }
            return sample_;
        }
        if (seeded_ && (dt < 0. || dt > .020)) { reset(); return out; }
        const double alpha = seeded_ ? -std::expm1(-dt/.020) : 0.;
        for (size_t i = 0; i < 4; ++i) {
            const double previous = seeded_ ? sample_.velocity[i] : target[i];
            const double max_step = (i%2 ? 500. : 450.) * dt;
            const double change = seeded_ ? std::clamp(
                alpha*(target[i]-previous), -max_step, max_step) : 0.;
            out.velocity[i] = previous + change;
            out.rate[i] = seeded_ ? change/dt : 0.;
            out.limited |= std::abs(out.velocity[i]-target[i]) > 1e-8;
            // Reject instead of silently overriding a hard travel guard.
            if (std::abs(out.velocity[i]) > (i%2 ? knee_speed_limit : 11.) ||
                std::abs(measured_q[i]+.030*out.velocity[i]) > (i%2 ? 1.56 : 1.52)) {
                reset(); return LandingHandoffSample{};
            }
        }
        for (size_t i = 0; i < 2; ++i) {
            const double desired = inverse_active ? std::clamp(feedback_target[i], -4., 4.) : 0.;
            const double previous = seeded_ ? sample_.hip_feedback[i] : 0.;
            out.hip_feedback[i] = previous + alpha*(desired-previous);
        }
        out.valid = true;
        seeded_ = true; stamp_ = now; sample_ = out;
        return out;
    }
private:
    bool seeded_ = false;
    double stamp_ = 0.;
    LandingHandoffSample sample_{};
};

} // namespace bbot_jump
