#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace bbot_jump {
// Produce the CSV timestamp from the same ROS-clock sample used to decide and
// execute this control update. Do not re-read /clock while logging: a concurrent
// ROS clock callback can otherwise move the logger into a different sample.
inline double control_sample_log_timestamp(double control_stamp_sec,
                                           double start_stamp_sec) {
    if (!std::isfinite(control_stamp_sec) || !std::isfinite(start_stamp_sec))
        return -1.0;
    return control_stamp_sec - start_stamp_sec;
}

// Accumulate continuous conditions from the timestamps of samples that
// actually satisfied them; never multiply a count by the latest control dt.
class StableDurationNs {
public:
    void reset() { start_ns_ = -1; last_ns_ = -1; }

    int64_t update(bool condition, int64_t stamp_ns) {
        if (!condition || stamp_ns < 0) {
            reset();
            return 0;
        }
        if (last_ns_ >= 0 && stamp_ns <= last_ns_) {
            if (stamp_ns == last_ns_) return elapsed_ns();
            // A clock rollback starts a new interval; it cannot bridge epochs.
            start_ns_ = stamp_ns;
            last_ns_ = stamp_ns;
            return 0;
        }
        if (start_ns_ < 0) start_ns_ = stamp_ns;
        last_ns_ = stamp_ns;
        return elapsed_ns();
    }

    int64_t elapsed_ns() const {
        return start_ns_ >= 0 && last_ns_ >= start_ns_ ? last_ns_ - start_ns_ : 0;
    }

private:
    int64_t start_ns_{-1};
    int64_t last_ns_{-1};
};

// Wall timer callbacks may greatly outnumber simulation steps. Only advance
// control state after a real 5 ms clock step; never manufacture elapsed time.
inline bool advance_control_time(double now, double & previous, double & dt) {
    dt = 0.0;
    if (!std::isfinite(now)) return false;
    if (!std::isfinite(previous) || now < previous) { previous=now; return false; }
    const double elapsed=now-previous;
    if (elapsed < 0.005-1e-9) return false;
    previous=now;
    dt=std::min(elapsed,0.050);
    return true;
}
}
