#pragma once
#include <algorithm>
#include <cmath>

namespace bbot_jump {
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
