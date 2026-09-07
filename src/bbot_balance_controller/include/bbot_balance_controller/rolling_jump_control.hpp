#pragma once
#include <algorithm>
#include <cmath>

namespace bbot_jump
{
// 本地日志显示2 s时仍在正常加速；保留严格准入条件，给准备过程4 s预算。
constexpr double kDefaultPreJumpTimeout = 4.0;

inline bool rolling_prepare_ready(double velocity, double target, double ramped_target,
                                  double pitch_error, double pitch_rate,
                                  double velocity_tolerance, double pitch_tolerance,
                                  double rate_limit)
{
    return std::abs(ramped_target - target) <= 0.001 &&
        std::abs(velocity - target) <= velocity_tolerance &&
        std::abs(pitch_error) <= pitch_tolerance && std::abs(pitch_rate) <= rate_limit;
}

inline double rolling_reference_blend(double initial, double final, double progress)
{
    const double s = std::clamp(progress, 0.0, 1.0);
    return initial + (final - initial) * s * s * (3.0 - 2.0 * s);
}

inline double rolling_abort_wheel_command(double target, double previous, double dt)
{
    const double bounded = std::clamp(target, -1.20, 1.20);
    const double step = 5.0 * std::clamp(dt, 0.0, 0.05);
    return previous + std::clamp(bounded - previous, -step, step);
}
} // namespace bbot_jump
