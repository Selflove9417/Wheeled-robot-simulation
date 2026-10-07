#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace bbot_jump
{

enum class ReferenceGroundPulseGuard : int
{
    Active = 0,
    Disabled,
    WrongPhase,
    GateClosed,
    AttitudeBlocked,
    EffortInactive,
    SwitchPending,
    ImuStale,
    JointStale,
    ContactInvalid,
    ContactNotBilateral,
    ContactStale,
    OutsideWindow,
    InvalidInput
};

struct ReferenceGroundPulseInput
{
    bool enabled{false};
    bool thrust_phase{false};
    bool gate_open{false};
    bool attitude_blocked{true};
    bool effort_active{false};
    bool switch_pending{true};
    bool imu_fresh{false};
    bool joint_fresh{false};
    bool contact_semantically_valid{false};
    bool contact_continuous{false};
    uint8_t contact_wheel_mask{0};
    int64_t now_ns{0};
    int64_t contact_stamp_ns{0};
    double gate_elapsed_sec{0.0};
};

struct ReferenceGroundPulseOutput
{
    bool active{false};
    ReferenceGroundPulseGuard guard{ReferenceGroundPulseGuard::InvalidInput};
    double shape{0.0};
    double requested_offset_nm{0.0};
};

inline double reference_ground_pulse_smoothstep5(double u)
{
    const double x = std::clamp(u, 0.0, 1.0);
    const double x2 = x * x;
    const double x3 = x2 * x;
    return 10.0 * x3 - 15.0 * x3 * x + 6.0 * x3 * x2;
}

// A bounded identification pulse: zero before gate+20 ms and after gate+120 ms,
// with 20 ms quintic entry/exit ramps. The contact source is deliberately
// stricter than takeoff evidence: every applying sample must be a fresh,
// continuous, semantically valid frame with both wheel-ground pairs.
inline ReferenceGroundPulseOutput reference_ground_torque_pulse(
    const ReferenceGroundPulseInput &in)
{
    ReferenceGroundPulseOutput out;
    if (!std::isfinite(in.gate_elapsed_sec) ||
        in.now_ns < 0 || in.contact_stamp_ns < 0)
    {
        out.guard = ReferenceGroundPulseGuard::InvalidInput;
        return out;
    }
    if (!in.enabled) { out.guard = ReferenceGroundPulseGuard::Disabled; return out; }
    if (!in.thrust_phase) { out.guard = ReferenceGroundPulseGuard::WrongPhase; return out; }
    if (!in.gate_open) { out.guard = ReferenceGroundPulseGuard::GateClosed; return out; }
    if (in.attitude_blocked) { out.guard = ReferenceGroundPulseGuard::AttitudeBlocked; return out; }
    if (!in.effort_active) { out.guard = ReferenceGroundPulseGuard::EffortInactive; return out; }
    if (in.switch_pending) { out.guard = ReferenceGroundPulseGuard::SwitchPending; return out; }
    if (!in.imu_fresh) { out.guard = ReferenceGroundPulseGuard::ImuStale; return out; }
    if (!in.joint_fresh) { out.guard = ReferenceGroundPulseGuard::JointStale; return out; }
    if (!in.contact_semantically_valid || !in.contact_continuous)
    {
        out.guard = ReferenceGroundPulseGuard::ContactInvalid;
        return out;
    }
    if (in.contact_wheel_mask != 0x3)
    {
        out.guard = ReferenceGroundPulseGuard::ContactNotBilateral;
        return out;
    }
    constexpr int64_t kFutureToleranceNs = 1'000'000;
    constexpr int64_t kMaxContactAgeNs = 10'000'000;
    if (in.now_ns < in.contact_stamp_ns - kFutureToleranceNs ||
        in.now_ns - in.contact_stamp_ns > kMaxContactAgeNs)
    {
        out.guard = ReferenceGroundPulseGuard::ContactStale;
        return out;
    }
    if (in.gate_elapsed_sec < 0.020 || in.gate_elapsed_sec > 0.120)
    {
        out.guard = ReferenceGroundPulseGuard::OutsideWindow;
        return out;
    }

    double shape = 1.0;
    if (in.gate_elapsed_sec < 0.040)
        shape = reference_ground_pulse_smoothstep5((in.gate_elapsed_sec - 0.020) / 0.020);
    else if (in.gate_elapsed_sec > 0.100)
        shape = 1.0 - reference_ground_pulse_smoothstep5((in.gate_elapsed_sec - 0.100) / 0.020);
    out.active = shape > 0.0;
    out.guard = ReferenceGroundPulseGuard::Active;
    out.shape = shape;
    out.requested_offset_nm = -2.0 * shape;
    return out;
}

// Records the net actuator-command change after the caller's existing final
// clamp. This is command-path telemetry, never a claim of measured torque.
inline double reference_ground_pulse_applied_delta(double baseline_command_nm,
                                                   double final_command_nm)
{
    if (!std::isfinite(baseline_command_nm) || !std::isfinite(final_command_nm))
        return 0.0;
    return final_command_nm - baseline_command_nm;
}

}  // namespace bbot_jump
