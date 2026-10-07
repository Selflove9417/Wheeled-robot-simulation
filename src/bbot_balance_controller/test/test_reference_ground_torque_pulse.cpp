#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "bbot_balance_controller/reference_ground_torque_pulse.hpp"

namespace
{
bbot_jump::ReferenceGroundPulseInput valid_input(double gate_elapsed = 0.060)
{
    bbot_jump::ReferenceGroundPulseInput in;
    in.enabled = true;
    in.thrust_phase = true;
    in.gate_open = true;
    in.attitude_blocked = false;
    in.effort_active = true;
    in.switch_pending = false;
    in.imu_fresh = true;
    in.joint_fresh = true;
    in.contact_semantically_valid = true;
    in.contact_continuous = true;
    in.contact_wheel_mask = 0x3;
    in.now_ns = 100'000'000;
    in.contact_stamp_ns = 95'000'000;
    in.gate_elapsed_sec = gate_elapsed;
    return in;
}
}

int main()
{
    using bbot_jump::ReferenceGroundPulseGuard;
    using bbot_jump::reference_ground_torque_pulse;

    const std::array<std::pair<double, double>, 9> expected = {{
        {0.019, 0.0}, {0.020, 0.0}, {0.030, 0.5}, {0.040, 1.0},
        {0.060, 1.0}, {0.100, 1.0}, {0.110, 0.5}, {0.120, 0.0}, {0.121, 0.0}}};
    for (const auto &[elapsed, shape] : expected)
    {
        const auto out = reference_ground_torque_pulse(valid_input(elapsed));
        assert(out.guard == (elapsed < 0.020 || elapsed > 0.120
                                 ? ReferenceGroundPulseGuard::OutsideWindow
                                 : ReferenceGroundPulseGuard::Active));
        assert(std::abs(out.shape - shape) < 1e-12);
        assert(std::abs(out.requested_offset_nm + 2.0 * shape) < 1e-12);
        assert(out.active == (shape > 0.0));
    }

    auto in = valid_input();
    auto out = reference_ground_torque_pulse(in);
    assert(out.active && std::abs(out.requested_offset_nm + 2.0) < 1e-12);
    in.enabled = false;
    out = reference_ground_torque_pulse(in);
    assert(!out.active && out.guard == ReferenceGroundPulseGuard::Disabled &&
           out.requested_offset_nm == 0.0);
    in = valid_input(); in.thrust_phase = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::WrongPhase);
    in = valid_input(); in.gate_open = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::GateClosed);
    in = valid_input(); in.attitude_blocked = true;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::AttitudeBlocked);
    in = valid_input(); in.effort_active = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::EffortInactive);
    in = valid_input(); in.switch_pending = true;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::SwitchPending);
    in = valid_input(); in.imu_fresh = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ImuStale);
    in = valid_input(); in.joint_fresh = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::JointStale);
    in = valid_input(); in.contact_semantically_valid = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ContactInvalid);
    in = valid_input(); in.contact_continuous = false;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ContactInvalid);
    in = valid_input(); in.contact_wheel_mask = 0x1;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ContactNotBilateral);
    in = valid_input(); in.now_ns = in.contact_stamp_ns + 10'000'001;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ContactStale);
    in = valid_input(); in.now_ns = in.contact_stamp_ns - 1'000'001;
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::ContactStale);
    in = valid_input(); in.gate_elapsed_sec = std::numeric_limits<double>::quiet_NaN();
    assert(reference_ground_torque_pulse(in).guard == ReferenceGroundPulseGuard::InvalidInput);

    // The logged applied value records the command difference after the final clip.
    const double baseline = std::clamp(-2.5, -3.0, 3.0);
    const double with_pulse = std::clamp(-2.5 - 2.0, -3.0, 3.0);
    assert(std::abs(bbot_jump::reference_ground_pulse_applied_delta(
                        baseline, with_pulse) + 0.5) < 1e-12);
    assert(bbot_jump::reference_ground_pulse_applied_delta(
               baseline, baseline) == 0.0);

    std::cout << "reference ground torque pulse guard/timing tests passed\n";
    return 0;
}
