#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "bbot_balance_controller/thrust_support_coordination.hpp"

namespace {
bbot_jump::ThrustSupportInput valid_input()
{
    bbot_jump::ThrustSupportInput in;
    in.enabled = true;
    in.thrust_phase = true;
    in.gate_open = true;
    in.effort_ready = true;
    in.extension_travel_available = true;
    in.contact_valid = true;
    in.contact_continuous = true;
    in.wheel_mask = 3;
    in.aligned_state_fresh = true;
    in.now = 10.020;
    in.contact_stamp = 10.018;
    in.imu_stamp = in.joint_stamp = in.com_stamp = 10.019;
    in.gate_elapsed = 0.020;
    in.q = {0.70, -0.90, 0.70, -0.90};
    in.nominal_qdot = {3.0, -8.0, 3.1, -7.9};
    in.pitch = 0.05;
    in.pitch_rate = 0.12;
    in.body_mass = 9.5;
    const auto geometry = bbot_jump::centroidal_geometry(in.q, in.body_mass);
    const Eigen::Vector3d vertical(0.0, -std::sin(in.pitch), std::cos(in.pitch));
    const Eigen::RowVector4d jz = vertical.transpose()*
        (geometry.com_jacobian-geometry.axle_jacobian);
    const auto relative = bbot_jump::rotate_about_hip(-in.pitch, geometry.com-geometry.axle);
    const Eigen::Map<const Eigen::Vector4d> qdot(in.nominal_qdot.data());
    in.desired_com_vz = jz.dot(qdot)-relative.y()*in.pitch_rate;
    in.target_forward = 0.010;
    in.forward_gain = 3.0;
    in.forward_rate_limit = 0.30;
    return in;
}
}

int main()
{
    using bbot_jump::ThrustSupportGuard;
    using bbot_jump::thrust_support_coordination;

    auto in = valid_input();
    const auto out = thrust_support_coordination(in);
    assert(out.active && out.guard == ThrustSupportGuard::Active);
    assert(out.blend == 1.0);
    assert(out.contact_forward < in.target_forward);
    assert(out.resulting_forward_velocity > 0.0);
    assert(std::abs(out.resulting_forward_velocity-out.desired_forward_velocity) < 1e-9);
    assert(std::abs(out.resulting_vertical_velocity-in.desired_com_vz) < 1e-9);
    for (std::size_t i=0;i<4;++i) {
        assert(std::abs(out.qdot[i]) <= (i%2 ? in.knee_speed_limit : in.hip_speed_limit));
        assert(std::abs(in.q[i]+.030*out.qdot[i]) <= (i%2 ? 1.56 : 1.52));
    }

    // Smooth activation starts with no reference step and moves monotonically.
    auto onset = valid_input(); onset.gate_elapsed = 0.0;
    const auto at_start = thrust_support_coordination(onset);
    assert(at_start.active && at_start.blend == 0.0);
    for (std::size_t i=0;i<4;++i) assert(at_start.qdot[i] == onset.nominal_qdot[i]);
    onset.gate_elapsed = 0.010;
    const auto halfway = thrust_support_coordination(onset);
    assert(halfway.active && halfway.blend > 0.0 && halfway.blend < 1.0);
    assert(std::abs(halfway.resulting_forward_velocity-
                    (halfway.nominal_forward_velocity + halfway.blend *
                     (halfway.bounded_forward_velocity_target-
                      halfway.nominal_forward_velocity))) < 1e-9);
    assert(std::abs(halfway.resulting_vertical_velocity-onset.desired_com_vz) < 1e-9);

    auto rejected = valid_input();
    rejected.enabled = false;
    auto fallback = thrust_support_coordination(rejected);
    assert(!fallback.active && fallback.guard == ThrustSupportGuard::Disabled);
    assert(fallback.qdot == rejected.nominal_qdot);
    rejected = valid_input(); rejected.contact_continuous = false;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::ContactInvalid);
    rejected = valid_input(); rejected.wheel_mask = 1;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::ContactNotBilateral);
    rejected = valid_input(); rejected.contact_stamp = rejected.now + .002;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::ContactStale);
    auto boundary = valid_input();
    boundary.contact_stamp = boundary.now - 0.0100000005;
    assert(thrust_support_coordination(boundary).active);
    boundary = valid_input();
    boundary.com_stamp = boundary.imu_stamp - 0.0100000005;
    assert(thrust_support_coordination(boundary).active);
    rejected = valid_input(); rejected.imu_stamp = rejected.joint_stamp + .002;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::StateStale);
    rejected = valid_input(); rejected.switch_pending = true;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::SwitchPending);
    rejected = valid_input(); rejected.attitude_blocked = true;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::AttitudeBlocked);
    rejected = valid_input(); rejected.extension_travel_available = false;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::TravelProtected);
    rejected = valid_input(); rejected.desired_com_vz += 0.01;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::VerticalTaskMismatch);
    rejected = valid_input(); rejected.nominal_qdot[1] = std::numeric_limits<double>::quiet_NaN();
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::InvalidInput);
    rejected = valid_input(); rejected.nominal_qdot = {11.0, 15.0, 11.0, 15.0};
    rejected.q = {1.50, 1.55, 1.50, 1.55};
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::SpeedLimit);
    rejected = valid_input(); rejected.q[0] = 1.5201;
    assert(thrust_support_coordination(rejected).guard == ThrustSupportGuard::PositionLimit);

    std::cout << "THRUST support geometry coordination tests passed\n";
    return 0;
}
