#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "bbot_balance_controller/centroidal_state.hpp"

namespace bbot_jump {

enum class ThrustSupportGuard : int {
    Disabled = 0,
    WrongPhase,
    GateClosed,
    EffortNotReady,
    SwitchPending,
    AttitudeBlocked,
    TravelProtected,
    ContactInvalid,
    ContactNotBilateral,
    ContactStale,
    StateStale,
    InvalidInput,
    VerticalTaskMismatch,
    SingularJacobian,
    SpeedLimit,
    PositionLimit,
    Active
};

struct ThrustSupportInput {
    bool enabled{false};
    bool thrust_phase{false};
    bool gate_open{false};
    bool effort_ready{false};
    bool switch_pending{false};
    bool attitude_blocked{false};
    bool extension_travel_available{false};
    bool contact_valid{false};
    bool contact_continuous{false};
    uint8_t wheel_mask{0};
    bool aligned_state_fresh{false};
    double now{0.0};
    double contact_stamp{0.0};
    double imu_stamp{0.0};
    double joint_stamp{0.0};
    double com_stamp{0.0};
    double gate_elapsed{0.0};
    std::array<double, 4> q{};
    std::array<double, 4> nominal_qdot{};
    double pitch{0.0};
    double pitch_rate{0.0};
    double body_mass{0.0};
    double desired_com_vz{0.0};
    double target_forward{0.010};
    double forward_gain{3.0};
    double forward_rate_limit{0.30};
    double hip_speed_limit{11.0};
    double knee_speed_limit{15.0};
};

struct ThrustSupportReference {
    bool active{false};
    ThrustSupportGuard guard{ThrustSupportGuard::Disabled};
    double contact_forward{0.0};
    double contact_vertical{0.0};
    double bounded_forward_velocity_target{0.0};
    double desired_forward_velocity{0.0};
    double nominal_forward_velocity{0.0};
    double nominal_vertical_velocity{0.0};
    double resulting_forward_velocity{0.0};
    double resulting_vertical_velocity{0.0};
    double blend{0.0};
    double jacobian_determinant{0.0};
    std::array<double, 4> qdot{};
    std::array<double, 4> delta_qdot{};
};

inline ThrustSupportReference thrust_support_coordination(
    const ThrustSupportInput &in)
{
    ThrustSupportReference out;
    out.qdot = in.nominal_qdot;
    if (!in.enabled) { out.guard = ThrustSupportGuard::Disabled; return out; }
    if (!in.thrust_phase) { out.guard = ThrustSupportGuard::WrongPhase; return out; }
    if (!in.gate_open) { out.guard = ThrustSupportGuard::GateClosed; return out; }
    if (!in.effort_ready) { out.guard = ThrustSupportGuard::EffortNotReady; return out; }
    if (in.switch_pending) { out.guard = ThrustSupportGuard::SwitchPending; return out; }
    if (in.attitude_blocked) { out.guard = ThrustSupportGuard::AttitudeBlocked; return out; }
    if (!in.extension_travel_available) { out.guard = ThrustSupportGuard::TravelProtected; return out; }
    if (!in.contact_valid || !in.contact_continuous) {
        out.guard = ThrustSupportGuard::ContactInvalid; return out;
    }
    if (in.wheel_mask != 0x3) {
        out.guard = ThrustSupportGuard::ContactNotBilateral; return out;
    }
    if (!std::isfinite(in.now) || !std::isfinite(in.contact_stamp) ||
        in.contact_stamp <= 0.0 || in.contact_stamp-in.now > 0.001+1.0e-9 ||
        in.now - in.contact_stamp > 0.010+1.0e-9) {
        out.guard = ThrustSupportGuard::ContactStale; return out;
    }
    if (!in.aligned_state_fresh || !std::isfinite(in.imu_stamp) ||
        !std::isfinite(in.joint_stamp) || !std::isfinite(in.com_stamp) ||
        in.imu_stamp <= 0.0 || in.joint_stamp <= 0.0 || in.com_stamp <= 0.0 ||
        in.imu_stamp-in.now > 0.001+1.0e-9 || in.joint_stamp-in.now > 0.001+1.0e-9 ||
        in.com_stamp-in.now > 0.001+1.0e-9 ||
        in.now - in.imu_stamp > 0.020+1.0e-9 || in.now - in.joint_stamp > 0.020+1.0e-9 ||
        in.now - in.com_stamp > 0.020+1.0e-9 ||
        std::abs(in.imu_stamp - in.joint_stamp) > 1.0e-9 ||
        std::abs(in.imu_stamp - in.com_stamp) > 0.010+1.0e-9 ||
        std::abs(in.joint_stamp - in.com_stamp) > 0.010+1.0e-9 ||
        std::abs(in.contact_stamp - in.imu_stamp) > 0.010+1.0e-9) {
        out.guard = ThrustSupportGuard::StateStale; return out;
    }
    const double scalars[] = {in.gate_elapsed, in.pitch, in.pitch_rate,
        in.body_mass, in.desired_com_vz, in.target_forward,
        in.forward_gain, in.forward_rate_limit, in.hip_speed_limit,
        in.knee_speed_limit};
    for (double v : scalars) {
        if (!std::isfinite(v)) { out.guard = ThrustSupportGuard::InvalidInput; return out; }
    }
    if (in.gate_elapsed < 0.0 || in.body_mass <= 0.0 ||
        in.forward_gain <= 0.0 || in.forward_rate_limit <= 0.0 ||
        in.hip_speed_limit <= 0.0 || in.knee_speed_limit <= 0.0) {
        out.guard = ThrustSupportGuard::InvalidInput; return out;
    }
    for (std::size_t i=0;i<4;++i) {
        if (!std::isfinite(in.q[i])) { out.guard = ThrustSupportGuard::InvalidInput; return out; }
        if (std::abs(in.q[i]) > (i%2 ? 1.56 : 1.52)) {
            out.guard = ThrustSupportGuard::PositionLimit; return out;
        }
    }
    for (double v : in.nominal_qdot) if (!std::isfinite(v)) {
        out.guard = ThrustSupportGuard::InvalidInput; return out;
    }
    for (std::size_t i=0;i<4;++i) {
        const double limit = i%2 ? 1.56 : 1.52;
        if (std::abs(in.nominal_qdot[i]) > (i%2 ? in.knee_speed_limit : in.hip_speed_limit) ||
            std::abs(in.q[i]+0.030*in.nominal_qdot[i]) > limit) {
            out.guard = ThrustSupportGuard::SpeedLimit; return out;
        }
    }

    const auto geometry = centroidal_geometry(in.q, in.body_mass);
    const auto relative = rotate_about_hip(-in.pitch, geometry.com-geometry.axle);
    out.contact_forward = relative.y();
    out.contact_vertical = relative.z();
    const Eigen::Vector3d forward(0.0, std::cos(in.pitch), std::sin(in.pitch));
    const Eigen::Vector3d vertical(0.0, -std::sin(in.pitch), std::cos(in.pitch));
    const auto relative_jacobian = geometry.com_jacobian-geometry.axle_jacobian;
    const Eigen::RowVector4d jf = forward.transpose()*relative_jacobian;
    const Eigen::RowVector4d jz = vertical.transpose()*relative_jacobian;

    out.nominal_forward_velocity = jf.dot(Eigen::Map<const Eigen::Vector4d>(in.nominal_qdot.data())) +
        out.contact_vertical*in.pitch_rate;
    out.nominal_vertical_velocity = jz.dot(Eigen::Map<const Eigen::Vector4d>(in.nominal_qdot.data())) -
        out.contact_forward*in.pitch_rate;
    if (std::abs(in.desired_com_vz-out.nominal_vertical_velocity) > 1.0e-6) {
        out.guard = ThrustSupportGuard::VerticalTaskMismatch;
        return out;
    }
    // Move toward a small forward support-line geometry target with a fixed
    // 3/s position-error law. A 20ms smoothstep onset avoids a phase-entry step.
    const double raw_forward = in.forward_gain*(in.target_forward-out.contact_forward);
    const double bounded_forward = std::clamp(raw_forward,
        -in.forward_rate_limit, in.forward_rate_limit);
    out.bounded_forward_velocity_target = bounded_forward;
    const double u = std::clamp(in.gate_elapsed/0.020, 0.0, 1.0);
    out.blend = u*u*(3.0-2.0*u);
    out.desired_forward_velocity = out.nominal_forward_velocity +
        out.blend*(bounded_forward-out.nominal_forward_velocity);

    // Two equality tasks in four joint rates. Find the minimum-norm correction
    // to the existing reference, preserving its actual vertical COM velocity
    // while changing only relative forward COM motion.
    const double g00 = jf.squaredNorm();
    const double g01 = jf.dot(jz);
    const double g11 = jz.squaredNorm();
    const double det = g00*g11-g01*g01;
    out.jacobian_determinant = det;
    const double scale = std::max(1.0, (g00+g11)*(g00+g11));
    if (!std::isfinite(det) || det <= 1.0e-8*scale) {
        out.guard = ThrustSupportGuard::SingularJacobian; return out;
    }
    const double err_f = out.desired_forward_velocity-out.nominal_forward_velocity;
    // desired COM vz is deliberately the existing nominal task output. The
    // second equality residual is therefore zero up to roundoff.
    const double lambda_f = g11*err_f/det;
    const double lambda_z = -g01*err_f/det;
    const Eigen::Vector4d delta = jf.transpose()*lambda_f+jz.transpose()*lambda_z;
    for (std::size_t i=0;i<4;++i) {
        out.delta_qdot[i] = delta[static_cast<Eigen::Index>(i)];
        out.qdot[i] = in.nominal_qdot[i]+out.delta_qdot[i];
        const double position_limit = i%2 ? 1.56 : 1.52;
        if (!std::isfinite(out.qdot[i]) ||
            std::abs(out.qdot[i]) > (i%2 ? in.knee_speed_limit : in.hip_speed_limit) ||
            std::abs(in.q[i]+0.030*out.qdot[i]) > position_limit) {
            out.qdot = in.nominal_qdot;
            out.delta_qdot = {};
            out.guard = ThrustSupportGuard::SpeedLimit;
            return out;
        }
    }
    const Eigen::Map<const Eigen::Vector4d> qdot(out.qdot.data());
    out.resulting_forward_velocity = jf.dot(qdot)+out.contact_vertical*in.pitch_rate;
    out.resulting_vertical_velocity = jz.dot(qdot)-out.contact_forward*in.pitch_rate;
    out.active = true;
    out.guard = ThrustSupportGuard::Active;
    return out;
}

} // namespace bbot_jump
