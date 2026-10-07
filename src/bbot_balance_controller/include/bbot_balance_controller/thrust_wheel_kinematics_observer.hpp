#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "bbot_balance_controller/centroidal_state.hpp"

namespace bbot_jump {

struct ThrustWheelKinematicsEstimate {
    double stamp = -1.0;
    double age = 0.0;
    double dt = 0.0;
    double r_dot = 0.0;
    double shank_projection = 0.0;
    double correction = 0.0;
    double raw_correction = 0.0;
    bool frame_valid = false;
    bool fresh = false;
    std::array<double, 4> q{};
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d com_minus_axle_body = Eigen::Vector3d::Zero();
};

// Aligns CAD COM/axle geometry at distinct odometry stamps. An odometry frame
// whose joint-history upper bracket has not arrived remains pending; no q/R
// extrapolation is performed. The last complete estimate remains usable only
// for 40 ms while that pending frame catches up.
class ThrustWheelKinematicsObserver {
public:
    static constexpr double kMaxInterval = 0.080;
    static constexpr double kMaxAge = 0.040;
    static constexpr double kWheelRadius = 0.07;

    void reset() {
        clear_history();
        estimate_ = {};
    }

private:
    void clear_history() {
        have_previous_ = false;
        have_seen_ = false;
        seen_q_valid_ = false;
        previous_stamp_ = -1.0;
        seen_stamp_ = -1.0;
        previous_q_ = {};
        seen_q_ = {};
        previous_rotation_.setIdentity();
        seen_rotation_.setIdentity();
        previous_forward_.setZero();
        seen_forward_.setZero();
    }

public:
    const ThrustWheelKinematicsEstimate & update(
        double stamp, double now, const Eigen::Matrix3d & rotation,
        const JointPoseHistory & history, const Eigen::Vector2d & frozen_forward,
        double body_mass, double wheel_radius = kWheelRadius)
    {
        if (!std::isfinite(stamp) || !std::isfinite(now) || stamp < 0.0 ||
            now < stamp || now - stamp > kMaxAge || !rotation.allFinite() ||
            (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
            std::abs(rotation.determinant() - 1.0) > 1e-5 ||
            !frozen_forward.allFinite() ||
            std::abs(frozen_forward.norm() - 1.0) > 1e-3 ||
            !std::isfinite(body_mass) || body_mass <= 0.0 ||
            !std::isfinite(wheel_radius) || wheel_radius <= 0.0) {
            reset();
            return estimate_;
        }

        const Eigen::Vector3d forward(frozen_forward.x(), frozen_forward.y(), 0.0);
        if (forward.norm() < 0.999 || forward.norm() > 1.001) {
            reset();
            return estimate_;
        }
        const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
        if (have_previous_ && (forward - previous_forward_).norm() > 1e-6) {
            reset();
            return estimate_;
        }
        const Eigen::Vector3d joint_axis = rotation.col(0).normalized();
        const double ground_projection = up.cross(joint_axis).dot(forward);
        if (!std::isfinite(ground_projection) || ground_projection <= 0.5 ||
            std::abs(joint_axis.z()) > 0.15) {
            reset();
            return estimate_;
        }

        // Detect stamp conflicts even while joint interpolation is pending.
        if (have_seen_) {
            if (stamp < seen_stamp_) {
                reset();
                return estimate_;
            }
            if ((forward - seen_forward_).norm() > 1e-6) {
                reset();
                return estimate_;
            }
            if (stamp == seen_stamp_ &&
                (!same_rotation(rotation, seen_rotation_) ||
                 (forward - seen_forward_).norm() > 1e-6)) {
                reset();
                return estimate_;
            }
        }

        std::array<double, 4> q{};
        if (!history.interpolate(stamp, q)) {
            if (!have_seen_ || stamp > seen_stamp_) {
                seen_stamp_ = stamp;
                seen_rotation_ = rotation;
                seen_forward_ = forward;
                have_seen_ = true;
                seen_q_valid_ = false;
            }
            refresh_age(now);
            return estimate_;
        }
        for (double value : q) {
            if (!std::isfinite(value)) {
                reset();
                return estimate_;
            }
        }
        if (have_seen_ && stamp == seen_stamp_ && seen_q_valid_ && !same_q(q, seen_q_)) {
            reset();
            return estimate_;
        }
        if (have_previous_ && stamp == previous_stamp_) {
            if (!same_rotation(rotation, previous_rotation_) ||
                !same_q(q, previous_q_) || (forward - previous_forward_).norm() > 1e-6) {
                reset();
                return estimate_;
            }
            refresh_age(now);
            return estimate_;  // duplicate complete frame: no new evidence
        }

        if (!have_previous_) {
            seed(stamp, rotation, q, forward);
            set_frame(stamp, now, 0.0, rotation, q, body_mass);
            seen_stamp_ = stamp;
            seen_rotation_ = rotation;
            seen_forward_ = forward;
            seen_q_ = q;
            seen_q_valid_ = true;
            have_seen_ = true;
            refresh_age(now);
            return estimate_;
        }

        const double dt = stamp - previous_stamp_;
        if (dt < 0.001 || dt > kMaxInterval) {
            reset();
            seed(stamp, rotation, q, forward);
            set_frame(stamp, now, 0.0, rotation, q, body_mass);
            seen_stamp_ = stamp;
            seen_rotation_ = rotation;
            seen_forward_ = forward;
            seen_q_ = q;
            seen_q_valid_ = true;
            have_seen_ = true;
            refresh_age(now);
            return estimate_;
        }

        const auto previous_geometry = centroidal_geometry(previous_q_, body_mass);
        const auto current_geometry = centroidal_geometry(q, body_mass);
        const Eigen::Vector3d previous_relative =
            previous_rotation_ * (previous_geometry.com - previous_geometry.axle);
        const Eigen::Vector3d current_relative =
            rotation * (current_geometry.com - current_geometry.axle);
        const Eigen::Vector3d previous_forward_3(
            previous_forward_.x(), previous_forward_.y(), 0.0);
        const Eigen::Vector3d current_forward_3(forward.x(), forward.y(), 0.0);
        const double previous_r = previous_forward_3.dot(previous_relative);
        const double current_r = current_forward_3.dot(current_relative);
        const Eigen::Matrix3d delta_rotation = rotation * previous_rotation_.transpose();
        const Eigen::AngleAxisd angle_axis(delta_rotation);
        if (!std::isfinite(previous_r) || !std::isfinite(current_r) ||
            !std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite()) {
            reset();
            return estimate_;
        }
        const Eigen::Vector3d omega_body_world = angle_axis.axis() * (angle_axis.angle() / dt);
        Eigen::Vector3d mean_joint_axis = previous_rotation_.col(0) + rotation.col(0);
        if (!mean_joint_axis.allFinite() || mean_joint_axis.norm() < 0.5) {
            reset();
            return estimate_;
        }
        mean_joint_axis.normalize();
        const double previous_leg_angle = 0.5 *
            (previous_q_[0] + previous_q_[1] + previous_q_[2] + previous_q_[3]);
        const double current_leg_angle = 0.5 * (q[0] + q[1] + q[2] + q[3]);
        const Eigen::Vector3d omega_shank_world = omega_body_world + mean_joint_axis *
            ((current_leg_angle - previous_leg_angle) / dt);
        const double r_dot = (current_r - previous_r) / dt;
        const double shank_projection = omega_shank_world.cross(up).dot(forward);
        const double raw = r_dot + wheel_radius * shank_projection;
        if (!std::isfinite(r_dot) || !std::isfinite(shank_projection) || !std::isfinite(raw)) {
            reset();
            return estimate_;
        }

        estimate_.stamp = stamp;
        estimate_.age = now - stamp;
        estimate_.dt = dt;
        estimate_.r_dot = r_dot;
        estimate_.shank_projection = shank_projection;
        estimate_.raw_correction = raw;
        estimate_.correction = std::clamp(raw, 0.0, 0.35);
        estimate_.frame_valid = true;
        estimate_.fresh = true;
        estimate_.q = q;
        estimate_.rotation = rotation;
        estimate_.com_minus_axle_body = current_geometry.com - current_geometry.axle;

        seed(stamp, rotation, q, forward);
        seen_stamp_ = stamp;
        seen_rotation_ = rotation;
        seen_forward_ = forward;
        seen_q_ = q;
        seen_q_valid_ = true;
        have_seen_ = true;
        return estimate_;
    }

private:
    static bool same_q(const std::array<double, 4> & a, const std::array<double, 4> & b) {
        for (std::size_t i = 0; i < a.size(); ++i)
            if (std::abs(a[i] - b[i]) > 1e-9) return false;
        return true;
    }
    static bool same_rotation(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b) {
        return (a - b).norm() <= 1e-8;
    }
    void seed(double stamp, const Eigen::Matrix3d & rotation,
              const std::array<double, 4> & q, const Eigen::Vector3d & forward) {
        previous_stamp_ = stamp;
        previous_rotation_ = rotation;
        previous_q_ = q;
        previous_forward_ = forward;
        have_previous_ = true;
    }
    void set_frame(double stamp, double now, double dt,
                   const Eigen::Matrix3d & rotation,
                   const std::array<double, 4> & q, double body_mass) {
        estimate_.stamp = stamp;
        estimate_.age = now - stamp;
        estimate_.dt = dt;
        estimate_.frame_valid = false;
        estimate_.fresh = estimate_.age >= 0.0 && estimate_.age <= kMaxAge;
        estimate_.q = q;
        estimate_.rotation = rotation;
        const auto geometry = centroidal_geometry(q, body_mass);
        estimate_.com_minus_axle_body = geometry.com - geometry.axle;
    }
    void refresh_age(double now) {
        if (estimate_.stamp < 0.0) return;
        estimate_.age = now - estimate_.stamp;
        estimate_.fresh = std::isfinite(estimate_.age) && estimate_.age >= 0.0 &&
                          estimate_.age <= kMaxAge;
        if (!estimate_.fresh) {
            reset();
        }
    }

    bool have_previous_ = false;
    bool have_seen_ = false;
    bool seen_q_valid_ = false;
    double previous_stamp_ = -1.0;
    double seen_stamp_ = -1.0;
    std::array<double, 4> previous_q_{};
    std::array<double, 4> seen_q_{};
    Eigen::Matrix3d previous_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d seen_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d previous_forward_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d seen_forward_ = Eigen::Vector3d::Zero();
    ThrustWheelKinematicsEstimate estimate_;
};

}  // namespace bbot_jump
