#include "bbot_balance_controller/thrust_wheel_kinematics_observer.hpp"
#include "bbot_balance_controller/jump_phase_control.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
}

using namespace bbot_jump;

static const Eigen::Vector2d kForward(0.0, 1.0);
static const Eigen::Matrix3d kIdentity = Eigen::Matrix3d::Identity();
static const std::array<double, 4> kZeroQ{};

static const ThrustWheelKinematicsEstimate & frame(
    ThrustWheelKinematicsObserver & observer, JointPoseHistory & history,
    double stamp, const std::array<double, 4> & q,
    const Eigen::Matrix3d & rotation = kIdentity,
    const Eigen::Vector2d & forward = kForward, double now_offset = 0.0) {
    history.push(stamp, q);
    return observer.update(stamp, stamp + now_offset, rotation, history,
                           forward, 9.5, 0.07);
}

int main() {
    // A stationary robot must not manufacture a rolling correction.
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        const auto & seed = frame(observer, history, 1.0, kZeroQ);
        require(!seed.frame_valid, "first aligned pose must seed, not estimate a rate");
        const auto & stationary = frame(observer, history, 1.02, kZeroQ);
        require(stationary.frame_valid && stationary.fresh,
                "two complete stationary frames did not produce a fresh estimate");
        require(std::abs(stationary.r_dot) < 1e-10 &&
                std::abs(stationary.shank_projection) < 1e-10 &&
                std::abs(stationary.correction) < 1e-10,
                "stationary frame generated nonzero kinematic compensation");
    }

    // For +X joint axis and +Y forward, increasing both leg angles gives a
    // negative projected shank rate. One-sided motion contributes half the
    // rate because the two legs are averaged.
    {
        ThrustWheelKinematicsObserver bilateral;
        JointPoseHistory history;
        frame(bilateral, history, 2.0, kZeroQ);
        const std::array<double, 4> both{0.02, 0.02, 0.02, 0.02};
        const auto & estimate = frame(bilateral, history, 2.02, both);
        require(estimate.frame_valid && estimate.shank_projection < 0.0,
                "bilateral extension has wrong shank-rate sign");
        require(std::abs(estimate.shank_projection + 2.0) < 1e-6,
                "bilateral motion was not averaged across both legs");
        const double expected = estimate.r_dot + 0.07 * estimate.shank_projection;
        require(std::abs(estimate.raw_correction - expected) < 1e-12,
                "correction does not implement r_dot + R*projected shank rate");

        ThrustWheelKinematicsObserver unilateral;
        JointPoseHistory one_history;
        frame(unilateral, one_history, 2.0, kZeroQ);
        const std::array<double, 4> one_leg{0.02, 0.02, 0.0, 0.0};
        const auto & one = frame(unilateral, one_history, 2.02, one_leg);
        require(std::abs(one.shank_projection - 0.5 * estimate.shank_projection) < 1e-6,
                "one-leg motion did not use the left/right mean");
    }

    // Rotating the entire pose and frozen forward axis together must preserve
    // the scalar correction (yaw equivariance).
    {
        const Eigen::Matrix3d yaw = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector2d forward = (yaw * Eigen::Vector3d::UnitY()).head<2>().normalized();
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 3.0, kZeroQ, yaw, forward);
        const std::array<double, 4> folded{-0.02, -0.02, -0.02, -0.02};
        const auto & estimate = frame(observer, history, 3.02, folded, yaw, forward);
        require(estimate.frame_valid && estimate.correction > 0.0,
                "yaw-rotated world frame lost valid positive brake correction");

        ThrustWheelKinematicsObserver identity_observer;
        JointPoseHistory identity_history;
        frame(identity_observer, identity_history, 3.0, kZeroQ);
        const auto & identity = frame(identity_observer, identity_history, 3.02, folded);
        require(std::abs(estimate.raw_correction - identity.raw_correction) < 1e-8,
                "kinematic correction changed under a common yaw rotation");
    }

    // Exercise the nonzero AngleAxis branch with a body pitch-rate while the
    // joints remain fixed. A common yaw rotation must rotate omega and the
    // frozen axis together without changing the scalar projection.
    {
        const double dtheta = 0.01;
        const Eigen::Matrix3d body_now =
            Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitX()).toRotationMatrix();
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 3.5, kZeroQ, kIdentity);
        const auto & estimate = frame(observer, history, 3.52, kZeroQ, body_now);
        require(estimate.frame_valid && estimate.shank_projection < 0.0 &&
                std::abs(estimate.shank_projection + 0.5) < 1e-6,
                "body angular velocity was not projected with the expected sign/rate");

        const Eigen::Matrix3d yaw = Eigen::AngleAxisd(-0.61, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector2d yaw_forward = (yaw * Eigen::Vector3d::UnitY()).head<2>().normalized();
        ThrustWheelKinematicsObserver yaw_observer;
        JointPoseHistory yaw_history;
        frame(yaw_observer, yaw_history, 3.5, kZeroQ, yaw, yaw_forward);
        const auto & yaw_estimate = frame(yaw_observer, yaw_history, 3.52, kZeroQ,
                                           yaw * body_now, yaw_forward);
        require(yaw_estimate.frame_valid &&
                std::abs(yaw_estimate.shank_projection - estimate.shank_projection) < 1e-8 &&
                std::abs(yaw_estimate.raw_correction - estimate.raw_correction) < 1e-8,
                "body-rate correction was not equivariant under common yaw rotation");
    }

    // Duplicate evidence is idempotent; conflicting same-stamp data and
    // rollback/long intervals reset the estimate.
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 4.0, kZeroQ);
        const std::array<double, 4> moving{-0.01, -0.01, -0.01, -0.01};
        const auto & valid = frame(observer, history, 4.02, moving);
        const double correction = valid.correction;
        const auto & duplicate = observer.update(4.02, 4.025, kIdentity, history,
                                                  kForward, 9.5, 0.07);
        require(duplicate.frame_valid && duplicate.correction == correction,
                "identical repeated frame changed or erased the last estimate");
        const std::array<double, 4> conflict{-0.02, -0.02, -0.02, -0.02};
        history.push(4.02, conflict);
        const auto & bad_duplicate = observer.update(4.02, 4.025, kIdentity, history,
                                                       kForward, 9.5, 0.07);
        require(!bad_duplicate.frame_valid && bad_duplicate.stamp < 0.0,
                "same-stamp joint conflict did not reset history");

        frame(observer, history, 4.04, conflict);
        const auto & rollback = observer.update(4.03, 4.035, kIdentity, history,
                                                 kForward, 9.5, 0.07);
        require(!rollback.frame_valid, "timestamp rollback did not invalidate estimate");
    }
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 5.0, kZeroQ);
        const auto & long_gap = frame(observer, history, 5.081, kZeroQ);
        require(!long_gap.frame_valid, "interval above 80 ms was accepted");
        const auto & restored = frame(observer, history, 5.101, kZeroQ);
        require(restored.frame_valid, "observer did not recover after a long-gap reseed");
    }

    // A missing joint upper bracket retains a fresh complete estimate only up
    // to 40 ms; expiry clears history and requires two new complete frames.
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 6.0, kZeroQ);
        const std::array<double, 4> moving{-0.01, -0.01, -0.01, -0.01};
        const auto & valid = frame(observer, history, 6.02, moving);
        require(valid.frame_valid, "pending-age test failed to create initial estimate");
        const auto & pending = observer.update(6.04, 6.055, kIdentity, history,
                                               kForward, 9.5, 0.07);
        require(pending.frame_valid && pending.fresh,
                "pending joint upper bracket prematurely discarded fresh estimate");
        const auto & expired = observer.update(6.04, 6.061, kIdentity, history,
                                               kForward, 9.5, 0.07);
        require(!expired.frame_valid && expired.stamp < 0.0,
                "expired pending estimate did not clear derivative history");
        frame(observer, history, 6.08, kZeroQ);
        const auto & recovered = frame(observer, history, 6.10, kZeroQ);
        require(recovered.frame_valid,
                "observer failed to rebuild from two complete frames after stale reset");
    }

    // Invalid geometry, frozen-axis changes and malformed transforms must
    // reject the candidate rather than feed an arbitrary wheel command.
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 7.0, kZeroQ);
        const auto & tilted = frame(observer, history, 7.02, kZeroQ,
            Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix());
        require(!tilted.frame_valid, "non-ground-compatible joint axis was accepted");

        observer.reset();
        JointPoseHistory axis_history;
        frame(observer, axis_history, 7.0, kZeroQ);
        const Eigen::Vector2d changed_axis(0.1, std::sqrt(0.99));
        const auto & changed = frame(observer, axis_history, 7.02, kZeroQ,
                                     kIdentity, changed_axis);
        require(!changed.frame_valid, "changed frozen J axis was accepted");

        Eigen::Matrix3d invalid = kIdentity;
        invalid(0, 0) = 1.01;
        observer.reset();
        JointPoseHistory invalid_history;
        const auto & rejected = frame(observer, invalid_history, 7.0, kZeroQ, invalid);
        require(!rejected.frame_valid, "non-orthonormal rotation was accepted");
    }

    // Positive compensation is bounded at 0.35 m/s and enters before the
    // existing total target clamp. Invalid/negative corrections add nothing.
    {
        ThrustWheelKinematicsObserver observer;
        JointPoseHistory history;
        frame(observer, history, 8.0, kZeroQ);
        const std::array<double, 4> fast_fold{-0.20, -0.20, -0.20, -0.20};
        const auto & estimate = frame(observer, history, 8.02, fast_fold);
        require(estimate.frame_valid && estimate.raw_correction > 0.35 &&
                estimate.correction == 0.35,
                "positive kinematic correction did not saturate at 0.35 m/s");
        require(std::abs(thrust_ground_wheel_target_with_kinematics(.45, 0.0,
                estimate.correction) + .10) < 1e-12,
                "bounded correction did not add positive braking before the target clamp");
        require(thrust_ground_wheel_target_with_kinematics(.45, 0.0, -0.20) == -.45,
                "negative kinematic correction increased forward acceleration");
        require(thrust_ground_wheel_target_with_kinematics(.45, 2.0, .35) == 1.50,
                "kinematic correction bypassed the existing total target limit");
        require(thrust_ground_wheel_target_with_kinematics(.45, 0.0,
                std::numeric_limits<double>::quiet_NaN()) == -.45,
                "nonfinite correction did not fall back to the original wheel law");
    }

    std::cout << "PASS: aligned THRUST wheel kinematics observer, freshness, reset, and bounded target correction\n";
}
