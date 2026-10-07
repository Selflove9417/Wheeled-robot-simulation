// Targeted unit tests for the isolated offline friction-response predictor.
// This executable is deliberately not registered in CMake by this task.
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "bbot_balance_controller/ground_friction_response.hpp"

namespace
{
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

bbot_jump::GroundFrictionResponseInput symmetric_equilibrium(
    bbot_jump::ThrustSupportDynamicsModel *model_out = nullptr,
    bbot_jump::SupportContactVector *force_out = nullptr)
{
    using namespace bbot_jump;
    GroundFrictionResponseInput in;
    // Build a true static support, with the model COM directly above the
    // symmetric wheel axle pair. Contact coordinates recover axle centers by
    // removing the rolling-angle term from their forward coordinate.
    in.q << 0.0, 0.0, 0.0, 0.5, -1.0, 0.5, -1.0, 0.0, 0.0;
    auto wheel_centers_in_base_coordinates = [&]() {
        const auto contact = thrust_support_contact_coordinates(in.q, kSupportWheelRadius);
        Eigen::Matrix<double, 2, 2> centers;
        for (int side = 0; side < 2; ++side)
        {
            const int h = 3 + 2 * side, k = h + 1;
            const double absolute_wheel_angle = in.q[2] + in.q[h] + in.q[k] + in.q[7 + side];
            centers(0, side) = contact[2 * side] - kSupportWheelRadius * absolute_wheel_angle;
            centers(1, side) = contact[2 * side + 1] + kSupportWheelRadius;
        }
        return centers;
    };
    const Eigen::Vector2d com_local = thrust_support_com(in.q, 9.5) - in.q.head<2>();
    const Eigen::Vector2d mean_axle_local = wheel_centers_in_base_coordinates().rowwise().mean();
    const Eigen::Vector2d com_to_axle = com_local - mean_axle_local;
    in.q[2] = std::atan2(com_to_axle.x(), com_to_axle.y());
    in.q[1] = 0.0;
    in.q[1] = kSupportWheelRadius - wheel_centers_in_base_coordinates()(1, 0);

    ThrustSupportDynamicsInput model_input;
    model_input.q = in.q;
    model_input.v = in.v;
    ThrustSupportDynamicsModel model;
    require(thrust_support_dynamics_model(model_input, model), "fixture CAD state invalid");
    SupportContactVector support_force(0.0, (9.5 + 8.0) * kSupportGravity / 2.0,
                                       0.0, (9.5 + 8.0) * kSupportGravity / 2.0);
    in.command = (model.gravity - model.contact_jacobian.transpose() * support_force).tail<6>();
    if (model_out) *model_out = model;
    if (force_out) *force_out = support_force;
    return in;
}

void require_discrete_dynamics_balance(
    const bbot_jump::GroundFrictionResponseInput &in,
    const bbot_jump::GroundFrictionResponseResult &result)
{
    using namespace bbot_jump;
    ThrustSupportDynamicsInput model_input;
    model_input.q = in.q;
    model_input.v = in.v;
    model_input.prediction_dt = in.dt;
    ThrustSupportDynamicsModel model;
    require(thrust_support_dynamics_model(model_input, model), "balance fixture CAD state invalid");
    SupportMatrix9 free_mass = model.mass;
    SupportQ9 free_rhs = -(model.velocity_bias + model.gravity);
    for (int a = 0; a < 6; ++a)
    {
        const int dof = 3 + a;
        const double damping = a < 4 ? 0.5 : 0.1;
        free_mass(dof, dof) += in.dt * damping;
        free_rhs[dof] += in.command[a] - damping * in.v[dof];
    }
    require((free_mass * result.free_acceleration - free_rhs).cwiseAbs().maxCoeff() < 2e-7,
            "implicit free-stage damping equation mismatch");
    require(result.free_stage_equation_residual < 2e-7,
            "reported free-stage equation residual is too large");
    require((result.free_velocity - (in.v + in.dt * result.free_acceleration))
                .cwiseAbs().maxCoeff() < 1e-12,
            "free velocity does not follow the forward-dynamics stage");
    SupportQ9 lhs = model.mass * result.acceleration;
    SupportQ9 rhs = model.mass * result.free_acceleration +
                    model.contact_jacobian.transpose() * result.contact_force;
    rhs.segment<4>(3) += result.leg_friction_torque;
    require((lhs - rhs).cwiseAbs().maxCoeff() < 2e-7,
            "ordinary-M constraint impulse update mismatch");
}
}

int main()
{
    using namespace bbot_jump;
    try
    {
        // Tiny signed numerical noise is absorbed by static friction. It must
        // not trigger the legacy sign(v) Coulomb jump.
        ThrustSupportDynamicsModel static_model;
        SupportContactVector static_force;
        auto quiet = symmetric_equilibrium(&static_model, &static_force);
        const auto static_contact = thrust_support_contact_coordinates(quiet.q, kSupportWheelRadius);
        // Contact coordinates already include the world base translation.
        const double left_wheel_z = static_contact[1] + kSupportWheelRadius;
        const double right_wheel_z = static_contact[3] + kSupportWheelRadius;
        require(std::abs(left_wheel_z - kSupportWheelRadius) < 1e-12 &&
                std::abs(right_wheel_z - kSupportWheelRadius) < 1e-12,
                "static fixture wheel axles are not at ground-contact height");
        const double com_forward = thrust_support_com(quiet.q, 9.5).x();
        const double left_axle_forward = static_contact[0] - kSupportWheelRadius *
            (quiet.q[2] + quiet.q[3] + quiet.q[4] + quiet.q[7]);
        const double right_axle_forward = static_contact[2] - kSupportWheelRadius *
            (quiet.q[2] + quiet.q[5] + quiet.q[6] + quiet.q[8]);
        require(std::abs(com_forward - 0.5 * (left_axle_forward + right_axle_forward)) < 1e-12,
                "static fixture COM is not above mean wheel axle");
        require((static_model.gravity.head<3>() -
                 (static_model.contact_jacobian.transpose() * static_force).head<3>())
                    .cwiseAbs().maxCoeff() < 1e-12,
                "static fixture contact forces do not balance base generalized gravity");
        require(std::abs(static_force[0]) <= static_force[1] + 1e-12 &&
                std::abs(static_force[2]) <= static_force[3] + 1e-12 &&
                static_force[1] > 0.0 && static_force[3] > 0.0,
                "static fixture contact force is outside the friction cone");
        SupportQ9 static_rhs = SupportQ9::Zero();
        static_rhs.tail<6>() = quiet.command;
        static_rhs += static_model.contact_jacobian.transpose() * static_force;
        require((static_rhs - static_model.gravity).cwiseAbs().maxCoeff() < 1e-12,
                "static fixture fails the complete generalized force balance");

        quiet.v[3] = 1e-13;
        quiet.v[4] = -1e-13;
        quiet.v[5] = -1e-13;
        quiet.v[6] = 1e-13;
        auto result = ground_friction_response(quiet);
        require(result.valid, "small-noise static equilibrium rejected");
        require(result.candidate_modes >= 1, "static response has no feasible mode");
        require(result.distinct_accelerations == 1, "boundary-equivalent static modes not deduplicated");
        require(result.acceleration.segment<4>(3).cwiseAbs().maxCoeff() < 2e-10,
                "small leg-rate noise caused nonzero acceleration");
        require(result.max_stick_effort <= 0.1 + 1e-8, "static friction exceeded its bound");

        // Captured 10.308 s regression frame. Under the split free/impulse
        // update, incompatible sliding branches must fail strict predicted
        // velocity sign checks; retain the frame and its one feasible result.
        GroundFrictionResponseInput boundary;
        boundary.q << 0.64918735116284509, 0.50762789797506813, -0.069950279008315436,
                      0.25577663063144013, -0.36923198343372499, 0.25577663133989709,
                     -0.36923198395626172, -8.8918662649634399, -8.8918661517103512;
        boundary.v << -0.0020531313255034568, -0.00018803039870607936, 0.0018469353950395584,
                     -5.6053078845152982e-13, 0.00013162654255469142, -5.606071162844728e-13,
                      0.0001316264863383404, 0.015346867058834972, 0.015346867074819193;
        boundary.command << -0.73715550093908921, -15.29628183759197,
                            -0.73715550819894016, -15.296281830889368,
                            -0.016821844269629235, -0.01682184363254741;
        result = ground_friction_response(boundary);
        require(result.valid, "qualified near-zero stick complement rejected");
        require(result.rejection_counts[ground_friction_detail::kRejectSlideDirection] > 0,
                "opposite/near-zero slide candidates were not rejected by strict sign checks");
        require(result.distinct_accelerations == 1,
                "boundary-equivalent modes incorrectly became distinct accelerations");

        // Retain the old theta-zero command=gravity-tail fixture as a
        // negative control: its symmetric wheel support wrench cannot balance
        // its base pitch gravity, so it is not a static equilibrium.
        GroundFrictionResponseInput old_nonstatic;
        old_nonstatic.q << 0.0, 0.39, 0.0, 0.5, -1.0, 0.5, -1.0, 0.0, 0.0;
        ThrustSupportDynamicsInput old_model_input;
        old_model_input.q = old_nonstatic.q;
        old_model_input.v = old_nonstatic.v;
        ThrustSupportDynamicsModel old_model;
        require(thrust_support_dynamics_model(old_model_input, old_model),
                "old negative-control CAD state invalid");
        old_nonstatic.command = old_model.gravity.tail<6>();
        const double base_pitch_support = old_model.contact_jacobian(1, 2) * old_model.gravity[1];
        require(std::abs(old_model.gravity[2] - base_pitch_support) > 1.0,
                "old theta-zero fixture unexpectedly satisfies static base support");

        // Above the static-friction boundary, a positive command increment
        // must select positive predicted slip with opposing dynamic friction.
        auto breakaway = symmetric_equilibrium();
        breakaway.command[0] += 0.5;
        result = ground_friction_response(breakaway);
        require(result.valid, "finite breakaway response rejected");
        require(result.acceleration[3] > 0.0, "above-bound command did not break away positively");
        require(std::abs(result.leg_friction_torque[0] + 0.1) < 1e-8,
                "positive sliding friction does not oppose predicted motion");
        require_discrete_dynamics_balance(breakaway, result);

        // Mirroring left/right state and input must mirror the predicted
        // sagittal response and contact witness.
        auto asymmetric = symmetric_equilibrium();
        asymmetric.q[3] += 0.025;
        asymmetric.q[5] -= 0.025;
        asymmetric.v[3] = 0.07;
        asymmetric.v[5] = -0.07;
        asymmetric.command[0] += 0.8;
        asymmetric.command[2] -= 0.8;
        result = ground_friction_response(asymmetric);
        require(result.valid, "mirrored coupled response rejected");
        GroundFrictionResponseInput mirror = asymmetric;
        std::swap(mirror.q[3], mirror.q[5]);
        std::swap(mirror.q[4], mirror.q[6]);
        std::swap(mirror.v[3], mirror.v[5]);
        std::swap(mirror.v[4], mirror.v[6]);
        std::swap(mirror.command[0], mirror.command[2]);
        std::swap(mirror.command[1], mirror.command[3]);
        std::swap(mirror.command[4], mirror.command[5]);
        const auto mirrored_result = ground_friction_response(mirror);
        require(mirrored_result.valid, "left/right mirrored response rejected");
        const auto &a = result.acceleration;
        const auto &b = mirrored_result.acceleration;
        require(std::abs(a[0] - b[0]) < 2e-6 && std::abs(a[1] - b[1]) < 2e-6 &&
                std::abs(a[2] - b[2]) < 2e-6 && std::abs(a[3] - b[5]) < 2e-6 &&
                std::abs(a[4] - b[6]) < 2e-6 && std::abs(a[5] - b[3]) < 2e-6 &&
                std::abs(a[6] - b[4]) < 2e-6 && std::abs(a[7] - b[8]) < 2e-6 &&
                std::abs(a[8] - b[7]) < 2e-6,
                "coupled left/right acceleration symmetry mismatch");

        // Reverse torque remains a normal forward prediction, even if it
        // initially accelerates with positive velocity before braking.
        auto reverse = symmetric_equilibrium();
        reverse.v[3] = 0.2;
        reverse.command[0] -= 0.5;
        result = ground_friction_response(reverse);
        require(result.valid, "reverse-command sample was filtered/rejected");
        require(result.acceleration.allFinite(), "reverse-command prediction is nonfinite");
        require(result.acceleration[3] < 0.0,
                "opposite-sign hip command did not produce braking acceleration");
        require(reverse.v[3] + reverse.dt * result.acceleration[3] > 0.0,
                "braking sample was forced to reverse velocity within one step");
        require_discrete_dynamics_balance(reverse, result);
        bool free_final_viscous_difference_seen = false;
        for (int j = 0; j < 4; ++j)
        {
            const int dof = 3 + j;
            require(std::abs(result.viscous_torque[dof] - 0.5 * result.free_velocity[dof]) < 1e-12,
                    "leg viscous diagnostic is not evaluated at free velocity");
            if (std::abs(result.free_velocity[dof] -
                         (reverse.v[dof] + reverse.dt * result.acceleration[dof])) > 1e-8)
                free_final_viscous_difference_seen = true;
        }
        for (int j = 0; j < 2; ++j)
        {
            const int dof = 7 + j;
            require(std::abs(result.viscous_torque[dof] - 0.1 * result.free_velocity[dof]) < 1e-12,
                    "wheel viscous diagnostic is not evaluated at free velocity");
        }
        require(free_final_viscous_difference_seen,
                "test failed to distinguish free-stage and final constrained velocity");

        SupportActuatorVector aggregate_command = SupportActuatorVector::Zero();
        SupportActuatorVector aggregate_wrench = SupportActuatorVector::Zero();
        SupportQ9 aggregate_viscous = SupportQ9::Zero();
        Eigen::Matrix<double, 4, 1> aggregate_friction = Eigen::Matrix<double, 4, 1>::Zero();
        aggregate_friction[0] = -0.1;
        const auto aggregate = ground_friction_aggregate_difference(
            aggregate_command, aggregate_viscous, aggregate_friction, aggregate_wrench);
        require(std::abs(aggregate[0] + 0.1) < 1e-12,
                "aggregate residual reversed the applied friction torque sign");

        auto bad = symmetric_equilibrium();
        bad.v[4] = std::numeric_limits<double>::quiet_NaN();
        require(!ground_friction_response(bad).valid, "NaN state accepted");
        bad = symmetric_equilibrium();
        bad.dt = 0.002;
        require(!ground_friction_response(bad).valid, "unsupported time step accepted");
        bad = symmetric_equilibrium();
        bad.bilateral_contact = false;
        require(!ground_friction_response(bad).valid, "unsupported contact mode accepted");
        bad = symmetric_equilibrium();
        bad.command[0] = 75.01;
        require(!ground_friction_response(bad).valid, "over-limit physical input accepted");

        std::cout << "ground friction response tests passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ground friction response test failed: " << e.what() << '\n';
        return 1;
    }
}
