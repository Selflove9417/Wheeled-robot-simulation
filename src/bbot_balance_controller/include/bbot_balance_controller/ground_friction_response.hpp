#pragma once

// Offline-only friction response using one explicit fixed-source 9-DOF profile. This
// enumerates DART-style static/dynamic joint-friction modes; it is not wired to
// a controller and does not alter thrust_support_dynamics.hpp.
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "bbot_balance_controller/ground_urdf_response_model.hpp"

namespace bbot_jump
{

struct GroundFrictionResponseInput
{
    SupportQ9 q = SupportQ9::Zero();
    SupportQ9 v = SupportQ9::Zero();
    SupportActuatorVector command = SupportActuatorVector::Zero();
    double dt = 0.001;
    bool bilateral_contact = true;
    double wheel_ground_friction = 1.0;
};

struct GroundFrictionResponseResult
{
    bool valid = false;
    bool free_stage_valid = false;
    std::string reason;
    // DART's force stage integrates this free velocity before constraint
    // impulses are solved. Keep both values visible for offline diagnostics.
    SupportQ9 free_acceleration = SupportQ9::Zero();
    SupportQ9 free_velocity = SupportQ9::Zero();
    double free_stage_equation_residual = std::numeric_limits<double>::infinity();
    SupportQ9 acceleration = SupportQ9::Zero();
    SupportContactVector contact_force = SupportContactVector::Zero();
    Eigen::Matrix<double, 4, 1> leg_friction_torque = Eigen::Matrix<double, 4, 1>::Zero();
    SupportQ9 viscous_torque = SupportQ9::Zero();
    std::array<int, 4> leg_mode{{0, 0, 0, 0}};  // -1 slip, 0 stick, +1 slip.
    int candidate_modes = 0;
    int kkt_rank = 0;
    int distinct_accelerations = 0;
    double max_equation_residual = std::numeric_limits<double>::infinity();
    double max_contact_residual = std::numeric_limits<double>::infinity();
    double max_stick_effort = 0.0;
    double max_contact_cone_violation = 0.0;
    std::array<int, 6> rejection_counts{};
    int boundary_redundant_modes = 0;
};

struct GroundFrictionResponseOptions
{
    GroundUrdfResponseProfile response_profile = GroundUrdfResponseProfile::OriginalCAD;
    double dt = 0.001;
    double static_friction = 0.1;
    double dynamic_friction = 0.1;
    double leg_viscous = 0.5;
    double wheel_viscous = 0.1;
    double max_contact_friction = 1.0;
    double max_abs_equation_residual = 1e-8;
    double relative_equation_residual = 1e-10;
    double contact_force_tolerance = 1e-7;
    double contact_residual_tolerance = 1e-8;
    double static_effort_tolerance = 1e-8;
    double slip_velocity_tolerance = 1e-9;
    double acceleration_equivalence_tolerance = 1e-7;
    std::array<double, 6> input_effort_limits{{75., 60., 75., 60., 10., 10.}};
};

namespace ground_friction_detail
{
struct Candidate
{
    SupportQ9 acceleration = SupportQ9::Zero();
    SupportContactVector contact_force = SupportContactVector::Zero();
    Eigen::Matrix<double, 4, 1> friction = Eigen::Matrix<double, 4, 1>::Zero();
    std::array<int, 4> mode{{0, 0, 0, 0}};
    int rank = 0;
    double equation_residual = 0.0;
    double contact_residual = 0.0;
    double stick_effort = 0.0;
    double cone_violation = 0.0;
};

enum Reject : int
{
    kRejectSingularOrNonfinite = 0,
    kRejectEquationResidual = 1,
    kRejectContactResidual = 2,
    kRejectContactCone = 3,
    kRejectStickEffort = 4,
    kRejectSlideDirection = 5
};

inline bool solve_mode(const GroundFrictionResponseInput &input,
                       const GroundFrictionResponseOptions &options,
                       const GroundUrdfResponseModel &model,
                       const SupportQ9 &free_acceleration,
                       const std::array<int, 4> &mode,
                       Candidate &candidate, Reject &rejection)
{
    std::vector<int> stick_joint;
    for (int j = 0; j < 4; ++j)
        if (mode[j] == 0) stick_joint.push_back(3 + j);
    const int stick_count = static_cast<int>(stick_joint.size());
    const int n = 13 + stick_count;
    Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);

    // DART first computes/integrates free velocity with implicit damping, then
    // applies constraint impulses through ordinary (non-implicit) M. Express
    // those impulses as equivalent forces over dt, so the dynamic block is
    // M*a_final = M*a_free + contact_force + Coulomb_force.
    kkt.block<9, 9>(0, 0) = model.M;
    kkt.block<9, 4>(0, 9) = -model.contactJacobian.transpose();
    kkt.block<4, 9>(9, 0) = model.contactJacobian;
    rhs.head<9>() = model.M * free_acceleration;
    rhs.segment<4>(9) = -model.contactBias;

    for (int j = 0; j < 4; ++j)
    {
        if (mode[j] != 0)
        {
            const int dof = 3 + j;
            // mode sign describes predicted v_next. Coulomb torque opposes it.
            rhs[dof] -= options.dynamic_friction * static_cast<double>(mode[j]);
        }
    }
    for (int s = 0; s < stick_count; ++s)
    {
        const int row = 13 + s;
        const int dof = stick_joint[s];
        kkt(dof, row) = -1.0;  // unknown is friction torque applied to the joint.
        kkt(row, dof) = 1.0;
        rhs[row] = -input.v[dof] / options.dt;
    }

    if (!kkt.allFinite() || !rhs.allFinite())
    {
        rejection = kRejectSingularOrNonfinite;
        return false;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(kkt, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success || !svd.singularValues().allFinite())
    {
        rejection = kRejectSingularOrNonfinite;
        return false;
    }
    const double svd_threshold = 64.0 * std::numeric_limits<double>::epsilon() *
                                 static_cast<double>(std::max(n, 1));
    svd.setThreshold(svd_threshold);
    const Eigen::VectorXd x = svd.solve(rhs);
    if (!x.allFinite())
    {
        rejection = kRejectSingularOrNonfinite;
        return false;
    }
    candidate.rank = static_cast<int>(svd.rank());
    candidate.equation_residual = (kkt * x - rhs).cwiseAbs().maxCoeff();
    const double rhs_scale = std::max(1.0, rhs.cwiseAbs().maxCoeff());
    if (candidate.equation_residual > options.max_abs_equation_residual +
        options.relative_equation_residual * rhs_scale)
    {
        rejection = kRejectEquationResidual;
        return false;
    }

    candidate.acceleration = x.head<9>();
    candidate.contact_force = x.segment<4>(9);
    candidate.mode = mode;
    candidate.friction.setZero();
    candidate.stick_effort = 0.0;
    for (int j = 0; j < 4; ++j)
    {
        const int dof = 3 + j;
        if (mode[j] == 0)
        {
            const auto it = std::find(stick_joint.begin(), stick_joint.end(), dof);
            const int s = static_cast<int>(std::distance(stick_joint.begin(), it));
            const double effort = x[13 + s];
            candidate.friction[j] = effort;
            candidate.stick_effort = std::max(candidate.stick_effort, std::abs(effort));
            if (std::abs(effort) > options.static_friction + options.static_effort_tolerance)
            {
                rejection = kRejectStickEffort;
                return false;
            }
        }
        else
        {
            candidate.friction[j] = -options.dynamic_friction * static_cast<double>(mode[j]);
            const double next_velocity = input.v[dof] + options.dt * candidate.acceleration[dof];
            const double sign_roundoff = 64.0 * std::numeric_limits<double>::epsilon() *
                std::max(1.0, std::max(std::abs(input.v[dof]),
                                       std::abs(options.dt * candidate.acceleration[dof])));
            if ((mode[j] > 0 && next_velocity <= sign_roundoff) ||
                (mode[j] < 0 && next_velocity >= -sign_roundoff))
            {
                rejection = kRejectSlideDirection;
                return false;
            }
        }
    }

    const SupportContactVector contact_error = model.contactJacobian * candidate.acceleration + model.contactBias;
    candidate.contact_residual = contact_error.cwiseAbs().maxCoeff();
    if (candidate.contact_residual > options.contact_residual_tolerance)
    {
        rejection = kRejectContactResidual;
        return false;
    }
    candidate.cone_violation = 0.0;
    for (int side = 0; side < 2; ++side)
    {
        const double tangent = candidate.contact_force[2 * side];
        const double normal = candidate.contact_force[2 * side + 1];
        const double violation = std::max(-normal,
            std::abs(tangent) - options.max_contact_friction * normal);
        candidate.cone_violation = std::max(candidate.cone_violation, violation);
        if (normal < -options.contact_force_tolerance ||
            std::abs(tangent) > options.max_contact_friction * std::max(0.0, normal) + options.contact_force_tolerance)
        {
            rejection = kRejectContactCone;
            return false;
        }
    }
    return true;
}
}  // namespace ground_friction_detail

inline GroundFrictionResponseResult ground_friction_response(
    const GroundFrictionResponseInput &input,
    const GroundFrictionResponseOptions &options = GroundFrictionResponseOptions{})
{
    GroundFrictionResponseResult result;
    const auto reject = [&](const std::string &why) {
        result.valid = false;
        result.reason = why;
        return result;
    };
    if (!input.q.allFinite() || !input.v.allFinite() || !input.command.allFinite() ||
        !std::isfinite(input.dt) || !std::isfinite(input.wheel_ground_friction))
        return reject("nonfinite_input");
    if (!input.bilateral_contact)
        return reject("unsupported_contact_mode_requires_bilateral_support");
    if (std::abs(input.dt - options.dt) > 1e-12 || std::abs(options.dt - 0.001) > 1e-12)
        return reject("unsupported_timestep_requires_1ms");
    if (input.wheel_ground_friction < 0.0 ||
        std::abs(input.wheel_ground_friction - options.max_contact_friction) > 1e-12)
        return reject("unsupported_or_invalid_ground_friction_coefficient");
    if (!(options.static_friction >= 0.0) || !(options.dynamic_friction >= 0.0) ||
        !(options.leg_viscous >= 0.0) || !(options.wheel_viscous >= 0.0) ||
        !(options.max_contact_friction >= 0.0))
        return reject("invalid_friction_configuration");
    for (int i = 0; i < 6; ++i)
    {
        if (!std::isfinite(options.input_effort_limits[i]) || options.input_effort_limits[i] <= 0.0)
            return reject("invalid_force_limit_configuration");
        if (std::abs(input.command[i]) > options.input_effort_limits[i] + 1e-9)
            return reject("input_force_limit_exceeded_actuator_" + std::to_string(i));
    }

    ThrustSupportDynamicsInput dynamics_input;
    dynamics_input.q = input.q;
    dynamics_input.v = input.v;
    dynamics_input.prediction_dt = options.dt;
    dynamics_input.friction_coefficient = options.max_contact_friction;
    GroundUrdfResponseModel model;
    if (!build_ground_urdf_response_model(dynamics_input, options.response_profile, model))
        return reject("fixed_cad_model_invalid");

    // Force/forward-dynamics stage. Damping is evaluated at v_free and is not
    // recomputed after the subsequent contact/Coulomb impulse update.
    SupportMatrix9 free_mass = model.M;
    SupportQ9 free_rhs = -(model.C + model.G);
    for (int a = 0; a < 6; ++a)
    {
        const int dof = 3 + a;
        const double viscous = a < 4 ? options.leg_viscous : options.wheel_viscous;
        free_mass(dof, dof) += options.dt * viscous;
        free_rhs[dof] += input.command[a] - viscous * input.v[dof];
    }
    Eigen::FullPivLU<SupportMatrix9> free_solver(free_mass);
    if (!free_mass.allFinite() || !free_rhs.allFinite() || !free_solver.isInvertible())
        return reject("free_forward_dynamics_singular_or_nonfinite");
    result.free_acceleration = free_solver.solve(free_rhs);
    if (!result.free_acceleration.allFinite())
        return reject("free_forward_dynamics_nonfinite_solution");
    result.free_stage_equation_residual =
        (free_mass * result.free_acceleration - free_rhs).cwiseAbs().maxCoeff();
    const double free_rhs_scale = std::max(1.0, free_rhs.cwiseAbs().maxCoeff());
    if (result.free_stage_equation_residual > options.max_abs_equation_residual +
        options.relative_equation_residual * free_rhs_scale)
        return reject("free_forward_dynamics_equation_residual_exceeded");
    result.free_velocity = input.v + options.dt * result.free_acceleration;
    if (!result.free_velocity.allFinite())
        return reject("free_velocity_nonfinite");
    result.free_stage_valid = true;

    std::vector<ground_friction_detail::Candidate> feasible;
    for (int code = 0; code < 81; ++code)
    {
        int value = code;
        std::array<int, 4> mode{};
        for (int j = 0; j < 4; ++j)
        {
            mode[j] = (value % 3) - 1;
            value /= 3;
        }
        ground_friction_detail::Candidate candidate;
        ground_friction_detail::Reject reason = ground_friction_detail::kRejectSingularOrNonfinite;
        if (ground_friction_detail::solve_mode(input, options, model,
                                               result.free_acceleration, mode,
                                               candidate, reason))
            feasible.push_back(candidate);
        else
            ++result.rejection_counts[static_cast<int>(reason)];
    }
    result.candidate_modes = static_cast<int>(feasible.size());
    if (feasible.empty())
    {
        result.reason = "no_consistent_friction_mode; rank-deficient KKT is allowed only when full residual and contact cone pass";
        return result;
    }

    // A sliding mode whose one-step rate lies inside the explicit boundary
    // deadband is redundant only if the corresponding all-near-zero-stick
    // complement is independently feasible under the same full equation,
    // contact residual, and static-friction checks above. Prefer that stick
    // representative; never consume an infeasible stick response or a slide
    // whose predicted velocity is outside the boundary band.
    std::vector<bool> boundary_redundant(feasible.size(), false);
    for (std::size_t i = 0; i < feasible.size(); ++i)
    {
        auto stick_complement = feasible[i].mode;
        bool has_near_zero_slide = false;
        for (int j = 0; j < 4; ++j)
        {
            if (feasible[i].mode[j] == 0) continue;
            const int dof = 3 + j;
            const double next_velocity = input.v[dof] + options.dt * feasible[i].acceleration[dof];
            if (std::abs(next_velocity) <= options.slip_velocity_tolerance)
            {
                stick_complement[j] = 0;
                has_near_zero_slide = true;
            }
        }
        if (!has_near_zero_slide) continue;
        for (std::size_t k = 0; k < feasible.size(); ++k)
        {
            if (k != i && feasible[k].mode == stick_complement)
            {
                boundary_redundant[i] = true;
                ++result.boundary_redundant_modes;
                break;
            }
        }
    }
    std::vector<ground_friction_detail::Candidate> nonredundant;
    nonredundant.reserve(feasible.size());
    for (std::size_t i = 0; i < feasible.size(); ++i)
        if (!boundary_redundant[i]) nonredundant.push_back(feasible[i]);
    feasible.swap(nonredundant);

    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < static_cast<int>(feasible.size()); ++i)
    {
        bool assigned = false;
        for (auto &cluster : clusters)
        {
            const auto &reference = feasible[cluster.front()].acceleration;
            if ((reference - feasible[i].acceleration).cwiseAbs().maxCoeff() <= options.acceleration_equivalence_tolerance)
            {
                cluster.push_back(i);
                assigned = true;
                break;
            }
        }
        if (!assigned) clusters.push_back({i});
    }
    result.distinct_accelerations = static_cast<int>(clusters.size());
    if (clusters.size() != 1)
        return reject("ambiguous_distinct_acceleration_solutions");

    // Boundary stick/slide patterns may be simultaneously valid. Accept their
    // single qdd solution and select the valid contact witness with the lowest
    // equality residual; no sign(v≈0) fallback is used.
    const auto &cluster = clusters.front();
    int chosen = cluster.front();
    for (int index : cluster)
        if (feasible[index].equation_residual < feasible[chosen].equation_residual)
            chosen = index;
    const auto &selected = feasible[chosen];
    result.valid = true;
    result.reason = "ok";
    result.acceleration = selected.acceleration;
    result.contact_force = selected.contact_force;
    result.leg_friction_torque = selected.friction;
    result.leg_mode = selected.mode;
    result.kkt_rank = selected.rank;
    result.max_equation_residual = selected.equation_residual;
    result.max_contact_residual = selected.contact_residual;
    result.max_stick_effort = selected.stick_effort;
    result.max_contact_cone_violation = selected.cone_violation;
    result.viscous_torque.setZero();
    for (int j = 0; j < 4; ++j)
        result.viscous_torque[3 + j] = options.leg_viscous * result.free_velocity[3 + j];
    result.viscous_torque[7] = options.wheel_viscous * result.free_velocity[7];
    result.viscous_torque[8] = options.wheel_viscous * result.free_velocity[8];
    return result;
}

// Aggregate torque residual uses applied joint-friction torque with its
// physical sign (opposing predicted motion): command - viscous + friction
// minus the independent transmitted-wrench observation.
inline SupportActuatorVector ground_friction_aggregate_difference(
    const SupportActuatorVector &command, const SupportQ9 &viscous_torque,
    const Eigen::Matrix<double, 4, 1> &leg_friction_torque,
    const SupportActuatorVector &transmitted_wrench)
{
    SupportActuatorVector aggregate = command;
    for (int j = 0; j < 4; ++j)
        aggregate[j] += leg_friction_torque[j] - viscous_torque[3 + j] - transmitted_wrench[j];
    aggregate[4] -= viscous_torque[7] + transmitted_wrench[4];
    aggregate[5] -= viscous_torque[8] + transmitted_wrench[5];
    return aggregate;
}

}  // namespace bbot_jump
