#include <cassert>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include "bbot_balance_controller/flight_landing_geometry.hpp"
#include "bbot_balance_controller/flight_trajectory.hpp"
#include "bbot_balance_controller/ground_launch_reference.hpp"
#include "bbot_kinematics/kinematics.hpp"

int main()
{
    bbot_kinematics::RobotParams params;
    constexpr double mass = 9.5;
    constexpr double pitch = 3.0 * M_PI / 180.0;

    const auto landing = bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.60, pitch, 0.030, mass);
    assert(landing.valid);
    assert(std::abs(landing.com_forward - 0.030) < 1e-8);
    assert(landing.target_x > 0.0 && landing.target_x < 0.14);
    assert(std::abs(landing.hip) < 1.57 && std::abs(landing.knee) < 1.57);
    assert(std::abs(landing.shank_angle) < 0.75);
    assert(params.l1 * std::cos(landing.shank_angle) >= 0.20);
    bbot_kinematics::Kinematics kin(params);
    assert(std::abs(kin.calculate_com_height(pitch, landing.hip, landing.knee) - 0.60) < 1e-6);
    assert(std::abs(landing.fk_horizontal_offset - (-0.01137221 - landing.target_x)) < 1e-6);

    const auto retract = bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.52, pitch, 0.030, mass);
    assert(retract.valid);
    assert(std::abs(retract.com_forward - 0.030) < 1e-8);
    assert(std::abs(retract.hip) < 1.57 && std::abs(retract.knee) < 1.57);
    assert(std::abs(retract.shank_angle) < 0.75);
    assert(std::abs(kin.calculate_com_height(pitch, retract.hip, retract.knee) - 0.52) < 1e-6);

    double max_shank = 0.0;
    double min_knee_clearance = 1.0;
    double com_z_touch = 0.0;
    double com_z_buffer = 0.0;
    bool found_centered_geometry = false;
    bool centered_geometry_became_unreachable_again = false;
    for (int i = 0; i <= 56; ++i) {
        const double h = 0.32 + 0.005 * i;
        const auto pose = bbot_jump::solve_landing_pose_for_com_forward(
            params, h, pitch, 0.030, mass);
        assert(pose.valid);
        max_shank = std::max(max_shank, std::abs(pose.shank_angle));
        min_knee_clearance = std::min(min_knee_clearance,
                                      params.l1 * std::cos(pose.shank_angle));
        const std::array<double, 4> q{pose.hip, pose.knee, pose.hip, pose.knee};
        const auto geometry = bbot_jump::centroidal_geometry(q, mass);
        const double com_z = kin.calculate_com_height(pitch, pose.hip, pose.knee) +
                             bbot_jump::rotate_about_hip(-pitch, geometry.com).z();
        if (i == 0) com_z_buffer = com_z;
        if (i == 56) com_z_touch = com_z;
        const auto centered = bbot_jump::solve_landing_pose_for_com_forward(
            params, h, pitch, 0.0, mass);
        if (centered.valid)
        {
            if (centered_geometry_became_unreachable_again)
                assert(false && "centered geometry must remain feasible above its threshold");
            found_centered_geometry = true;
            assert(std::abs(centered.com_forward) < 1e-6);
            assert(std::abs(centered.shank_angle) < 0.75);
            assert(params.l1 * std::cos(centered.shank_angle) >= 0.20);
        }
        else
        {
            // Low, compressed poses stay on the forward support target until
            // a zero-COM-forward shape is physically reachable.
            if (found_centered_geometry) centered_geometry_became_unreachable_again = true;
        }
        if (std::abs(h - 0.32) < 1e-9) assert(!centered.valid);
    }
    assert(max_shank < 0.75);
    assert(found_centered_geometry);
    assert(min_knee_clearance >= 0.20);
    assert(com_z_touch - com_z_buffer >= 0.167);
    std::cout << "PASS: landing sweep max_shank=" << max_shank
              << " min_knee_clearance=" << min_knee_clearance
              << " COM_stroke=" << (com_z_touch - com_z_buffer) << " m\n";

    const auto legacy_zero = bbot_jump::landing_ik_target(params, 0.60, pitch, 0.0);
    const double legacy_forward = bbot_jump::landing_com_forward_from_axle(
        {legacy_zero[0], legacy_zero[1], legacy_zero[0], legacy_zero[1]}, pitch, mass);
    assert(legacy_forward < 0.0);  // Old back-bias geometry leaves COM behind the axle.

    const auto infeasible = bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.60, pitch, 1.0, mass);
    assert(!infeasible.valid);
    const auto invalid = bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.60, pitch, 0.030, 0.0);
    assert(!invalid.valid);
    assert(!bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.95, pitch, 0.030, mass).valid);
    assert(!bbot_jump::landing_ik_reachable(params, 0.60, pitch, 0.20));
    const auto legacy_observe_pose = bbot_jump::solve_landing_pose_for_com_forward(
        params, 0.69, 0.085, 0.005, mass);
    assert(legacy_observe_pose.valid);

    // A low ground-support shape may exceed the landing-only shank cone while
    // remaining reachable and retaining the actual 20 cm knee clearance.
    for (double h : {0.34, 0.40, 0.475})
    {
        const auto ground_pose = bbot_jump::solve_leg_pose_for_com_forward(
            params, h, 0.05, 0.010, mass);
        const auto landing_pose = bbot_jump::solve_landing_pose_for_com_forward(
            params, h, 0.05, 0.010, mass);
        assert(ground_pose.valid);
        assert(std::abs(ground_pose.com_forward - 0.010) < 1e-8);
        assert(std::abs(ground_pose.hip) <= 1.52);
        assert(std::abs(ground_pose.knee) <= 1.5708);
        assert(params.l1 * std::cos(ground_pose.shank_angle) >= 0.20);
        assert(!landing_pose.valid);
    }

    // Ground launch moves the support geometry while keeping the leg IK on
    // the same height trajectory. Verify both the first Position jump and a
    // later Effort-supported jump without treating this as a torque proof.
    assert(bbot_jump::ground_launch_blend(0.0) == 0.0);
    assert(bbot_jump::ground_launch_blend(1.0) == 1.0);
    assert(bbot_jump::ground_launch_blend(-0.1) == 0.0);
    assert(bbot_jump::ground_launch_blend(1.1) == 1.0);
    assert(std::isnan(bbot_jump::ground_launch_blend(
        std::numeric_limits<double>::quiet_NaN())));
    for (double u : {0.0, 1.0})
    {
        const double eps = 1e-5;
        const double near = bbot_jump::ground_launch_blend(u == 0.0 ? eps : 1.0 - eps);
        assert(std::abs(near - u) < 1e-12);
    }

    for (double desired_com_forward : {0.030, 0.010})
    for (const auto & [duration, samples] :
         {std::pair<double, int>{0.50, 501}, {0.65, 651}})
    {
        bbot_jump::QuinticTrajectory height_trajectory;
        height_trajectory.init(0.0, duration, 0.50, 0.0, 0.0,
                              0.34, 0.0, 0.0);
        std::array<std::array<double, 2>, 651> q_samples{};
        std::array<double, 651> heights{};
        for (int i = 0; i < samples; ++i)
        {
            const double t = duration * static_cast<double>(i) / (samples - 1);
            double h = 0.0, hdot = 0.0, hddot = 0.0;
            height_trajectory.evaluate(t, h, hdot, hddot);
            const double blend = bbot_jump::ground_launch_blend(t / duration);
            const auto pose = bbot_jump::ground_launch_pose(
                params, h, 0.05, desired_com_forward, mass, blend);
            assert(pose.valid);
            assert(std::abs(kin.calculate_com_height(
                       blend * 0.05, pose.q[0], pose.q[1]) - h) < 1e-6);
            assert(std::abs(pose.blend - blend) < 1e-15);
            assert(std::abs(pose.q[0]) <= 1.52);
            assert(std::abs(pose.q[1]) <= 1.5708);
            assert(std::isfinite(pose.target_x));
            assert(std::isfinite(pose.com_forward));
            const double expected_com_forward = bbot_jump::landing_com_forward_from_axle(
                {pose.q[0], pose.q[1], pose.q[0], pose.q[1]}, 0.05, mass);
            assert(std::abs(pose.com_forward - expected_com_forward) < 1e-12);
            const double ground_shank = std::atan2(0.28210870, 0.19553796) +
                pose.q[0] + pose.q[1] - blend * 0.05;
            assert(params.l1 * std::cos(ground_shank) >= 0.20);
            heights[i] = h;
            q_samples[i] = pose.q;
        }

        // Blend zero exactly retains the original vertical IK; blend one
        // reaches the fixed-pitch, forward-COM endpoint at the same height.
        const auto first = bbot_jump::ground_launch_pose(
            params, heights[0], 0.05, desired_com_forward, mass, 0.0);
        const auto vertical = bbot_jump::landing_ik_target(
            params, heights[0], 0.0, 0.0);
        assert(first.valid);
        assert(first.q == vertical);
        const auto original_ik = kin.inverse_kinematics(heights[0], 0.0);
        assert(std::abs(first.q[0] - original_ik.theta_hip) < 1e-12);
        assert(std::abs(first.q[1] - original_ik.theta_knee) < 1e-12);
        const auto last = bbot_jump::ground_launch_pose(
            params, heights[samples - 1], 0.05, desired_com_forward, mass, 1.0);
        assert(last.valid);
        assert(std::abs(last.com_forward - desired_com_forward) < 1e-8);
        const auto endpoint = bbot_jump::solve_leg_pose_for_com_forward(
            params, heights[samples - 1], 0.05, desired_com_forward, mass);
        assert(endpoint.valid);
        assert(std::abs(last.target_x - endpoint.target_x) < 1e-12);
        assert(std::abs(last.q[0] - endpoint.hip) < 1e-12);
        assert(std::abs(last.q[1] - endpoint.knee) < 1e-12);

        const double dt = duration / (samples - 1);
        double max_hip_speed = 0.0, max_knee_speed = 0.0;
        double max_hip_accel = 0.0, max_knee_accel = 0.0;
        for (int i = 1; i + 1 < samples; ++i)
        {
            for (std::size_t j = 0; j < 2; ++j)
            {
                const double velocity =
                    (q_samples[i + 1][j] - q_samples[i - 1][j]) / (2.0 * dt);
                const double acceleration =
                    (q_samples[i + 1][j] - 2.0 * q_samples[i][j] +
                     q_samples[i - 1][j]) / (dt * dt);
                if (j == 0)
                {
                    max_hip_speed = std::max(max_hip_speed, std::abs(velocity));
                    max_hip_accel = std::max(max_hip_accel, std::abs(acceleration));
                }
                else
                {
                    max_knee_speed = std::max(max_knee_speed, std::abs(velocity));
                    max_knee_accel = std::max(max_knee_accel, std::abs(acceleration));
                }
            }
        }
        assert(max_hip_speed <= 3.0);
        assert(max_knee_speed <= 3.0);
        assert(max_hip_accel <= 450.0);
        assert(max_knee_accel <= 500.0);
        assert(std::abs(q_samples[1][0] - q_samples[0][0]) < 1e-5);
        assert(std::abs(q_samples[1][1] - q_samples[0][1]) < 1e-5);
        assert(std::abs(q_samples[samples - 1][0] -
                        q_samples[samples - 2][0]) < 1e-5);
        assert(std::abs(q_samples[samples - 1][1] -
                        q_samples[samples - 2][1]) < 1e-5);
        std::cout << "ground launch trajectory COMforward=" << desired_com_forward
                  << " duration=" << duration
                  << " max_speed=" << max_hip_speed << "/" << max_knee_speed
                  << " max_accel=" << max_hip_accel << "/" << max_knee_accel
                  << " rad/s, rad/s^2\n";
    }

    assert(!bbot_jump::ground_launch_pose(
        params, 0.34, 0.05, 0.030, mass,
        std::numeric_limits<double>::quiet_NaN()).valid);
    assert(!bbot_jump::ground_launch_pose(params, 0.34, 0.05, 0.030, mass, -0.01).valid);
    assert(!bbot_jump::ground_launch_pose(params, 0.34, 0.05, 0.030, mass, 1.01).valid);
    assert(!bbot_jump::ground_launch_pose(params, 0.95, 0.05, 0.030, mass, 1.0).valid);
    assert(!bbot_jump::ground_launch_pose(params, 0.34, 0.05, 0.030, 0.0, 1.0).valid);
    return 0;
}
