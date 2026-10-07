#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "bbot_balance_controller/ground_angular_momentum.hpp"

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

bbot_jump::StampedJointKinematics sample(double stamp)
{
    bbot_jump::StampedJointKinematics s;
    s.stamp = stamp;
    s.q << 0.567284, -0.665356, 0.567284, -0.665356;
    s.qdot << 8.44788, -14.7256, 8.44788, -14.7256;
    s.wheel_rates << -3.81819, -3.81819;
    return s;
}
}

int main()
{
    using namespace bbot_jump;

    // Same-stamp production trace fixture at 3.250 s (the next controller
    // record is 3.252 s); this independently anchors the CAD observer scale.
    const auto fixture = sample(3.250);
    double h = 0.0;
    require(flight_planar_angular_momentum(fixture.q, fixture.qdot, 0.79028,
                fixture.wheel_rates, 9.5, h),
            "finite aligned production fixture computes H");
    require(std::abs(h - 1.794375) < 0.01,
            "CAD H agrees with independently audited same-stamp trace");

    JointVector q_golden, qdot_golden;
    q_golden << .65, -.70, .65, -.70;
    qdot_golden << 7.892, -15.201, 7.892, -15.201;
    const Eigen::Vector2d wheel_golden(11.995, 11.995);
    double h_golden = 0.0;
    require(flight_planar_angular_momentum(q_golden, qdot_golden, -.20,
                wheel_golden, 9.5, h_golden),
            "CAD full-system H golden computes with controller-rate sign conversion");
    require(std::abs(h_golden - .19980136) < 2e-4,
            "controller pitch rate +.20 maps to CAD theta rate -.20 and H≈.1998");
    double theta_from_h = 0.0;
    require(flight_pitch_rate_for_total_momentum(q_golden, qdot_golden,
                wheel_golden, h_golden, 9.5, theta_from_h),
            "full-system conserved-H inverse returns a finite base rate");
    require(std::abs(theta_from_h + .20) < 1e-10,
            "full-system inverse preserves golden CAD theta-rate sign");
    FastPlanarMomentumTerms fast_golden;
    require(flight_fast_planar_momentum_terms(
                q_golden, qdot_golden, 9.5, fast_golden),
            "direct rigid-body momentum terms accept the golden state");
    const double h_fast_golden = fast_golden.effective_inertia * (-.20) +
        fast_golden.nonwheel_momentum + .006481 * wheel_golden.sum();
    require(std::abs(h_fast_golden - h_golden) < 2e-10,
            "direct rigid-body sum matches the full CAD momentum golden");
    Eigen::Vector3d fast_com, fast_left, fast_right;
    require(fast_centroidal_positions(q_golden, 9.5, fast_com, fast_left, fast_right),
            "fast CAD positions accept golden state");
    const std::array<double,4> golden_array{q_golden[0],q_golden[1],q_golden[2],q_golden[3]};
    const auto matrix_geometry=centroidal_geometry(golden_array,9.5);
    require((fast_com-matrix_geometry.com).norm()<1e-12 &&
                (fast_left-matrix_geometry.axle_left).norm()<1e-12 &&
                (fast_right-matrix_geometry.axle_right).norm()<1e-12,
            "fast CAD position path equals full centroidal geometry");
    require(std::abs(fast_minimum_wheel_bottom_clearance_yz(
                fast_golden.com_yz,fast_golden.axle_left_yz,fast_golden.axle_right_yz,
                .12,.61,.07,0.0) - minimum_wheel_bottom_clearance(
                matrix_geometry,.12,.61,.07,0.0)) < 1e-12,
            "fast wheel clearance preserves independent left/right CAD geometry");
    double theta_fast = 0.0;
    require(flight_theta_rate_from_fast_terms(
                fast_golden, wheel_golden, h_golden, theta_fast) &&
                std::abs(theta_fast + .20) < 2e-10,
            "direct momentum inverse recovers the golden base rate");
    for (int i = 0; i < 64; ++i) {
        JointVector qr, vr;
        for (int j = 0; j < 4; ++j) {
            qr[j] = 1.35 * std::sin(.71 * (i + 1) * (j + 1));
            vr[j] = 19.0 * std::cos(.37 * (i + 2) * (j + 1));
        }
        const double theta = 2.7 * std::sin(.23 * (i + 1));
        const Eigen::Vector2d wr(18.0 * std::sin(.31 * (i + 1)),
                                  18.0 * std::cos(.29 * (i + 2)));
        double h_matrix = 0.0;
        FastPlanarMomentumTerms fast;
        require(flight_planar_angular_momentum(qr, vr, theta, wr, 9.5, h_matrix) &&
                    flight_fast_planar_momentum_terms(qr, vr, 9.5, fast),
                "asymmetric direct/matrix momentum sample is valid");
        const double h_direct = fast.effective_inertia * theta +
            fast.nonwheel_momentum + .006481 * wr.sum();
        require(std::abs(h_direct - h_matrix) < 2e-9,
                "direct rigid-body H agrees with CAD Schur matrix for asymmetric states");
        Eigen::Vector3d cfast,lfast,rfast;
        const std::array<double,4> qarr{qr[0],qr[1],qr[2],qr[3]};
        const auto gmatrix=centroidal_geometry(qarr,9.5);
        require(fast_centroidal_positions(qr,9.5,cfast,lfast,rfast) &&
                    (cfast-gmatrix.com).norm()<1e-12 &&
                    (lfast-gmatrix.axle_left).norm()<1e-12 &&
                    (rfast-gmatrix.axle_right).norm()<1e-12,
                "fast CAD positions agree on asymmetric states");
        const double c_fast=fast_minimum_wheel_bottom_clearance_yz(
            fast.com_yz,fast.axle_left_yz,fast.axle_right_yz,.23,.55,.07,0.0);
        const double c_matrix=minimum_wheel_bottom_clearance(
            gmatrix,.23,.55,.07,0.0);
        require(std::abs(c_fast-c_matrix)<1e-12,
                "fast separate-wheel contact geometry agrees on asymmetric states");
        double theta_direct = 0.0;
        require(flight_theta_rate_from_fast_terms(fast, wr, h_matrix, theta_direct) &&
                    std::abs(theta_direct - theta) < 2e-9,
                "direct H inverse agrees across asymmetric states");
    }
    double wheel_rate_from_h = 0.0;
    require(flight_symmetric_wheel_rate_for_momentum(q_golden, qdot_golden,
                -.20, h_golden, 9.5, wheel_rate_from_h),
            "full-system H inverse computes a wheel-relative rate");
    require(std::abs(wheel_rate_from_h - 11.995) < 2e-8,
            "H inverse golden preserves the wheel-relative rate sign and value");
    JointVector catch_qdot;
    catch_qdot << .2, .4, .6, .8;
    require(std::abs(flight_ground_catch_shank_rate(catch_qdot, .25) - .75) < 1e-12,
            "ground capture uses half the four leg rates minus controller pitch rate");
    require(!certified_arrest_complete(.059, .060) &&
                certified_arrest_complete(.060, .060) &&
                !certified_arrest_complete(.060, 0.0),
            "certified Tuck cannot interrupt the complete ARREST reference");
    const std::array<double, 4> tuck_q{.2, -.4, .2, -.4};
    const std::array<double, 4> tuck_v{0.0, 0.0, 0.0, 0.0};
    require(certified_tuck_endpoint_matches(tuck_q, tuck_v, tuck_q),
            "measured completed Tuck matches the certified endpoint");
    auto drifted_tuck_q = tuck_q;
    drifted_tuck_q[1] += .006;
    require(!certified_tuck_endpoint_matches(drifted_tuck_q, tuck_v, tuck_q),
            "Tuck endpoint drift cannot reuse stale wheel/contact certification");
    auto moving_tuck_v = tuck_v;
    moving_tuck_v[2] = .11;
    require(!certified_tuck_endpoint_matches(tuck_q, moving_tuck_v, tuck_q),
            "Tuck handoff with residual joint speed fails closed");

    const JointVector flight_q0 = q_golden;
    const JointVector flight_v0 = qdot_golden;
    const Eigen::Vector2d flight_wheel0(11.995, 11.995);
    const std::array<double, 4> tuck_target{.200328, -.472509, .200328, -.472509};
    const std::array<double, 4> landing_target{.432, -.799, .432, -.799};
    const auto budget_start = std::chrono::steady_clock::now();
    const auto envelope = plan_momentum_flight_budget(
        flight_q0, flight_v0, flight_wheel0, h_golden, .2873, 3.0 * M_PI / 180.0,
        .461, .437855, 2.3, 0.0, .060, tuck_target, landing_target, 9.5,
                11.0, 13.0, 450.0, 500.0, 1.52, 1.5708);
    const double budget_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - budget_start).count();
    require(envelope.valid && envelope.rejection_detail == 0 &&
                std::abs(envelope.arrest_duration - .060) < 1e-12 &&
                envelope.tuck_duration >= .19 &&
                envelope.extend_duration >= .11 &&
        envelope.first_contact_time > 0.40 && envelope.first_contact_time <= .501 &&
        envelope.ground_handoff_blend_at_contact > 0.99 &&
        std::abs(envelope.ground_capture_forward_velocity - .45) < 1e-12,
            "state-dependent full-H envelope includes the descending ground-wheel handoff");
    require(envelope.max_abs_wheel_rate <= 28.5714 + 1e-9 &&
                envelope.max_wheel_torque <= 50.0 + 1e-9 &&
                envelope.max_required_hip_torque <= 75.0 + 1e-9 &&
                envelope.max_required_knee_torque <= 60.0 + 1e-9 &&
                envelope.max_wheel_clearance >= .20 &&
                envelope.first_contact_pitch >= 0.0 && envelope.first_contact_pitch <= 5.0*M_PI/180.0 &&
                std::abs(envelope.first_contact_pitch_rate) <= .20,
            "flight envelope preserves wheel limits, clearance, and first-contact attitude");
    const auto insufficient = plan_momentum_flight_budget(
        flight_q0, flight_v0, flight_wheel0, h_golden, .2873, 3.0 * M_PI / 180.0,
        .18, .437855, 2.3, 0.0, .060, tuck_target, landing_target, 9.5,
        11.0, 13.0, 450.0, 500.0, 1.52, 1.5708);
    require(!insufficient.valid && insufficient.rejection_code == 2,
            "insufficient fresh ballistic interval rejects the whole phase plan");

    JointVector reference_q;
    reference_q << 0.7963, -1.073, 0.7963, -1.073;
    double forward_pitch = 0.0;
    require(ground_com_forward_pitch_reference(reference_q, 9.5, 0.03, 0.45,
                forward_pitch),
            "CAD geometry has a valid forward COM pitch reference");
    require(std::abs(forward_pitch - 0.209) < 0.01 && forward_pitch > 0.0,
            "positive COM-forward target has the independently checked pitch sign");
    require(!ground_com_forward_pitch_reference(reference_q, 9.5, 0.03, 0.10,
                forward_pitch),
            "pitch safety cone rejects an unreachable angular reference");
    require(!ground_com_forward_pitch_reference(reference_q, 9.5,
                std::numeric_limits<double>::quiet_NaN(), 0.45, forward_pitch),
            "nonfinite geometry target is rejected");

    GroundMomentumReference momentum_ref;
    const double ieff = flight_pitch_joint_mass_matrix(reference_q, 9.5)(0, 0) +
        2.0 * 0.006481;
    require(ground_h_momentum_pitch_rate_reference(0.10, 1.80, 0.20,
                ieff, 1.20, momentum_ref),
            "same-stamp H produces a valid pitch-rate reference");
    require(momentum_ref.raw_rate > 0.10 &&
                std::abs(momentum_ref.bounded_rate) <= 1.20 &&
                std::abs(momentum_ref.effective_inertia - ieff) < 1e-12,
            "positive excess H advances forward rate within the original guard");
    require(ground_h_momentum_pitch_rate_reference(0.10, 8.0, 0.20,
                ieff, 1.20, momentum_ref) &&
                std::abs(momentum_ref.bounded_rate - 1.20) < 1e-12,
            "large positive momentum demand saturates at the original rate guard");
    require(ground_h_momentum_pitch_rate_reference(0.30, -0.20, 0.20,
                ieff, 1.20, momentum_ref) && momentum_ref.raw_rate < 0.30,
            "negative H remains negative-direction feedback rather than being hidden");
    require(!ground_h_momentum_pitch_rate_reference(0.10,
                std::numeric_limits<double>::quiet_NaN(), 0.20,
                ieff, 1.20, momentum_ref),
            "nonfinite H cannot produce a reference");
    require(smooth_reference_blend(0.0, 0.12) == 0.0 &&
                smooth_reference_blend(0.12, 0.12) == 1.0 &&
                std::abs(smooth_reference_blend(0.06, 0.12) - 0.5) < 1e-12,
            "C1 smooth reference blend has exact endpoints and midpoint");
    require(std::abs(slew_reference(0.05, 0.21, 1.0, 0.005) - 0.055) < 1e-12 &&
                std::abs(slew_reference(-0.20, 0.20, 8.0, 0.005) + 0.16) < 1e-12,
            "reference slew is bounded and preserves either sign");
    require(std::abs(slew_reference(0.05, 1.20, 30.0, 0.005) - 0.20) < 1e-12,
            "H reference follows the approved 30 rad/s^2 bounded transition");

    auto governed = govern_ground_pitch_reference(
        0.138, 0.138, 0.0, 0.35, 1.20, 0.0, 0.005);
    require(governed.valid && governed.pitch <= 0.2180000001 &&
                governed.pitch >= -0.45,
            "CAD attraction cannot lead measured pitch beyond 0.08 rad");
    require(std::abs(governed.rate - (governed.pitch - 0.138) / 0.005) < 1e-12 &&
                std::abs(governed.rate) <= 1.20,
            "rate reference equals the applied angle-reference derivative");
    double ref = governed.pitch, rate = governed.rate;
    double max_lead = ref - 0.138;
    for (int i=1; i<=100; ++i) {
        const double measured = 0.138 + 0.0005 * i;
        governed = govern_ground_pitch_reference(
            ref, measured, 0.10, 0.35, 1.20, rate, 0.005);
        if (!governed.valid)
        {
            std::cerr << "governor invalid at iteration " << i
                      << " ref=" << ref << " measured=" << measured
                      << " rate=" << rate << '\n';
            return 2;
        }
        require(governed.valid && governed.pitch - measured <= 0.0800000001,
                "reference governor preserves the measured-pitch lead envelope");
        require(std::abs(governed.rate) <= 1.20 + 1e-12 &&
                    std::abs(governed.pitch-ref) <= 1.20*0.005+1e-12,
                "reference governor preserves rate and slew bounds");
        max_lead = std::max(max_lead, governed.pitch-measured);
        ref = governed.pitch;
        rate = governed.rate;
    }
    require(max_lead <= 0.0800000001,
            "integrated reference never runs ahead of the measured body");
    require(!govern_ground_pitch_reference(0.0, 0.50, 0.0, 0.0, 0.0, 0.0, .005).valid,
            "reference outside absolute safety cone fails closed");
    require(!govern_ground_pitch_reference(
                0.218, 0.120, -19.6, 0.120, -1.20, 0.0, 0.005).valid,
            "fast measured backward motion with no feasible angle/rate intersection fails closed");
    require(!govern_ground_pitch_reference(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0).valid,
            "invalid time step fails closed");
    require(!govern_ground_pitch_reference(
                0.218, 0.120, 0.0, 0.35, 0.0, 1.20, .005).valid,
            "incompatible lead and rate-slew bounds fail closed instead of misreporting derivative");

    // Optional ground-only pitch floor: sustained negative H/CAD requests
    // decelerate before zero instead of angle-clamping with an inconsistent rate.
    double floor_ref = 0.050;
    double floor_rate = 0.0;
    double min_seen_pitch = floor_ref;
    bool reached_floor = false;
    for (int i = 0; i < 240; ++i) {
        const double measured = std::max(0.0, floor_ref);
        const auto floor_governed = govern_ground_pitch_reference(
            floor_ref, measured, floor_rate, -0.20, -1.20, floor_rate, 0.005,
            0.08, 0.45, 1.20, 30.0, 2.0, 0.0);
        require(floor_governed.valid,
                "negative H and negative CAD requests remain feasible with a zero floor");
        require(floor_governed.pitch >= -1e-12 &&
                    std::abs(floor_governed.rate -
                        (floor_governed.pitch - floor_ref) / 0.005) < 1e-12,
                "floor trajectory stays nonnegative and rate equals discrete angle derivative");
        require(std::abs(floor_governed.rate) <= 1.20 + 1e-12 &&
                    std::abs(floor_governed.rate - floor_rate) <= 30.0 * 0.005 + 1e-12,
                "floor trajectory preserves original rate and 30 rad/s^2 slew limits");
        min_seen_pitch = std::min(min_seen_pitch, floor_governed.pitch);
        floor_ref = floor_governed.pitch;
        floor_rate = floor_governed.rate;
        if (floor_ref <= 1e-10 && std::abs(floor_rate) <= 1e-10) {
            reached_floor = true;
            break;
        }
    }
    require(reached_floor && min_seen_pitch >= -1e-12,
            "persistent negative request reaches and holds the floor without crossing it");
    auto floor_rise = govern_ground_pitch_reference(
        floor_ref, 0.0, floor_rate, 0.10, 0.50, floor_rate, 0.005,
        0.08, 0.45, 1.20, 30.0, 2.0, 0.0);
    require(floor_rise.valid && floor_rise.pitch >= floor_ref && floor_rise.rate >= 0.0 &&
                std::abs(floor_rise.rate - (floor_rise.pitch-floor_ref)/0.005) < 1e-12,
            "positive request can smoothly move away from the stopped floor");
    require(!govern_ground_pitch_reference(
                .001, 0.0, -.50, -.2, -1.20, -.50, .005,
                .08, .45, 1.20, 30.0, 2.0, 0.0).valid,
            "floor cannot repair an already infeasible downward reference rate");
    require(!govern_ground_pitch_reference(
                .05, .05, 0.0, 0.0, -1.20, 0.0, .005,
                .08, .45, 1.20, 30.0, 2.0,
                std::numeric_limits<double>::quiet_NaN()).valid &&
            !govern_ground_pitch_reference(
                .05, .05, 0.0, 0.0, -1.20, 0.0, .005,
                .08, .45, 1.20, 30.0, 2.0, .46).valid,
            "NaN and out-of-safety-cone floors are rejected");
    require(!govern_ground_pitch_reference(
                -.01, 0.0, 0.0, 0.0, 0.0, 0.0, .005,
                .08, .45, 1.20, 30.0, 2.0, 0.0).valid,
            "reference already below the requested floor fails closed");

    JointKinematicsHistory history;
    require(history.push(fixture), "history accepts a complete finite frame");
    StampedAngularMomentumObserver observer;
    require(observer.update(3.255, 3.250, -0.79028, history, 9.5),
            "observer accepts exact IMU/joint stamp");
    require(std::abs(observer.momentum() - h) < 1e-12,
            "observer holds exact CAD momentum");
    require(std::abs(observer.raw_pitch_rate() + 0.79028) < 1e-12,
            "observer retains the raw rate paired with the H sample");
    require(observer.update(3.265, 3.260, -0.80, history, 9.5),
            "new IMU without joint upper frame holds only prior fresh estimate");
    require(std::abs(observer.momentum() - h) < 1e-12,
            "pending joint frame does not synthesize an estimate");
    require(std::abs(observer.raw_pitch_rate() + 0.79028) < 1e-12,
            "pending new IMU cannot be mixed with the old H sample");
    require(!observer.update(3.271, 3.250, -0.79028, history, 9.5) ||
                !observer.valid(3.271),
            "stale source is rejected");
    require(!observer.valid(3.271), "stale rejection clears estimate");

    require(history.push(sample(3.270)), "new complete frame restores history");
    require(observer.update(3.272, 3.270, -0.80, history, 9.5),
            "two matching channels recover after stale reset");
    require(!observer.update(3.269, 3.269, -0.80, history, 9.5),
            "IMU stamp rollback is rejected");
    require(!observer.valid(3.269), "rollback clears held estimate");

    const double applied = thrust_ground_angular_momentum_correction(
        true, true, false, true, true, true, observer, 3.272, 0.20, 8.0, 8.0);
    // observer was deliberately invalidated by the rollback above.
    require(applied == 0.0, "invalid observer cannot request torque");
    require(thrust_positive_angular_momentum_feedback(1.70, 0.20, 8.0, 8.0) == 8.0,
            "positive excess saturates at the additional per-hip budget");
    require(std::abs(thrust_positive_angular_momentum_feedback(0.30, 0.20, 8.0, 8.0) - 0.8) < 1e-12,
            "feedback tapers as H approaches the target");
    require(thrust_positive_angular_momentum_feedback(0.19, 0.20, 8.0, 8.0) == 0.0,
            "positive-only feedback does not reverse a low-H jump");

    auto recovered = sample(3.280);
    require(history.push(recovered), "history accepts next finite frame");
    require(observer.update(3.281, 3.280, -0.80, history, 9.5),
            "observer can be reseeded after rollback");
    require(thrust_ground_angular_momentum_correction(
                true, true, false, true, true, true, observer, 3.281,
                0.20, 8.0, 8.0) > 0.0,
            "valid gated grounded excess produces correction");
    require(thrust_ground_angular_momentum_correction(
                true, false, false, true, true, true, observer, 3.281,
                0.20, 8.0, 8.0) == 0.0,
            "Position mode cannot receive correction");
    require(thrust_ground_angular_momentum_correction(
                true, true, true, true, true, true, observer, 3.281,
                0.20, 8.0, 8.0) == 0.0,
            "pending Effort switch cannot receive correction");
    require(thrust_ground_angular_momentum_correction(
                true, true, false, false, true, true, observer, 3.281,
                0.20, 8.0, 8.0) == 0.0,
            "closed THRUST gate cannot receive correction");
    require(thrust_ground_angular_momentum_correction(
                true, true, false, true, false, true, observer, 3.281,
                0.20, 8.0, 8.0) == 0.0,
            "airborne robot cannot receive ground correction");
    require(thrust_ground_angular_momentum_correction(
                true, true, false, true, true, false, observer, 3.281,
                0.20, 8.0, 8.0) == 0.0,
            "stale IMU cannot receive correction");

    std::cout << "ground_angular_momentum checks passed; fixture H=" << h
              << " envelope_plan_ms=" << budget_ms
              << " peak_hip_torque=" << envelope.max_required_hip_torque
              << " peak_knee_torque=" << envelope.max_required_knee_torque << '\n';
    return 0;
}
