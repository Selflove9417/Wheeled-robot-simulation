#include "bbot_balance_controller/flight_trajectory.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace bbot_jump;

void require(bool ok, const char * message)
{
    if (!ok) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char * message)
{
    require(std::abs(actual - expected) <= tolerance, message);
}

// Independent position-only quintic Hermite basis. Numerical differentiation
// below does not share coefficient assembly or analytic velocity evaluation.
double hermite_position(double s, double duration, double q0, double v0,
                        double a0, double q1, double v1, double a1)
{
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    return q0 * (1 - 10*s3 + 15*s4 - 6*s5) +
        duration * v0 * (s - 6*s3 + 8*s4 - 3*s5) +
        duration * duration * a0 * .5 * (s2 - 3*s3 + 3*s4 - s5) +
        q1 * (10*s3 - 15*s4 + 6*s5) +
        duration * v1 * (-4*s3 + 7*s4 - 3*s5) +
        duration * duration * a1 * .5 * (s3 - 2*s4 + s5);
}

double independent_peak_speed(double q0, double v0, double a0, double q1,
                              double duration)
{
    QuinticTrajectory trajectory;
    trajectory.init(0.0, duration, q0, v0, a0, q1, 0.0, 0.0);
    double peak = 0.0;
    constexpr double ds = 1e-6;
    for (int i = 1; i < 20000; ++i) {
        const double s = i / 20000.0;
        const double numerical =
            (hermite_position(s+ds, duration, q0, v0, a0, q1, 0, 0) -
             hermite_position(s-ds, duration, q0, v0, a0, q1, 0, 0)) /
            (2 * ds * duration);
        double q, v, a;
        trajectory.evaluate(s * duration, q, v, a);
        near(v, numerical, 2e-7, "velocity differs from independent Hermite derivative");
        peak = std::max(peak, std::abs(numerical));
    }
    return peak;
}

int main()
{
    const std::array<double, 4> rest{};

    // v6.13, 12.581 s: L=0.30 m tuck IK at pitch .0871677-.038,
    // wheel radius .07, CAD thigh/shank .300/.34325 and target_x=0.
    // The endpoint constants are independently obtained by triangle geometry.
    const std::array<double, 4> failed_q{.988561, -1.27599, .988561, -1.27599};
    const std::array<double, 4> failed_v{2.46842, 1.87054, 2.46842, 1.87054};
    const std::array<double, 4> full_tuck{
        -.25237138341975224, .32883257405457744,
        -.25237138341975224, .32883257405457744};
    // Current L_TOUCH=0.50 m, pitch=.093, and the 12.665 s log's
    // target_x=.0679332 touchdown IK.
    const std::array<double, 4> landing{
        .17780705756364037, -.3990575527575557,
        .17780705756364037, -.3990575527575557};
    require(!flight_round_trip_admissible(
        failed_q, failed_v, rest, full_tuck, landing, .10, .09, .40),
        "v6.13 forced tuck accepted despite excessive joint motion");
    const double hip_peak = independent_peak_speed(failed_q[0], failed_v[0], 0,
                                                    full_tuck[0], .10);
    const double knee_peak = independent_peak_speed(failed_q[1], failed_v[1], 0,
                                                     full_tuck[1], .10);
    require(hip_peak > 24.0 && hip_peak < 24.5 &&
            knee_peak > 29.0 && knee_peak < 29.6,
            "independent replay does not reproduce logged tuck speed peaks");

    // With the actual L_RETRACT and L_TOUCH endpoints, even an already tucked
    // robot cannot complete the fixed .09 s deployment within the existing
    // speed budget. This is a planning limitation, not a failing tuck sensor.
    require(!flight_round_trip_admissible(full_tuck, rest, rest, full_tuck,
                                          landing, .10, .09, .40),
            "fixed deployment duration wrongly accepts actual leg geometry");
    const double deployment_hip_peak = independent_peak_speed(
        full_tuck[0], 0, 0, landing[0], .09);
    const double deployment_knee_peak = independent_peak_speed(
        full_tuck[1], 0, 0, landing[1], .09);
    near(deployment_hip_peak, 8.96205085382068, 2e-7,
         "actual hip deployment peak differs from independent prediction");
    near(deployment_knee_peak, 15.164377641919442, 2e-7,
         "actual knee deployment peak differs from independent prediction");

    // Actual geometry can pass with a modest tuck from a nearby starting pose
    // and sufficient deployment time. The controller's .10 s tuck duration is
    // retained; .14 s deployment exceeds the knee's .13648 s speed minimum.
    auto near_tuck = full_tuck;
    near_tuck[0] += .02; near_tuck[2] += .02;
    near_tuck[1] -= .03; near_tuck[3] -= .03;
    require(flight_segment_admissible(near_tuck, rest, rest, full_tuck, .10),
            "actual .30 m tuck endpoint rejected for a feasible nearby pose");
    require(flight_round_trip_admissible(near_tuck, rest, rest, full_tuck,
                                         landing, .10, .14, .25),
            "actual tuck and touchdown geometry rejected despite sufficient time");

    // Independent replay of the 12.581 s body ballistic timing used by the
    // controller: z=.7965, vz=1.28812, ground offset approximately zero,
    // landing body height .50, deployment margin .055 and extra margin .020.
    // This body trajectory estimate is a planning input, not a COM jump height.
    const double remaining = (1.28812 + std::sqrt(
        1.28812*1.28812 + 2*9.81*(.7965-.50))) / 9.81;
    const double available = remaining - .055 - .020;
    near(available, .3350360988771507, 1e-12,
         "logged body timing replay changed");
    require(!plan_flight_round_trip(failed_q, failed_v, rest, full_tuck,
                                    landing, .10, .09, available).valid,
            "latest failed tuck accepted within its remaining flight time");

    const auto nearby_plan = plan_flight_round_trip(near_tuck, rest, rest,
        full_tuck, landing, .10, .09, .25);
    require(nearby_plan.valid && nearby_plan.tuck_duration == .10 &&
            nearby_plan.extend_duration >= .14 &&
            nearby_plan.tuck_duration + nearby_plan.extend_duration <= .25,
            "planner cannot allocate feasible time for actual nearby geometry");
    require(flight_round_trip_admissible(near_tuck, rest, rest, full_tuck, landing,
        nearby_plan.tuck_duration, nearby_plan.extend_duration, .25),
        "returned nearby plan violates original motion budgets");

    auto moderate_q = full_tuck;
    moderate_q[0] += .50; moderate_q[2] += .45;
    moderate_q[1] -= .60; moderate_q[3] -= .55;
    require(!flight_segment_admissible(moderate_q, rest, rest, full_tuck, .10),
            "moderate example does not actually need extra tuck time");
    const auto moderate_plan = plan_flight_round_trip(moderate_q, rest, rest,
        full_tuck, landing, .10, .09, .30);
    require(moderate_plan.valid && moderate_plan.tuck_duration > .10 &&
            moderate_plan.extend_duration >= .14 &&
            moderate_plan.tuck_duration + moderate_plan.extend_duration <= .30,
            "planner cannot extend a moderately over-budget nominal tuck");
    near((moderate_plan.tuck_duration-.10)/.005,
         std::round((moderate_plan.tuck_duration-.10)/.005), 1e-10,
         "tuck time search left the control-period grid");
    near(moderate_plan.extend_duration/.005,
         std::round(moderate_plan.extend_duration/.005), 1e-10,
         "deployment time is not rounded to the control-period grid");
    require(flight_round_trip_admissible(moderate_q, rest, rest, full_tuck, landing,
        moderate_plan.tuck_duration, moderate_plan.extend_duration, .30),
        "returned moderate plan violates original motion budgets");
    require(!plan_flight_round_trip(near_tuck, rest, rest, full_tuck,
                                    landing, .10, .09, .23).valid,
            "planner uses time reserved for landing");
    require(!plan_flight_round_trip(moderate_q, rest, rest, full_tuck,
                                    landing, .10, .09, .25).valid,
            "planner accepts moderate tuck without its required extra time");

    // A modest actual leg motion can complete both segments within budget.
    const std::array<double, 4> q{.2, -.4, .18, -.38};
    const std::array<double, 4> v{.2, -.3, -.15, .25};
    const std::array<double, 4> a{1., -2., -.8, 1.5};
    const std::array<double, 4> mid{.1, -.25, .09, -.24};
    const std::array<double, 4> end{.22, -.4, .2, -.39};
    require(flight_round_trip_admissible(q, v, a, mid, end, .20, .20, .40),
            "feasible two-segment flight rejected");
    for (std::size_t i = 0; i < q.size(); ++i) {
        QuinticTrajectory tuck, deploy;
        tuck.init(2., .20, q[i], v[i], a[i], mid[i], 0, 0);
        deploy.init(2.20, .20, mid[i], 0, 0, end[i], 0, 0);
        double tq, tv, ta, dq, dv, da;
        tuck.evaluate(2., tq, tv, ta);
        near(tq, q[i], 1e-12, "initial position changed");
        near(tv, v[i], 1e-12, "initial velocity changed");
        near(ta, a[i], 1e-12, "initial acceleration changed");
        tuck.evaluate(2.20, tq, tv, ta);
        deploy.evaluate(2.20, dq, dv, da);
        near(tq, dq, 1e-12, "segment position boundary is discontinuous");
        near(tv, dv, 1e-11, "segment velocity boundary is discontinuous");
        near(ta, da, 1e-10, "segment acceleration boundary is discontinuous");
        deploy.evaluate(2.40, dq, dv, da);
        near(dq, end[i], 1e-12, "deployment endpoint was clipped");
        near(dv, 0, 1e-11, "deployment terminal velocity is not zero");
        near(da, 0, 1e-10, "deployment terminal acceleration is not zero");
        const double peak = independent_peak_speed(q[i], v[i], a[i], mid[i], .20);
        require(peak < 2., "independent feasible trajectory speed is excessive");

        auto bad_v = v;
        bad_v[i] = (i % 2 == 0) ? 7.51 : 10.01;
        require(!flight_segment_admissible(q, bad_v, a, mid, .20),
                "one side's excess initial speed was hidden by averaging");
        auto bad_a = a;
        bad_a[i] = (i % 2 == 0) ? 240.01 : 320.01;
        require(!flight_segment_admissible(q, v, bad_a, mid, .20),
                "one joint's excess acceleration was ignored");
        auto bad_mid = mid;
        bad_mid[i] = 1.451;
        require(!flight_round_trip_admissible(q, v, a, bad_mid, end, .20, .20, .40),
                "tuck position limit ignored");
        auto bad_end = end;
        bad_end[i] = -1.451;
        require(!flight_round_trip_admissible(q, v, a, mid, bad_end, .20, .20, .40),
                "deployment position limit ignored");
    }

    // Check the second segment's interior speed, not only the tuck and final q.
    const std::array<double, 4> zero{}, far{1., 0., 0., 0.};
    require(!flight_round_trip_admissible(zero, zero, zero, zero, far, .10, .10, .20),
            "over-budget deployment accepted after stationary tuck");
    // Acceleration can violate its budget even when speed and positions fit.
    const std::array<double, 4> tiny{.01, 0., 0., 0.};
    require(!flight_segment_admissible(zero, zero, zero, tiny, .01),
            "interior acceleration limit ignored");
    // For a short move acceleration, rather than speed, determines the
    // minimum deployment time: sqrt((10/sqrt(3))*.1/240)=.049047 s.
    const std::array<double, 4> acceleration_limited_end{.10, 0., 0., 0.};
    const auto acceleration_plan = plan_flight_round_trip(zero, zero, zero,
        zero, acceleration_limited_end, .005, .005, .10);
    require(acceleration_plan.valid && acceleration_plan.extend_duration == .05,
            "planner ignored the exact rest-to-rest acceleration peak");
    require(!flight_segment_admissible(zero, zero, zero,
        acceleration_limited_end, .045),
        "one grid step below the acceleration-limited duration should fail");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {-1., 0., 1e-5, 1e-4, nan, inf}) {
        require(!flight_segment_admissible(q, v, a, mid, bad),
                "invalid single-segment duration accepted");
        require(!flight_round_trip_admissible(q, v, a, mid, end, bad, .20, .50),
                "invalid tuck duration accepted");
        require(!flight_round_trip_admissible(q, v, a, mid, end, .20, bad, .50),
                "invalid deployment duration accepted");
        require(!plan_flight_round_trip(q, v, a, mid, end, bad, .20, .50).valid &&
                !plan_flight_round_trip(q, v, a, mid, end, .20, bad, .50).valid,
                "planner accepted invalid nominal duration");
    }
    for (double bad : {-1., .399, nan, inf}) {
        require(!flight_round_trip_admissible(q, v, a, mid, end, .20, .20, bad),
                "insufficient or invalid remaining flight time accepted");
    }
    for (std::size_t i = 0; i < q.size(); ++i) {
        for (double bad : {nan, inf}) {
            auto bad_q = q; bad_q[i] = bad;
            auto bad_v = v; bad_v[i] = bad;
            auto bad_a = a; bad_a[i] = bad;
            auto bad_mid = mid; bad_mid[i] = bad;
            auto bad_end = end; bad_end[i] = bad;
            require(!flight_round_trip_admissible(bad_q, v, a, mid, end, .20, .20, .40) &&
                    !flight_round_trip_admissible(q, bad_v, a, mid, end, .20, .20, .40) &&
                    !flight_round_trip_admissible(q, v, bad_a, mid, end, .20, .20, .40) &&
                    !flight_round_trip_admissible(q, v, a, bad_mid, end, .20, .20, .40) &&
                    !flight_round_trip_admissible(q, v, a, mid, bad_end, .20, .20, .40),
                    "nonfinite joint boundary accepted");
            require(!plan_flight_round_trip(bad_q, v, a, mid, end, .20, .20, .50).valid &&
                    !plan_flight_round_trip(q, bad_v, a, mid, end, .20, .20, .50).valid &&
                    !plan_flight_round_trip(q, v, bad_a, mid, end, .20, .20, .50).valid &&
                    !plan_flight_round_trip(q, v, a, bad_mid, end, .20, .20, .50).valid &&
                    !plan_flight_round_trip(q, v, a, mid, bad_end, .20, .20, .50).valid,
                    "planner accepted nonfinite joint boundary");
        }
    }
    for (double bad : {-1., 0., nan, inf}) {
        require(!plan_flight_round_trip(q, v, a, mid, end, .20, .20, bad).valid,
                "planner accepted invalid remaining time");
    }
    require(!plan_flight_round_trip(q, v, a, mid, end, .61, .20, 100.).valid &&
            !plan_flight_round_trip(q, v, a, mid, end, .20, .61, 100.).valid,
            "planner ignored the existing .60 s flight timeout");
    const std::array<double, 4> max_position{1.45, 1.45, 1.45, 1.45};
    const std::array<double, 4> min_position{-1.45, -1.45, -1.45, -1.45};
    require(!plan_flight_round_trip(max_position, rest, rest, min_position,
        min_position, .005, .005, 100.).valid,
        "planner searched past the flight timeout for an infeasible long motion");
    std::cout << "PASS: v6.13 tuck rejected (independent speed peaks " << hip_peak
              << "/" << knee_peak << " rad/s), feasible C2 round trip, per-joint "
              << "budgets, bounded time allocation (nearby " << nearby_plan.tuck_duration
              << "+" << nearby_plan.extend_duration << " s, moderate "
              << moderate_plan.tuck_duration << "+" << moderate_plan.extend_duration
              << " s), deployment checks and invalid inputs\n";
}
