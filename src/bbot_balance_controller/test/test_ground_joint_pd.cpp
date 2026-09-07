#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "bbot_balance_controller/ground_joint_pd.hpp"

using namespace bbot_jump;

namespace
{
void require(bool value, const char * message)
{
    if (!value) throw std::runtime_error(message);
}

struct Result { double peak_error, final_error, peak_torque; };

// Frozen local hip-axis/contact model, not a Gazebo landing simulation.
// Contact adds a positive-semidefinite inertia to the free-base Schur complement.
// Keep the actual plant inertia independent of the conservative controller model.
// The two hip joints rotate the body in the same sense in this constrained mode.
Result simulate(bool implicit, const JointVector & reference,
                const JointVector & initial_error, const JointVector & initial_v,
                double inertia_scale, int sample_ms, double contact_inertia, double gyro_alpha=0.15)
{
    const JointVector b(0.5, 0, 0.5, 0);
    const JointMatrix nominal = flight_joint_inertia(reference, 9.5);
    const JointMatrix actual = inertia_scale * nominal +
        contact_inertia * b * b.transpose();
    const auto dynamics = actual.ldlt();
    const JointVector kp(25, 45, 25, 45), kd(3.5, 6, 3.5, 6);
    const JointVector limits(75, 60, 75, 60);
    // This constant support exactly opposes an independent static plant load.
    // Adding it outside the feedback solve must preserve this equilibrium.
    const JointVector support(7, -22, 7, -22);
    JointVector q = reference + initial_error, v = initial_v, qs = q, vs = v;
    JointVector pending = support, applied = support, previous = support;
    double sampled_rate = b.dot(v);
    double peak_error = initial_error.norm(), peak_torque = 0;
    for (int ms = 0; ms < 4000; ++ms) {
        if (ms % sample_ms == 0) {
            applied = pending;
            qs = q; vs = v;
            // Include filtered body-rate feedback, rather than an ideal gyro.
            sampled_rate = gyro_alpha * b.dot(v) + (1.0-gyro_alpha) * sampled_rate;
        }
        if (ms % 5 == 0) {
            const double pitch_error = b.dot(qs - reference);
            const double body_per_hip = std::clamp(
                -0.5 * (55 * pitch_error + 15 * sampled_rate), -20.0, 20.0);
            const JointVector attitude(body_per_hip, 0, body_per_hip, 0);
            JointVector request = support + kp.cwiseProduct(reference - qs) -
                kd.cwiseProduct(vs) + attitude;
            if (implicit) {
                const double horizon = std::clamp(
                    0.002 * sample_ms + 0.001 * (ms % sample_ms), 0.025, 0.060);
                request = support + discrete_ground_feedback(nominal, qs, vs,
                    reference, JointVector::Zero(), kp, kd, attitude,
                    sampled_rate, 55, 15, horizon);
            }
            request = request.cwiseMax(-limits).cwiseMin(limits);
            if (!implicit) request = previous +
                (request - previous).cwiseMax(-15.0).cwiseMin(15.0);
            previous = request; pending = request;
            peak_torque = std::max(peak_torque, pending.cwiseAbs().maxCoeff());
        }
        // 0.5 Nms/rad is the existing URDF joint damping, unchanged in comparison.
        v += 0.001 * dynamics.solve(applied - support - 0.5 * v);
        q += 0.001 * v;
        require(q.allFinite() && v.allFinite(), "nonfinite grounded state");
        peak_error = std::max(peak_error, (q - reference).norm());
    }
    return {peak_error, (q - reference).norm(), peak_torque};
}
} // namespace

int main()
{
    const JointVector zero = JointVector::Zero();
    const JointVector kp(25,45,25,45), kd(3.5,6,3.5,6);
    const JointVector reference(.2,-.4,.2,-.4);
    const auto mass = flight_joint_inertia(reference,9.5);
    const JointVector feedback = discrete_ground_feedback(
        mass,reference,zero,reference,zero,kp,kd,zero,0,55,15,.025);
    require(feedback.norm()<1e-12,"static equilibrium generated feedback");
    const JointVector support(7,-22,7,-22);
    require(((support+feedback)-support).norm()<1e-12,
            "support was attenuated by implicit feedback");

    const JointVector disturbance(.04,0,.04,0);
    // Faster gyro feedback is a stress case for the old explicit/slew path.
    // The sweep below separately uses the production alpha=0.15. Do not claim
    // that this frozen model must reproduce every feature of the real log.
    const auto old_result=simulate(false,reference,disturbance,zero,1.0,20,.254,.35);
    const auto new_result=simulate(true,reference,disturbance,zero,1.0,20,.254,.35);
    require(new_result.final_error<.01 && new_result.peak_error<.12,
            "joint/body sampled feedback did not converge");
    require(old_result.peak_error>new_result.peak_error*3,
            "comparison did not reproduce delayed explicit/slew oscillation");
    std::cout<<"Ground local model old/new peak error="<<old_result.peak_error
             <<"/"<<new_result.peak_error<<", final="<<new_result.final_error<<'\n';

    int scenarios=0;
    for (double hip:{-.15,.2,.59}) {
        for (double knee:{-.8,-.4}) {
            const JointVector q(hip,knee,hip+.03,knee-.02);
            for (double scale:{.7,1.0,1.3}) {
                for (int period:{10,15,20}) {
                    for (double contact:{0.0,.254,.8}) {
                        const auto result=simulate(true,q,JointVector(.03,-.02,-.01,.025),
                            zero,scale,period,contact);
                        require(result.peak_error<.12 && result.final_error<.012,
                                "ground inertia/delay sweep diverged");
                        ++scenarios;
                    }
                }
            }
        }
    }
    // Measured entry configuration and joint velocity from the 8.307s CSV row.
    // Track its current pose at entry, then dissipate the contact impulse. This
    // tests the feedback boundary; it does not replace the moving landing plan.
    const JointVector measured(.593045,-.176395,.59278,-.176634);
    const JointVector measured_velocity(-.588061,5.16509,-.591943,5.16855);
    const double measured_rate=1.39667, measured_pitch_error=.380085-.038;
    const double measured_body=std::clamp(
        -.5*(55*measured_pitch_error+15*measured_rate),-20.0,20.0);
    const JointVector entry=discrete_ground_feedback(
        flight_joint_inertia(measured,9.5),measured,measured_velocity,measured,
        zero,kp,kd,JointVector(measured_body,0,measured_body,0),
        measured_rate,55,15,.025);
    require(entry.allFinite() && entry.cwiseAbs().maxCoeff()<40,
            "measured touchdown feedback generated an immediate torque spike");
    const auto landing=simulate(true,measured,zero,measured_velocity,1.0,20,.254);
    require(landing.peak_error<.65 && landing.final_error<.02,
            "measured touchdown velocity caused persistent oscillation");
    std::cout<<"Measured touchdown local model peak error="<<landing.peak_error
             <<", final="<<landing.final_error<<'\n';
    std::cout<<"PASS: static-load equilibrium, body/joint sampled loop, "
             <<scenarios<<" inertia/contact/delay cases (not Gazebo validation)\n";
}
