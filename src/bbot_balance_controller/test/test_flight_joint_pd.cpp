#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <Eigen/Eigenvalues>
#include "bbot_balance_controller/flight_joint_pd.hpp"

using namespace bbot_jump;

void require(bool ok, const char * message)
{
    if (!ok) throw std::runtime_error(message);
}

struct Result { double final_error, peak_error, peak_torque; };

// 1 ms integration, 100 Hz sensing/actuation, 200 Hz commands. Commands are
// applied on the NEXT actuator tick, reproducing a sampled/held feedback path.
// This is a local frozen-inertia model, not a Gazebo jumping-success test.
Result simulate(bool implicit, JointVector operating_q, JointVector initial_error,
                double inertia_scale = 1.0, int actuator_ms = 10)
{
    const JointMatrix nominal = flight_joint_inertia(operating_q, 9.5);
    const JointMatrix actual = inertia_scale * nominal;
    const auto dynamics = actual.ldlt();
    const JointVector kp(32,40,32,40), kd(7,8,7,8), limits(75,60,75,60);
    JointVector q = operating_q + initial_error, v = JointVector::Zero();
    JointVector qs = q, vs = v, torque = JointVector::Zero();
    JointVector pending = torque, applied = torque;
    double peak_error = initial_error.norm(), peak_torque = 0.0;
    for (int ms = 0; ms < 3000; ++ms) {
        if (ms % actuator_ms == 0) {
            applied = pending;
            qs = q; vs = v;
        }
        if (ms % 5 == 0) {
            JointVector request = kp.cwiseProduct(operating_q - qs) - kd.cwiseProduct(vs);
            if (implicit) {
                const double horizon = std::max(0.025, 0.002 * actuator_ms + (ms % actuator_ms) * 0.001);
                request = discrete_flight_pd(nominal, qs, vs, operating_q,
                    JointVector::Zero(), kp, kd, JointVector::Zero(), horizon);
            }
            request = request.cwiseMax(-limits).cwiseMin(limits);
            if (implicit) torque = request;
            else torque += (request - torque).cwiseMax(-15.0).cwiseMin(15.0);
            pending = torque;
            peak_torque = std::max(peak_torque, torque.cwiseAbs().maxCoeff());
        }
        // URDF joint damping=0.5 Nm*s/rad, in both old and new comparisons.
        v += 0.001 * dynamics.solve(applied - 0.5 * v);
        q += 0.001 * v;
        require(q.allFinite() && v.allFinite(), "non-finite closed-loop state");
        peak_error = std::max(peak_error, (q - operating_q).norm());
    }
    return {(q - operating_q).norm(), peak_error, peak_torque};
}

// Replay the measured v3 takeoff boundary against a moving 0.24 s reference.
// Trajectory is specified independently in Hermite form, not a copy of the PD.
void moving_takeoff_regression()
{
    const JointVector q0(.796263,-.939687,.796262,-.939687);
    const JointVector v0(2.94386,-8.10712,2.9438,-8.10708);
    const JointVector qf(.1979,-.3669,.1979,-.3669);
    const JointMatrix mass = flight_joint_inertia(q0,9.5);
    const auto dynamics = mass.ldlt();
    JointVector q=q0, v=v0, qs=q, vs=v;
    JointVector pending=JointVector::Zero(), applied=pending;
    double peak_torque=0, peak_speed=0;
    for (int ms=0; ms<1500; ++ms) {
        if (ms%10==0) { applied=pending; qs=q; vs=v; }
        if (ms%5==0) {
            const double t=0.001*ms, u=std::min(t/.24,1.0);
            const double u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u;
            const JointVector qd=q0+.24*v0*(u-6*u3+8*u4-3*u5)+(qf-q0)*(10*u3-15*u4+6*u5);
            const JointVector vd=v0*(1-18*u2+32*u3-15*u4)+(qf-q0)/.24*(30*u2-60*u3+30*u4);
            const double blend=std::min(t/.1,1.0);
            const JointVector kp(18+14*blend,22+18*blend,18+14*blend,22+18*blend);
            const JointVector kd(4.5+2.5*blend,5+3*blend,4.5+2.5*blend,5+3*blend);
            pending=discrete_flight_pd(mass,qs,vs,qd,vd,kp,kd,JointVector::Zero(),.025);
            peak_torque=std::max(peak_torque,pending.cwiseAbs().maxCoeff());
        }
        v+=.001*dynamics.solve(applied-.5*v);
        q+=.001*v;
        peak_speed=std::max(peak_speed,v.cwiseAbs().maxCoeff());
        require(q.cwiseAbs().maxCoeff()<1.45,"moving trajectory drove into joint limit");
    }
    require(peak_torque<25 && peak_speed<10 && (q-qf).norm()<.01,
            "measured takeoff boundary developed oscillation");
    std::cout << "Measured takeoff model: peak torque=" << peak_torque
              << ", peak speed=" << peak_speed << ", final error=" << (q-qf).norm() << '\n';
}

int main()
{
    const JointVector operating(0.8,-1.0,0.8,-1.0), disturbance(0.04,0,0.04,0);
    const auto old_result = simulate(false, operating, disturbance);
    const auto new_result = simulate(true, operating, disturbance);
    require(old_result.peak_error > 0.5, "regression no longer reproduces old slew/PD oscillation");
    require(new_result.peak_error < 0.08 && new_result.final_error < 0.001,
            "discrete PD failed to arrest the oscillation");
    std::cout << "Old explicit PD + slew: peak joint-error norm=" << old_result.peak_error
              << ", final=" << old_result.final_error << '\n';
    std::cout << "New discrete PD: peak joint-error norm=" << new_result.peak_error
              << ", final=" << new_result.final_error << '\n';

    for (double hip : {-0.4,0.2,0.8,1.1}) {
        for (double knee : {-1.2,-0.4,0.3}) {
            const JointVector config(hip,knee,hip+0.06,knee-0.04);
            const auto mass = flight_joint_inertia(config,9.5);
            require((mass-mass.transpose()).norm()<1e-10,"inertia not symmetric");
            require(Eigen::SelfAdjointEigenSolver<JointMatrix>(mass).eigenvalues().minCoeff()>0,
                    "inertia not positive definite");
            for (double scale : {0.7,1.0,1.3}) {
                for (int period : {10,15,20}) {
                    const auto result = simulate(true,config,JointVector(0.03,-0.02,-0.01,0.025),scale,period);
                    require(result.peak_error<0.10 && result.final_error<0.003,
                            "discrete PD failed inertia/delay sweep");
                }
            }
        }
    }
    // A matched moving reference must not manufacture damping torque.
    const JointVector moving(2,-4,1,-3), zero=JointVector::Zero();
    require(discrete_flight_pd(flight_joint_inertia(operating,9.5),operating,moving,
        operating,moving,JointVector(32,40,32,40),JointVector(7,8,7,8),zero,.025).norm()<1e-12,
        "matched target velocity generated a braking impulse");
    moving_takeoff_regression();
    std::cout << "PASS: sample/hold regression, 108 inertia-delay scenarios, moving-reference boundary\n";
}
