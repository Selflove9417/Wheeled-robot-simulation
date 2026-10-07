#include <cassert>
#include <cmath>
#include <limits>

#include "bbot_balance_controller/velocity_launch_capture.hpp"

using namespace bbot_jump;

static JointVector symmetric_q(double hip, double knee)
{
    JointVector q;
    q << hip, knee, hip, knee;
    return q;
}

static void verify_equalities(const JointVector & q, double pitch, double pitch_rate,
    double vz, double H, const std::array<double, 2> & wrel,
    const VelocityLaunchCaptureReference & ref)
{
    assert(ref.valid);
    const auto g = centroidal_geometry({q[0], q[1], q[2], q[3]}, 9.5);
    const Eigen::Vector3d vertical(0.0, -std::sin(pitch), std::cos(pitch));
    const Eigen::RowVector4d j = vertical.transpose() *
        (g.com_jacobian - g.axle_jacobian);
    const double com_h = j[0] + j[2];
    const double com_k = j[1] + j[3];
    const double com_rate = com_h * ref.hip_velocity + com_k * ref.knee_velocity -
        rotate_about_hip(-pitch, g.com-g.axle).y() * pitch_rate;
    constexpr double iw = 0.006481;
    const auto m = flight_pitch_joint_mass_matrix(q, 9.5);
    const double abs_wheel_l = -pitch_rate + ref.hip_velocity + ref.knee_velocity + wrel[0];
    const double abs_wheel_r = -pitch_rate + ref.hip_velocity + ref.knee_velocity + wrel[1];
    const double momentum = -m(0,0)*pitch_rate +
        (m(0,1)+m(0,3))*ref.hip_velocity +
        (m(0,2)+m(0,4))*ref.knee_velocity + iw*(abs_wheel_l+abs_wheel_r);
    assert(std::abs(com_rate-vz) < 1e-9);
    assert(std::abs(momentum-H) < 1e-9);
    assert(std::abs(ref.predicted_com_vz-vz) < 1e-9);
    assert(std::abs(ref.predicted_momentum-H) < 1e-9);
}

static void verify_callback_pairing_and_session()
{
    VelocityCaptureSampleHistory history;
    LandingJointSample joint{1.0,symmetric_q(0.4,-0.7),JointVector::Zero(),{0.0,0.0}};
    history.joints(joint); assert(!history.paired(1.0).valid);
    history.imu(1.0,0.03,0.05); assert(history.paired(1.005).valid);
    // A newer IMU arriving first must not hide the previous exact source pair.
    history.imu(1.005,0.04,0.07); assert(history.paired(1.008).joints.stamp==1.0);
    joint.stamp=1.005;history.joints(joint);
    assert(history.paired(1.008).joints.stamp==1.005);
    assert(!history.paired(1.030).valid);
    joint.stamp=0.50;history.joints(joint);assert(!history.paired(0.505).valid);

    const std::array<double,4> q{0.5,-0.7,0.5,-0.7}, velocity{2,-5,2,-5};
    auto stop=velocity_capture_stopping_distance(q,velocity,100,200,0.010);
    assert(stop.valid&&!stop.brake);assert(std::abs(stop.required[1]-0.1225)<1e-12);
    auto near=q;near[1]=-1.50;
    assert(velocity_capture_stopping_distance(near,velocity,100,200,0.010).brake);
    assert(!velocity_capture_stopping_distance(q,velocity,0,200,0.010).valid);
    VelocityCaptureSession session;
    const std::array<double,4> target{8,-12,8,-12}, measured{};
    auto state=session.update(1.0,false,true,true,true,false,true,target,measured,stop);
    assert(!state.owns_command);
    state=session.update(1.0,true,true,true,true,false,true,target,measured,stop);
    assert(state.owns_command&&state.phase==VelocityCapturePhase::Tracking);
    assert(state.reference==measured); // starts at true measured velocity
    state=session.update(1.005,true,true,true,true,false,true,target,measured,stop);
    assert(std::abs(state.reference[0]-2.25)<1e-10);
    assert(std::abs(state.reference[1]+2.5)<1e-10);
    state=session.update(1.010,true,true,true,true,false,false,target,measured,stop);
    assert(state.phase==VelocityCapturePhase::Braking);
    assert(state.reason==VelocityCaptureReason::InverseRejected);
    for(int i=0;i<20;++i) {
        state=session.update(1.015+0.005*i,true,true,true,true,false,i%2==0,target,measured,stop);
        assert(state.owns_command&&state.phase==VelocityCapturePhase::Braking);
        assert(state.reason==VelocityCaptureReason::InverseRejected);
    }
    session.reset();state=session.update(2.0,true,false,true,true,false,true,target,measured,stop);
    assert(state.reason==VelocityCaptureReason::StaleSensors);
    session.reset();state=session.update(2.0,true,true,false,true,false,true,target,measured,stop);
    assert(state.reason==VelocityCaptureReason::ContactUnavailable);
    session.reset();state=session.update(2.0,true,true,true,true,false,true,target,measured,
        velocity_capture_stopping_distance(q,velocity,0,0,0.010));
    assert(state.reason==VelocityCaptureReason::BrakeEvidenceUnavailable);
    assert(state.phase==VelocityCapturePhase::Braking);
    session.reset();state=session.update(2.0,true,true,true,false,false,true,target,measured,stop);
    assert(state.reason==VelocityCaptureReason::ActuatorUnavailable);
    session.reset();state=session.update(2.0,true,true,true,true,false,true,target,measured,
        velocity_capture_stopping_distance(near,velocity,100,200,0.010));
    assert(state.reason==VelocityCaptureReason::StoppingDistance);
}

int main()
{
    verify_callback_pairing_and_session();
    const auto q = symmetric_q(0.40, -0.70);
    const std::array<double,2> wheel_relative{0.30, -0.20};
    const auto ref = velocity_launch_capture_reference(
        q, 0.03, 0.05, 0.20, 0.08, wheel_relative, 9.5);
    verify_equalities(q, 0.03, 0.05, 0.20, 0.08, wheel_relative, ref);
    assert(std::abs(ref.hip_velocity) <= 11.0);
    assert(std::abs(ref.knee_velocity) <= 15.0);
    for (int i=0;i<4;++i) {
        const double qdot = (i%2==0) ? ref.hip_velocity : ref.knee_velocity;
        const double limit = (i%2==0) ? 1.52 : 1.56;
        assert(std::abs(q[i] + 0.030*qdot) <= limit);
    }

    // A feasible baseline family must stay within physical speed and travel
    // bounds. Independently expand the absolute-wheel contribution above.
    for (double hip : {0.35, 0.55, 0.75}) {
        const auto qb = symmetric_q(hip, -0.85);
        const auto rb = velocity_launch_capture_reference(
            qb, 0.02, 0.05, 0.15, 0.08, {0.0,0.0}, 9.5);
        assert(rb.valid);
        assert(std::abs(rb.hip_velocity) <= 11.0);
        assert(std::abs(rb.knee_velocity) <= 15.0);
        verify_equalities(qb, 0.02, 0.05, 0.15, 0.08, {0.0,0.0}, rb);
    }

    JointVector asymmetric; asymmetric << 0.40,-0.70,0.42,-0.71;
    const auto asym_ref=velocity_launch_capture_reference(asymmetric,0.03,0.05,
        0.20,0.08,wheel_relative,9.5);
    verify_equalities(asymmetric,0.03,0.05,0.20,0.08,wheel_relative,asym_ref);

    auto bad = velocity_launch_capture_reference(
        q, 0.03, std::numeric_limits<double>::quiet_NaN(),
        0.20, 0.08, wheel_relative, 9.5);
    assert(!bad.valid);
    bad = velocity_launch_capture_reference(
        q, 0.03, 0.05, 0.20, 1.0e6, wheel_relative, 9.5);
    assert(!bad.valid); // reject over-limit solution, never clip it
    auto near_stop = symmetric_q(1.515, -1.50);
    bad = velocity_launch_capture_reference(
        near_stop, 0.03, 0.05, 0.20, 0.08, wheel_relative, 9.5);
    assert(!bad.valid); // no 30 ms hard-stop crossing
}
