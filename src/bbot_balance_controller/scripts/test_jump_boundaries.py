#!/usr/bin/env python3
"""Compile actual trajectory/geometry/gate code in a ROS-free regression fixture.

No ROS node is created, so existing Gazebo CSV logs are never opened/truncated.
Run: python3 src/bbot_balance_controller/scripts/test_jump_boundaries.py
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp').read_text()
namespace = source[source.index('namespace bbot_jump'):source.index('class BBotVelocityJumpController')]
methods = source[source.index('    double body_height_above_wheel_ground'):source.index('    // ── 阶段 3：腾空相控制')]
gate = source[source.index('        const bool fresh_world ='):source.index('        if (leg_compressed || torque_spike || imu_impact)')]
gate = gate.replace('        log_data(cmd_x, 0.0, 0.0, 0.0);', '')
cpp = r'''
#include <array>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <iostream>
#include "bbot_kinematics/kinematics.hpp"
''' + namespace + r'''
struct Fixture {
    bbot_kinematics::Kinematics kinematics_;
    double pitch_=0, balance_offset_=0.034;
    double L_RETRACT_=0.30, L_TOUCH_=0.42;
    double hip_pos_left_=0.5, knee_pos_left_=-0.8, hip_pos_right_=0.4, knee_pos_right_=-0.7;
    double hip_vel_left_=3.6, knee_vel_left_=-8.6, hip_vel_right_=1.8, knee_vel_right_=-4.2;
    bool flight_trajectory_initialized_=false;
    bbot_jump::FlightSubphase flight_subphase_=bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY;
    bbot_jump::QuinticTrajectory arrest_hip_left_traj_, arrest_knee_left_traj_, arrest_hip_right_traj_, arrest_knee_right_traj_;
    bbot_jump::QuinticTrajectory protective_hip_left_traj_, protective_knee_left_traj_, protective_hip_right_traj_, protective_knee_right_traj_;
    std::array<bbot_jump::QuinticTrajectory,4> normal_flight_joint_traj_;
    bool odom_received_=true, contact_window_=false;
    double last_world_odom_time_=1, gazebo_world_z_dot_=-1, last_world_descent_time_=-1;
    double gazebo_world_z_=0.4, ground_height_offset_=0, wheel_clearance_=0;
    double current_z_=0.4, current_z_dot_=0, knee_effort_left_=40, knee_effort_right_=40, acc_z_filt_=0;
    int touchdown_knee_effort_count_=0;
''' + methods + '''
    bool contact(double now_sec, bool legs_deployed, double L_target) {
''' + gate + r'''
        return leg_compressed || torque_spike || imu_impact;
    }
};
void close(double a, double b) { assert(std::abs(a-b)<1e-7); }
int main() {
    // Actual failed run: a 50 ms tuck asks for roughly 60 rad/s.
    bbot_jump::QuinticTrajectory fast_tuck;
    fast_tuck.init(0,0.05,1.29874,0,0,-0.30,0,0);
    assert(!bbot_jump::flight_trajectory_admissible(fast_tuck,30,10000));
    // Endpoints inside joint bounds do not imply the entire curve is safe.
    bbot_jump::QuinticTrajectory overshoot;
    overshoot.init(0,0.24,-1.34551,-16.0863,0,-0.367,0,0);
    assert(!bbot_jump::flight_trajectory_admissible(overshoot,30,10000));
    close(bbot_jump::joint_extension_scale(0.2,0),1);
    close(bbot_jump::joint_extension_scale(-1.34551,-16.0863),0);
    assert(bbot_jump::joint_extension_scale(-0.94586,-10.9981)<1);
    Fixture f;
    std::array<double,4> q,v,a, q2,v2,a2;
    f.plan_flight_joints(0,0.24,0.42,true);
    f.sample_flight_joints(0,q,v,a);
    close(q[0],0.5); close(q[2],0.4);
    close(v[0],3.6); close(v[1],-8.6); close(v[2],1.8); close(v[3],-4.2);
    // V3 inherits the full measured speed, not the obsolete +/-5 and +/-10 clips.
    Fixture high_speed;
    high_speed.hip_vel_left_=11.6223; high_speed.knee_vel_left_=-16.0863;
    high_speed.plan_flight_joints(0,0.24,0.42,true);
    high_speed.sample_flight_joints(0,q2,v2,a2);
    close(v2[0],11.6223); close(v2[1],-16.0863);
    assert(!high_speed.flight_leg_motion_ready());
    Fixture stable;
    auto standing=stable.kinematics_.inverse_kinematics(0.42,-0.034);
    stable.hip_pos_left_=stable.hip_pos_right_=standing.theta_hip;
    stable.knee_pos_left_=stable.knee_pos_right_=standing.theta_knee;
    stable.hip_vel_left_=stable.hip_vel_right_=stable.knee_vel_left_=stable.knee_vel_right_=0;
    stable.plan_flight_joints(0,0.24,0.42,true);
    assert(stable.tuck_round_trip_feasible(0,0.16,0.16,0.5));
    assert(!stable.tuck_round_trip_feasible(0,0.16,0.16,0.2));
    assert(!stable.tuck_round_trip_feasible(0,0.05,0.16,0.5));
    f.flight_trajectory_initialized_=true;
    // Interrupt an unfinished trajectory, then immediately retarget again.
    for (int stage=0;stage<3;++stage) {
        double t=0.06+stage*0.02;
        f.sample_flight_joints(t,q,v,a);
        bool protect=stage==2;
        f.plan_flight_joints(t,0.16,protect?0.42:0.35,protect);
        f.flight_subphase_=protect?bbot_jump::FLIGHT_SUBPHASE_PROTECTIVE_DEPLOY:
            (stage==0?bbot_jump::FLIGHT_SUBPHASE_TUCK:bbot_jump::FLIGHT_SUBPHASE_EXTEND);
        f.sample_flight_joints(t,q2,v2,a2);
        for (int i=0;i<4;++i) { close(q[i],q2[i]); close(v[i],v2[i]); close(a[i],a2[i]); }
    }
    f.sample_flight_joints(1,q,v,a);
    for (int i=0;i<4;++i) { close(v[i],0); close(a[i],0); }
    // Independent transform of URDF offsets, including large body pitch.
    for (double pitch : {-1.0,-0.3,0.0,0.3}) {
        f.pitch_=pitch;
        double h=0.2,k=-0.4;
        double dy=-0.29348091*std::cos(h)+0.06220095*std::sin(h)
            +0.28210870*std::cos(h+k)+0.19553796*std::sin(h+k);
        double dz=-0.29348091*std::sin(h)-0.06220095*std::cos(h)
            +0.28210870*std::sin(h+k)-0.19553796*std::cos(h+k);
        double expected=(0.125+dy)*std::sin(pitch)-(-0.07+dz)*std::cos(pitch)+0.07;
        // Model link lengths round the CAD dimensions by a few micrometers.
        assert(std::abs(f.body_height_above_wheel_ground(h,k)-expected)<2e-5);
    }
    f.pitch_=0;
    auto ik=f.kinematics_.inverse_kinematics(0.42,0);
    f.hip_pos_left_=f.hip_pos_right_=ik.theta_hip;
    f.knee_pos_left_=f.knee_pos_right_=ik.theta_knee;
    f.gazebo_world_z_=0.56; f.current_z_dot_=-0.3; f.acc_z_filt_=20;
    assert(!f.contact(1,true,0.42)); // airborne torque/IMU spike
    f.gazebo_world_z_=0.425; f.current_z_dot_=0; f.acc_z_filt_=0;
    assert(!f.contact(1,true,0.42)); // actuator effort alone is insufficient
    f.acc_z_filt_=20; f.gazebo_world_z_dot_=0;
    assert(f.contact(1.02,true,0.42)); // impact stopped descent; history survives
    assert(!f.contact(1.2,true,0.42)); // stale odometry cannot confirm contact
    f.last_world_odom_time_=2; f.gazebo_world_z_dot_=1;
    assert(!f.contact(2,true,0.42)); // ascending spike
    std::cout << "PASS: flight speed/travel guards, tuck admission, measured-speed handoff, C2 transitions, geometry/contact\n";
}
'''
with tempfile.TemporaryDirectory(prefix='bbot_jump_test_') as directory:
    test = Path(directory) / 'test.cpp'
    binary = Path(directory) / 'test'
    test.write_text(cpp)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-I', str(root / 'src/bbot_kinematics/include'), str(test), str(root / 'src/bbot_kinematics/src/kinematics.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
