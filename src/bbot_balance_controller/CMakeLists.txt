cmake_minimum_required(VERSION 3.8)
project(bbot_balance_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bbot_kinematics REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(include)

# 键盘控制版本 (串级 PID)
add_executable(balance_controller_keyboard
  src/balance_controller_keyboard.cpp
  src/pid.cpp
  src/keyboard_reader.cpp
)
ament_target_dependencies(balance_controller_keyboard
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# 键盘自平衡 LQR 控制器
add_executable(lqr_balance_controller_yaokong
  src/lqr_balance_controller_yaokong.cpp
  src/keyboard_reader.cpp
)
ament_target_dependencies(lqr_balance_controller_yaokong
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# 论文版：基于轮毂力矩输入的 Gain-Scheduled LQR
add_executable(lqr_gain_scheduled_controller
  src/lqr_gain_scheduled_controller.cpp
  src/keyboard_reader.cpp
)

ament_target_dependencies(lqr_gain_scheduled_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# 论文 3.2：带慢速等效质心偏置估计的 Gain-Scheduled LQR
add_executable(adaptive_lqr_balance_controller
  src/adaptive_lqr_balance_controller.cpp
)
ament_target_dependencies(adaptive_lqr_balance_controller
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  bbot_kinematics
)

# 力矩级三环串级 PID 控制器 (高度增益调度对照)
add_executable(torque_cascade_pid_controller
  src/torque_cascade_pid_controller.cpp
)
ament_target_dependencies(torque_cascade_pid_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# Independent four-loop position -> velocity -> attitude -> rate PID
add_executable(position_torque_cascade_pid_controller
  src/position_torque_cascade_pid_controller.cpp
)
ament_target_dependencies(position_torque_cascade_pid_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# Independent constrained Linear MPC balance controller (fixed height, phase 1)
add_executable(linear_mpc_balance_controller
  src/linear_mpc_balance_controller.cpp
  src/keyboard_reader.cpp
)
target_link_libraries(linear_mpc_balance_controller Eigen3::Eigen)
# 每拍按 current_height 重建 Ad/Bd/P 与预测矩阵/QP。实测默认构建（CMAKE_BUILD_TYPE
# 为空，即 -O0）下重建 3.16 ms + QP 1.47 ms，已超 5 ms 控制周期；同一代码 -O2 下合计
# 97 us（预算的 1.9%）。只给本目标开优化，其余控制器口径不变。
target_compile_options(linear_mpc_balance_controller PRIVATE -O2)
ament_target_dependencies(linear_mpc_balance_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
)

# C++ 键盘控制终端 (无 Python / Conda 冲突)
add_executable(teleop_keyboard
  src/teleop_keyboard.cpp
)
ament_target_dependencies(teleop_keyboard
  rclcpp
  geometry_msgs
  std_msgs
)

# BBot 跳跃与 LQR 自平衡复合控制器
add_executable(bbot_jump_controller
  src/bbot_jump_controller.cpp
  src/keyboard_reader.cpp
)
ament_target_dependencies(bbot_jump_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
  controller_manager_msgs
)

# 目标离地速度闭环跳跃控制器（独立于原跳跃控制器）
add_executable(bbot_velocity_jump_controller
  src/bbot_velocity_jump_controller.cpp
  src/keyboard_reader.cpp
)
target_link_libraries(bbot_velocity_jump_controller Eigen3::Eigen)
ament_target_dependencies(bbot_velocity_jump_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  bbot_kinematics
  controller_manager_msgs
)

# Independent open-source-inspired jumping experiment.
add_executable(bbot_reference_jump_controller
  src/bbot_reference_jump_controller.cpp src/keyboard_reader.cpp)
target_link_libraries(bbot_reference_jump_controller Eigen3::Eigen)
ament_target_dependencies(bbot_reference_jump_controller
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs
  bbot_kinematics controller_manager_msgs)

# Landing repair experiment copied from the complete-jump baseline.
add_executable(bbot_landing_repair_controller
  src/bbot_landing_repair_controller.cpp src/keyboard_reader.cpp)
target_link_libraries(bbot_landing_repair_controller Eigen3::Eigen)
ament_target_dependencies(bbot_landing_repair_controller
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs
  bbot_kinematics controller_manager_msgs)

# 安装规则
install(TARGETS
  balance_controller_keyboard
  linear_mpc_balance_controller
  lqr_balance_controller_yaokong
  lqr_gain_scheduled_controller
  adaptive_lqr_balance_controller
  torque_cascade_pid_controller
  position_torque_cascade_pid_controller
  bbot_jump_controller
  bbot_velocity_jump_controller
  bbot_reference_jump_controller
  bbot_landing_repair_controller
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  scripts/optimize_velocity_jump.py
  scripts/run_position_pid_exploration.py
  scripts/tune_position_torque_pid.py
  scripts/plot_position_pid_exploration.py
  scripts/run_height_campaign.py
  scripts/run_formal_pid_gslqr_campaign.py
  scripts/analyze_formal_pid_gslqr.py
  scripts/analyze_chapter42_height_recheck.py
  scripts/run_flat_ground_jump_trial.py
  scripts/jump_profile.py
  scripts/audit_ground_contact_frames.py
  scripts/audit_landing_geometry.py
  scripts/jump_startup_gate.py
  scripts/record_wheel_contacts.py
  scripts/record_wheel_joint_state.py
  scripts/wheel_effort_velocity_servo.py
  scripts/analyze_wheel_chain.py
  scripts/analyze_flat_jump_repeatability.py
  scripts/analyze_jump_handoff.py
  scripts/analyze_command_transport.py
  DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_reference_ground_torque_pulse test/test_reference_ground_torque_pulse.cpp)
  add_test(NAME reference_ground_torque_pulse COMMAND test_reference_ground_torque_pulse)
  add_executable(test_reference_jump_policy test/test_reference_jump_policy.cpp)
  ament_target_dependencies(test_reference_jump_policy bbot_kinematics)
  target_link_libraries(test_reference_jump_policy Eigen3::Eigen)
  add_test(NAME reference_jump_policy COMMAND test_reference_jump_policy)
  add_executable(test_flight_landing_geometry test/test_flight_landing_geometry.cpp)
  ament_target_dependencies(test_flight_landing_geometry bbot_kinematics)
  target_link_libraries(test_flight_landing_geometry Eigen3::Eigen)
  add_test(NAME flight_landing_geometry COMMAND test_flight_landing_geometry)

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(
    NAME landing_geometry_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_landing_geometry_audit.py
  )
  add_test(
    NAME jump_startup_gate
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_jump_startup_gate.py
  )
  add_test(
    NAME jump_profile
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_jump_profile.py
  )
  add_test(
    NAME runner_controller_loading
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runner_controller_loading.py
  )
  add_test(
    NAME ground_contact_frames
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ground_contact_frames.py
  )
  add_test(
    NAME contact_flight_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_contact_flight_audit.py
  )
  add_test(
    NAME jump_session_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_jump_session_audit.py
  )
  add_test(
    NAME ground_contact_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ground_contact_audit.py
  )
  add_test(
    NAME landing_hold_audit
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_landing_hold_audit.py
  )
  add_executable(
    test_adaptive_equilibrium_estimator
    test/test_adaptive_equilibrium_estimator.cpp
  )
  add_test(
    NAME adaptive_equilibrium_estimator
    COMMAND test_adaptive_equilibrium_estimator
  )
  add_executable(test_flight_trajectory test/test_flight_trajectory.cpp)
  add_test(NAME flight_trajectory COMMAND test_flight_trajectory)
  add_executable(test_flight_joint_pd test/test_flight_joint_pd.cpp)
  target_link_libraries(test_flight_joint_pd Eigen3::Eigen)
  add_test(NAME flight_joint_pd COMMAND test_flight_joint_pd)
  add_executable(test_ground_motion_probe test/test_ground_motion_probe.cpp)
  target_link_libraries(test_ground_motion_probe Eigen3::Eigen)
  add_test(NAME ground_motion_probe COMMAND test_ground_motion_probe)
  add_executable(test_ground_engine_state test/test_ground_engine_state.cpp)
  target_link_libraries(test_ground_engine_state Eigen3::Eigen)
  target_compile_options(test_ground_engine_state PRIVATE -UNDEBUG)
  add_test(NAME ground_engine_state COMMAND test_ground_engine_state)
  add_executable(test_ground_contact_motion_control test/offline/test_ground_contact_motion_control.cpp)
  target_link_libraries(test_ground_contact_motion_control Eigen3::Eigen)
  target_compile_options(test_ground_contact_motion_control PRIVATE -UNDEBUG -O2)
  add_test(NAME ground_contact_motion_control COMMAND test_ground_contact_motion_control)
  add_executable(test_arrest_dynamics_feedforward test/test_arrest_dynamics_feedforward.cpp)
  target_link_libraries(test_arrest_dynamics_feedforward Eigen3::Eigen)
  add_test(NAME arrest_dynamics_feedforward COMMAND test_arrest_dynamics_feedforward)
  add_executable(test_landing_repair_momentum test/test_landing_repair_momentum.cpp)
  target_link_libraries(test_landing_repair_momentum Eigen3::Eigen)
  add_test(NAME landing_repair_momentum COMMAND test_landing_repair_momentum)
  add_executable(test_thrust_support_dynamics test/test_thrust_support_dynamics.cpp)
  target_link_libraries(test_thrust_support_dynamics Eigen3::Eigen)
  target_compile_options(test_thrust_support_dynamics PRIVATE -UNDEBUG)
  add_test(NAME thrust_support_dynamics COMMAND test_thrust_support_dynamics)
  add_executable(test_allocator_flight_plan test/test_allocator_flight_plan.cpp)
  target_link_libraries(test_allocator_flight_plan Eigen3::Eigen)
  ament_target_dependencies(test_allocator_flight_plan bbot_kinematics)
  target_compile_options(test_allocator_flight_plan PRIVATE -UNDEBUG -O2)
  add_test(NAME allocator_flight_plan COMMAND test_allocator_flight_plan)

  add_executable(test_adaptive_flight_braking test/test_adaptive_flight_braking.cpp)
  target_link_libraries(test_adaptive_flight_braking Eigen3::Eigen)
  ament_target_dependencies(test_adaptive_flight_braking bbot_kinematics)
  target_compile_options(test_adaptive_flight_braking PRIVATE -UNDEBUG -O2)
  add_test(NAME adaptive_flight_braking COMMAND test_adaptive_flight_braking)

  add_executable(test_allocator_leg_symmetry test/test_allocator_leg_symmetry.cpp)
  target_link_libraries(test_allocator_leg_symmetry Eigen3::Eigen)
  target_compile_options(test_allocator_leg_symmetry PRIVATE -UNDEBUG -O2)
  add_test(NAME allocator_leg_symmetry COMMAND test_allocator_leg_symmetry)
  add_executable(test_thrust_body_pitch test/test_thrust_body_pitch.cpp)
  target_link_libraries(test_thrust_body_pitch Eigen3::Eigen)
  target_compile_options(test_thrust_body_pitch PRIVATE -UNDEBUG -O2)
  add_test(NAME thrust_body_pitch COMMAND test_thrust_body_pitch)
  add_executable(test_thrust_support_allocator test/test_thrust_support_allocator.cpp)
  target_link_libraries(test_thrust_support_allocator Eigen3::Eigen)
  target_compile_options(test_thrust_support_allocator PRIVATE -UNDEBUG -O2)
  add_test(NAME thrust_support_allocator COMMAND test_thrust_support_allocator)
  add_executable(test_landing_repair_handoff test/test_landing_repair_handoff.cpp)
  add_test(NAME landing_repair_handoff COMMAND test_landing_repair_handoff)
  add_executable(test_thrust_support_coordination test/test_thrust_support_coordination.cpp)
  target_link_libraries(test_thrust_support_coordination Eigen3::Eigen)
  target_compile_options(test_thrust_support_coordination PRIVATE -UNDEBUG)
  add_test(NAME thrust_support_coordination COMMAND test_thrust_support_coordination)
  add_executable(test_velocity_launch_capture test/test_velocity_launch_capture.cpp)
  target_link_libraries(test_velocity_launch_capture Eigen3::Eigen)
  target_compile_options(test_velocity_launch_capture PRIVATE -UNDEBUG)
  add_test(NAME velocity_launch_capture COMMAND test_velocity_launch_capture)
  add_executable(test_thrust_output_ownership test/test_thrust_output_ownership.cpp)
  target_include_directories(test_thrust_output_ownership PRIVATE include)
  add_test(NAME thrust_output_ownership COMMAND test_thrust_output_ownership)
  add_test(
    NAME landing_repair_controller_paths
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_landing_repair_controller_paths.py
  )

  add_executable(test_landing_repair_thrust_reference test/test_landing_repair_thrust_reference.cpp)
  target_link_libraries(test_landing_repair_thrust_reference Eigen3::Eigen)
  add_test(NAME landing_repair_thrust_reference COMMAND test_landing_repair_thrust_reference)
  add_executable(test_rolling_jump test/test_rolling_jump.cpp)
  add_test(NAME rolling_jump COMMAND test_rolling_jump)
  add_executable(test_takeoff_detection test/test_takeoff_detection.cpp)
  add_test(NAME takeoff_detection COMMAND test_takeoff_detection)
  add_executable(test_jump_phase_control test/test_jump_phase_control.cpp)
  target_link_libraries(test_jump_phase_control Eigen3::Eigen)
  add_test(NAME jump_phase_control COMMAND test_jump_phase_control)

  add_executable(test_control_timing test/test_control_timing.cpp)
  add_test(NAME control_timing COMMAND test_control_timing)
  add_executable(test_latest_sensor_qos test/test_latest_sensor_qos.cpp)
  ament_target_dependencies(test_latest_sensor_qos rclcpp)
  add_test(NAME latest_sensor_qos COMMAND test_latest_sensor_qos)
  add_executable(test_complete_contact_takeoff test/test_complete_contact_takeoff.cpp)
  target_include_directories(test_complete_contact_takeoff PRIVATE include)
  add_test(NAME complete_contact_takeoff COMMAND test_complete_contact_takeoff)
  add_executable(test_allocator_contact_history test/test_allocator_contact_history.cpp)
  target_include_directories(test_allocator_contact_history PRIVATE include)
  target_compile_options(test_allocator_contact_history PRIVATE -UNDEBUG)
  add_test(NAME allocator_contact_history COMMAND test_allocator_contact_history)
  add_executable(test_thrust_wheel_kinematics test/test_thrust_wheel_kinematics.cpp)
  target_link_libraries(test_thrust_wheel_kinematics Eigen3::Eigen)
  add_test(NAME thrust_wheel_kinematics COMMAND test_thrust_wheel_kinematics)
  add_executable(test_world_pose_velocity test/test_world_pose_velocity.cpp)
  add_test(NAME world_pose_velocity COMMAND test_world_pose_velocity)
  add_executable(test_centroidal_capture test/test_centroidal_capture.cpp)
  target_link_libraries(test_centroidal_capture Eigen3::Eigen)
  add_test(NAME centroidal_capture COMMAND test_centroidal_capture)
  add_executable(test_thrust_velocity_reference test/test_thrust_velocity_reference.cpp)
  target_link_libraries(test_thrust_velocity_reference Eigen3::Eigen)
  add_test(NAME thrust_velocity_reference COMMAND test_thrust_velocity_reference)
  add_executable(test_centroidal_state test/test_centroidal_state.cpp)
  target_link_libraries(test_centroidal_state Eigen3::Eigen)
  add_test(NAME centroidal_state COMMAND test_centroidal_state)
  add_executable(test_effort_allocation test/test_effort_allocation.cpp)
  target_link_libraries(test_effort_allocation Eigen3::Eigen)
  add_test(NAME effort_allocation COMMAND test_effort_allocation)
  add_executable(test_torso_pitch_control test/test_torso_pitch_control.cpp)
  target_link_libraries(test_torso_pitch_control Eigen3::Eigen)
  add_test(NAME torso_pitch_control COMMAND test_torso_pitch_control)
  add_executable(test_dense_active_set_qp test/test_dense_active_set_qp.cpp)
  target_link_libraries(test_dense_active_set_qp Eigen3::Eigen)
  add_test(NAME dense_active_set_qp COMMAND test_dense_active_set_qp)
  add_executable(test_lqr_plant_model test/test_lqr_plant_model.cpp)
  target_link_libraries(test_lqr_plant_model Eigen3::Eigen)
  ament_target_dependencies(test_lqr_plant_model bbot_kinematics)
  add_test(NAME lqr_plant_model COMMAND test_lqr_plant_model)
  add_executable(test_linear_mpc test/test_linear_mpc.cpp)
  target_link_libraries(test_linear_mpc Eigen3::Eigen)
  ament_target_dependencies(test_linear_mpc bbot_kinematics)
  add_test(NAME linear_mpc COMMAND test_linear_mpc)
  add_executable(test_ground_joint_pd test/test_ground_joint_pd.cpp)
  target_link_libraries(test_ground_joint_pd Eigen3::Eigen)
  add_test(NAME ground_joint_pd COMMAND test_ground_joint_pd)
  add_executable(test_jump_regression test/test_jump_regression.cpp)
  target_link_libraries(test_jump_regression Eigen3::Eigen)
  add_test(NAME jump_regression COMMAND test_jump_regression)
  add_test(
    NAME wheel_effort_velocity_servo
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wheel_effort_velocity_servo.py
  )
  add_test(
    NAME jump_handoff_diagnostics
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_jump_handoff_diagnostics.py
  )
  add_test(
    NAME mpc_sweep_harness
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mpc_sweep_harness.py
  )
endif()

ament_export_include_directories(include)
ament_package()
