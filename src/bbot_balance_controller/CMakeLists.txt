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

# 安装规则
install(TARGETS
  balance_controller_keyboard
  lqr_balance_controller_yaokong
  lqr_gain_scheduled_controller
  bbot_jump_controller
  bbot_velocity_jump_controller
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  scripts/optimize_velocity_jump.py
  DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_flight_joint_pd test/test_flight_joint_pd.cpp)
  target_link_libraries(test_flight_joint_pd Eigen3::Eigen)
  add_test(NAME flight_joint_pd COMMAND test_flight_joint_pd)
  add_executable(test_rolling_jump test/test_rolling_jump.cpp)
  add_test(NAME rolling_jump COMMAND test_rolling_jump)
  add_executable(test_takeoff_detection test/test_takeoff_detection.cpp)
  add_test(NAME takeoff_detection COMMAND test_takeoff_detection)
  add_executable(test_jump_phase_control test/test_jump_phase_control.cpp)
  add_test(NAME jump_phase_control COMMAND test_jump_phase_control)
  add_executable(test_world_pose_velocity test/test_world_pose_velocity.cpp)
  add_test(NAME world_pose_velocity COMMAND test_world_pose_velocity)
  add_executable(test_centroidal_capture test/test_centroidal_capture.cpp)
  target_link_libraries(test_centroidal_capture Eigen3::Eigen)
  add_test(NAME centroidal_capture COMMAND test_centroidal_capture)
  add_executable(test_centroidal_state test/test_centroidal_state.cpp)
  target_link_libraries(test_centroidal_state Eigen3::Eigen)
  add_test(NAME centroidal_state COMMAND test_centroidal_state)
  add_executable(test_effort_allocation test/test_effort_allocation.cpp)
  target_link_libraries(test_effort_allocation Eigen3::Eigen)
  add_test(NAME effort_allocation COMMAND test_effort_allocation)
  add_executable(test_torso_pitch_control test/test_torso_pitch_control.cpp)
  target_link_libraries(test_torso_pitch_control Eigen3::Eigen)
  add_test(NAME torso_pitch_control COMMAND test_torso_pitch_control)
  add_executable(test_ground_joint_pd test/test_ground_joint_pd.cpp)
  target_link_libraries(test_ground_joint_pd Eigen3::Eigen)
  add_test(NAME ground_joint_pd COMMAND test_ground_joint_pd)
endif()

ament_package()
