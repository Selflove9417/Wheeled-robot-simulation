cmake_minimum_required(VERSION 3.8)
project(bbot_balance_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

# 安装规则
install(TARGETS
  balance_controller_keyboard
  lqr_balance_controller_yaokong
  bbot_jump_controller
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()