cmake_minimum_required(VERSION 3.8)
project(bbot_torque_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(bbot_kinematics REQUIRED)

# ======== 力矩监控节点 ========
add_executable(torque_monitor
  src/torque_monitor.cpp
)

target_include_directories(torque_monitor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(torque_monitor
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  bbot_kinematics
)

# ======== 高度控制器节点 ========
add_executable(height_controller
  src/height_controller.cpp
)

target_include_directories(height_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(height_controller
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  trajectory_msgs
  bbot_kinematics
)

# ======== 安装 ========
install(TARGETS
  torque_monitor
  height_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
