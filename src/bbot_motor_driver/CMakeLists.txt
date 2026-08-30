cmake_minimum_required(VERSION 3.8)

project(bbot_motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
include
)

add_executable(
motor_driver_node

src/can_interface.cpp
src/motor_driver_node.cpp
)

ament_target_dependencies(
motor_driver_node

rclcpp
std_msgs
)

install(
TARGETS motor_driver_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()