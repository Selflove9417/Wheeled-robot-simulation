cmake_minimum_required(VERSION 3.8)
project(bbot_kinematics)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Kinematics library (shared)
add_library(bbot_kinematics SHARED
  src/kinematics.cpp
)

target_include_directories(bbot_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(bbot_kinematics
  rclcpp::rclcpp
)

# Install library
install(TARGETS bbot_kinematics
  EXPORT export_bbot_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export targets and include directories for downstream packages
ament_export_targets(export_bbot_kinematics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

# Install package.xml
ament_package()
