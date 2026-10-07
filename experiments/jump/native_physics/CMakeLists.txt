cmake_minimum_required(VERSION 3.10.2)
project(bbot_ground_native_physics LANGUAGES CXX)

find_package(ignition-gazebo6 6.18 REQUIRED)
find_package(ignition-physics5 5.4 REQUIRED)
find_package(ignition-common4 REQUIRED COMPONENTS profiler)

add_library(ignition-gazebo-physics-system SHARED
  upstream/src/systems/physics/Physics.cc)
target_include_directories(ignition-gazebo-physics-system PRIVATE
  upstream/src/systems/physics)
target_compile_features(ignition-gazebo-physics-system PRIVATE cxx_std_17)
target_link_libraries(ignition-gazebo-physics-system PRIVATE
  ignition-gazebo6::core
  ignition-physics5::core
  ignition-common4::core
  ignition-common4::profiler)
set_target_properties(ignition-gazebo-physics-system PROPERTIES
  OUTPUT_NAME ignition-gazebo-physics-system
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugin")
