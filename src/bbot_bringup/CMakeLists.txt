cmake_minimum_required(VERSION 3.8)
project(bbot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bbot_balance_controller REQUIRED)

add_library(bbot_ground_contact_frame_recorder SHARED
  src/ground_contact_frame_recorder.cc)
target_link_libraries(bbot_ground_contact_frame_recorder
  ignition-gazebo6::core
  ignition-plugin1::ignition-plugin1
  ignition-transport11::core
  ignition-msgs8::core)
ament_target_dependencies(bbot_ground_contact_frame_recorder bbot_balance_controller)
target_compile_features(bbot_ground_contact_frame_recorder PRIVATE cxx_std_17)
install(TARGETS bbot_ground_contact_frame_recorder
  LIBRARY DESTINATION lib)

# A separate passive observer loaded only by the reference experiment world.
add_library(bbot_reference_jump_geometry_recorder SHARED
  src/reference_jump_geometry_recorder.cc)
target_link_libraries(bbot_reference_jump_geometry_recorder
  ignition-gazebo6::core ignition-plugin1::ignition-plugin1)
target_compile_features(bbot_reference_jump_geometry_recorder PRIVATE cxx_std_17)
install(TARGETS bbot_reference_jump_geometry_recorder LIBRARY DESTINATION lib)

# Disabled unless BBOT_NATIVE_WRENCH_CSV is supplied by the repair runner.
add_library(bbot_landing_repair_wrench_recorder SHARED
  src/landing_repair_wrench_recorder.cc)
target_link_libraries(bbot_landing_repair_wrench_recorder
  ignition-gazebo6::core ignition-plugin1::ignition-plugin1
  ignition-transport11::core ignition-msgs8::core)
target_compile_features(bbot_landing_repair_wrench_recorder PRIVATE cxx_std_17)
install(TARGETS bbot_landing_repair_wrench_recorder LIBRARY DESTINATION lib)

# Separate filename prevents an experiment from loading an old installed
# recorder when the normal bringup library directory is first on the path.
add_library(bbot_native_command_observer SHARED src/landing_repair_wrench_recorder.cc)
target_link_libraries(bbot_native_command_observer
  ignition-gazebo6::core ignition-plugin1::ignition-plugin1
  ignition-transport11::core ignition-msgs8::core)
target_compile_features(bbot_native_command_observer PRIVATE cxx_std_17)
install(TARGETS bbot_native_command_observer LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_native_command_observer test/test_native_command_observer.cc)
  target_link_libraries(test_native_command_observer
    ignition-gazebo6::core ignition-plugin1::ignition-plugin1 ignition-msgs8::core)
  target_compile_features(test_native_command_observer PRIVATE cxx_std_17)
  target_compile_options(test_native_command_observer PRIVATE -UNDEBUG)
  add_test(NAME test_native_command_observer COMMAND test_native_command_observer)

  # Keep the native-state wire parser test in this independent observer build;
  # the balance controller's normal CMake/test registration is untouched.
  add_executable(test_native_command_state
    ../bbot_balance_controller/test/test_native_command_state.cpp)
  target_include_directories(test_native_command_state PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../bbot_balance_controller/include)
  target_link_libraries(test_native_command_state Eigen3::Eigen)
  ament_target_dependencies(test_native_command_state bbot_balance_controller)
  target_compile_features(test_native_command_state PRIVATE cxx_std_17)
  target_compile_options(test_native_command_state PRIVATE -UNDEBUG)
  add_test(NAME test_native_command_state COMMAND test_native_command_state)
endif()

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
