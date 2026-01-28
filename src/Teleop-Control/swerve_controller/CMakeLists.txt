cmake_minimum_required(VERSION 3.8)
project(swerve_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#===============================================================================
# Dependencies
#===============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

#===============================================================================
# Library
#===============================================================================
add_library(swerve_controller_controller SHARED
  src/swerve_controller.cpp
  src/axle.cpp
)

target_compile_features(swerve_controller_controller PUBLIC cxx_std_17)

target_include_directories(swerve_controller_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(swerve_controller_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  realtime_tools
  nav_msgs
)

#===============================================================================
# Plugin description (for pluginlib / controller_manager)
#===============================================================================
pluginlib_export_plugin_description_file(controller_interface swerve_plugin.xml)

#===============================================================================
# Install
#===============================================================================

# Install headers so that #include "swerve_controller/..." works
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the shared library
install(
  TARGETS swerve_controller_controller
  EXPORT export_swerve_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#===============================================================================
# Export
#===============================================================================
ament_export_targets(export_swerve_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  realtime_tools
)

#===============================================================================
# Tests / lint
#===============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
