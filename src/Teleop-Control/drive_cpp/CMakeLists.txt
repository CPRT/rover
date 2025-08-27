cmake_minimum_required(VERSION 3.5)
project(drive_cpp)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_phoenix REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories
include_directories(include/${PROJECT_NAME})

# ========================================================
# Build shared library for TalonDriveController component
add_library(talon_drive_controller SHARED
  src/TalonDriveController.cpp
  src/WheelControl.cpp
)

#  Link component with dependencies
ament_target_dependencies(talon_drive_controller
  rclcpp
  rclcpp_components
  pluginlib
  geometry_msgs
  nav_msgs
  ros_phoenix
)

#  Register the node as a component
rclcpp_components_register_nodes(talon_drive_controller "TalonDriveController")

#  Install the shared library
install(TARGETS
  talon_drive_controller
  DESTINATION lib/
)

# ========================================================
add_executable(drive_cpp_node src/main.cpp)

target_link_libraries(drive_cpp_node
  talon_drive_controller
)

ament_target_dependencies(drive_cpp_node
  rclcpp
  geometry_msgs
  nav_msgs
  ros_phoenix
)

install(TARGETS
  drive_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES talon_drive_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


# ========================================================
# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
pluginlib_export_plugin_description_file(rclcpp_components talon_drive_controller_plugin.xml)
