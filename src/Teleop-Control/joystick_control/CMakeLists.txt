cmake_minimum_required(VERSION 3.5)
project(joystick_control)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories for ROS 2
include_directories(include/${PROJECT_NAME})

# Declare the executables for the nodes
add_executable(flightstick_control 
  src/FlightstickControl.cpp 
  src/DriveMode.cpp 
  src/ArmManualMode.cpp 
  src/ArmIKMode.cpp
  src/ArmHelpers.cpp
  src/ArmDummyMode.cpp
  src/ScienceMode.cpp)

# Link the dependencies to the executable
ament_target_dependencies(flightstick_control
  rclcpp
  geometry_msgs
  nav_msgs
  joy
  ros_phoenix
  interfaces
  control_msgs
  std_srvs
  controller_manager_msgs
)

# Install the executable
install(TARGETS
flightstick_control
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
