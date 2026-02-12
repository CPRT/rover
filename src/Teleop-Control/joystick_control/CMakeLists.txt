cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#moveit things
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_library(movegroup_action SHARED
src/movegroup_controller.cpp
)

add_executable(drive
  src/drive.cpp
)

add_executable(arm
  src/arm.cpp
)

add_executable(movegroup_controller
  src/movegroup_controller.cpp
)

target_compile_definitions(movegroup_action
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")


ament_target_dependencies(movegroup_action
  rclcpp
  rclcpp_action
  rclcpp_components
)

ament_target_dependencies(drive
  rclcpp
  geometry_msgs
  joy
  ros_phoenix
  interfaces
  control_msgs
  std_srvs
)

ament_target_dependencies(arm
  rclcpp
  geometry_msgs
  joy
  ros_phoenix
  interfaces
  control_msgs
  std_srvs
)

ament_target_dependencies(
  movegroup_controller
  interfaces
  "moveit_ros_planning_interface"
  "moveit_core"
  "rclcpp"
)


install(TARGETS
  drive
  arm
  movegroup_controller
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
#install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
