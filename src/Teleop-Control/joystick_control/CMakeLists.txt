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
<<<<<<< HEAD
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)

=======

#moveit things
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)

>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)
set(CMAKE_CXX_STANDARD 14)

include_directories(include)

<<<<<<< HEAD
=======
add_library(movegroup_action SHARED
src/movegroup_controller.cpp
)
>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)

add_executable(drive
  src/drive.cpp
)

add_executable(arm
  src/arm.cpp
)

<<<<<<< HEAD
add_executable(science
  src/science.cpp
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
=======
add_executable(movegroup_controller
  src/movegroup_controller.cpp
)

target_compile_definitions(movegroup_action
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")


ament_target_dependencies(movegroup_action
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  moveit_core 
  moveit_servo
  interfaces
)

ament_target_dependencies(drive
>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)
  rclcpp
  geometry_msgs
  joy
  ros_phoenix
  interfaces
  control_msgs
  std_srvs
)

<<<<<<< HEAD
ament_target_dependencies(science
  rclcpp
  joy
  ros_phoenix
  interfaces
  std_msgs
)

install(TARGETS
  drive
  arm
  science
=======
ament_target_dependencies(arm
  rclcpp
  geometry_msgs
  joy
  ros_phoenix
  interfaces
  control_msgs
  std_srvs
  rclcpp_action
)

ament_target_dependencies(
  movegroup_controller
  interfaces
  "moveit_ros_planning_interface"
  "moveit_core"
  "rclcpp"
  rclcpp_action
)


install(TARGETS
  drive
  arm
  movegroup_controller
>>>>>>> cc5fb9f (feat: implement modeless teleop and move_group action server)
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
