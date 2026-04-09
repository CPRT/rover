cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)

set(CMAKE_CXX_STANDARD 14)
include_directories(include)

add_executable(drive src/drive.cpp)
add_executable(arm src/arm.cpp)
add_executable(science src/science.cpp)
add_executable(movegroup_controller src/movegroup_controller.cpp)

ament_target_dependencies(drive 
  rclcpp geometry_msgs joy ros_phoenix interfaces control_msgs std_srvs
)

ament_target_dependencies(science 
  rclcpp joy ros_phoenix interfaces std_msgs
)

ament_target_dependencies(arm 
  rclcpp rclcpp_action geometry_msgs joy ros_phoenix interfaces control_msgs std_srvs
)

ament_target_dependencies(movegroup_controller 
  rclcpp rclcpp_action interfaces moveit_ros_planning_interface moveit_core moveit_servo
)

install(TARGETS 
  drive 
  arm 
  science 
  movegroup_controller 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()