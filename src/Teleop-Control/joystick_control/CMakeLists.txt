cmake_minimum_required(VERSION 3.8)
project(joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)


add_executable(drive
  src/drive.cpp
)

add_executable(arm
  src/arm.cpp
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

install(TARGETS
  drive
  arm
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
