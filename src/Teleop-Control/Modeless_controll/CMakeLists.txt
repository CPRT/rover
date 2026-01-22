cmake_minimum_required(VERSION 3.8)
project(Modeless_controll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
 # Required for your MoveServo service
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(driveModeless src/driveModeless.cpp)
add_executable(armManualModeless 
  src/armManualModeless.cpp
  src/ArmHelpers.cpp
)
target_include_directories(driveModeless PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driveModeless PUBLIC c_std_99 cxx_std_17)
target_include_directories(armManualModeless PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(armManualModeless PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(driveModeless
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

ament_target_dependencies(armManualModeless
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

install(TARGETS
 driveModeless
 armManualModeless
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
