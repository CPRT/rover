cmake_minimum_required(VERSION 3.8)
project(cprt_behavior_tree_plugins)

# Default to C++14 for ROS 2 Humble compatibility
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

set(dependencies
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  nav2_util
  rclcpp
)

add_library(cprt_remove_in_collision_goals_bt_node SHARED
  src/remove_in_collision_goals_action.cpp
)

ament_target_dependencies(cprt_remove_in_collision_goals_bt_node
  ${dependencies}
)

install(TARGETS cprt_remove_in_collision_goals_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cprt_remove_in_collision_goals_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
