cmake_minimum_required(VERSION 3.5)
project(science_sensors LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../ros_odrive/odrive_base/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}
  "../ros_odrive/odrive_base/src/epoll_event_loop.cpp"
  "../ros_odrive/odrive_base/src/socket_can.cpp"
  "src/science_node.cpp"
  "src/main.cpp"
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "interfaces"
)

# Install Components
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include/
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()