cmake_minimum_required(VERSION 3.8)
project(system-telemetry-cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/system-telemetry-cpp)
add_executable(system_telemetry_publisher
  src/system_telemetry_publisher.cpp
  src/cpu_collector.cpp
  src/memory_collector.cpp
  src/gpu_collector.cpp
)

ament_target_dependencies(system_telemetry_publisher
  rclcpp
  interfaces
)


install(TARGETS
  system_telemetry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
