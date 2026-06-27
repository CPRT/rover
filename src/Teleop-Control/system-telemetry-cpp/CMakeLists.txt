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
find_package(std_msgs REQUIRED)

find_path(NETSNMP_INCLUDE_DIR net-snmp/net-snmp-config.h)
find_library(NETSNMP_LIBRARY netsnmp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/system-telemetry-cpp
  ${NETSNMP_INCLUDE_DIR}
)

add_executable(system_telemetry_publisher
  src/system_telemetry_publisher.cpp
  src/cpu_collector.cpp
  src/memory_collector.cpp
  src/gpu_collector.cpp
)

add_executable(node_status_publisher
  src/node_status_publisher.cpp
)

ament_target_dependencies(system_telemetry_publisher
  rclcpp
  interfaces
)

ament_target_dependencies(node_status_publisher
  rclcpp
  std_msgs
)

add_executable(snmp_network_stats
  src/snmp_network_stats.cpp
)

ament_export_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/system-telemetry-cpp
  ${NETSNMP_INCLUDE_DIR}
)

ament_target_dependencies(snmp_network_stats
  rclcpp
  std_msgs
)

target_link_libraries(snmp_network_stats
  ${NETSNMP_LIBRARY}
)

install(TARGETS
  system_telemetry_publisher
  snmp_network_stats
  node_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()