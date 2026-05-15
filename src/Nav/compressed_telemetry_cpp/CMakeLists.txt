cmake_minimum_required(VERSION 3.8)
project(compressed_telemetry_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ZLIB REQUIRED)

add_library(compressed_telemetry_cpp_nodes SHARED
  src/CostmapCompressor.cpp
  src/CostmapDecompressor.cpp
  src/GridmapCompressor.cpp
  src/GridmapDecompressor.cpp
  src/TFTelemetry.cpp
  src/TFRelay.cpp
)

target_include_directories(compressed_telemetry_cpp_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(compressed_telemetry_cpp_nodes PUBLIC cxx_std_17)

ament_target_dependencies(compressed_telemetry_cpp_nodes
  rclcpp
  rclcpp_components
  nav_msgs
  map_msgs
  std_msgs
  grid_map_msgs
  tf2_msgs
  geometry_msgs
)

target_link_libraries(compressed_telemetry_cpp_nodes
  ZLIB::ZLIB
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::CostmapCompressor"
  EXECUTABLE costmap_compressor_node
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::CostmapDecompressor"
  EXECUTABLE costmap_decompressor_node
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::GridmapCompressor"
  EXECUTABLE gridmap_compressor_node
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::GridmapDecompressor"
  EXECUTABLE gridmap_decompressor_node
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::TFTelemetry"
  EXECUTABLE tf_telemetry_node
)

rclcpp_components_register_node(compressed_telemetry_cpp_nodes
  PLUGIN "compressed_telemetry_cpp::TFRelayNode"
  EXECUTABLE tf_relay_node
)

install(TARGETS
  compressed_telemetry_cpp_nodes
  costmap_compressor_node
  costmap_decompressor_node
  gridmap_compressor_node
  gridmap_decompressor_node
  tf_telemetry_node
  tf_relay_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
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
