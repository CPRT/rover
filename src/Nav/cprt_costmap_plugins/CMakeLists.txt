cmake_minimum_required(VERSION 3.5)
project(cprt_costmap_plugins)

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

set(library_name ${PROJECT_NAME}_core)

# Get a linker error when there are undefined symbols
add_link_options(-Wl,--no-undefined)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${tf2_sensor_msgs_INCLUDE_DIRS}
    ${grid_map_core_INCLUDE_DIRS}
    ${grid_map_ros_INCLUDE_DIRS}
    ${grid_map_msgs_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
            src/gridmap_layer.cpp
)

target_link_libraries(${library_name}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${library_name}
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
  builtin_interfaces
  pluginlib
  grid_map_core
  grid_map_ros
  grid_map_msgs
  nav2_costmap_2d
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${library_name}
        DESTINATION lib)

ament_export_dependencies(rosidl_default_runtime)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()