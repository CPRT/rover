cmake_minimum_required(VERSION 3.16)
project(video_streaming LANGUAGES CXX)

# ---- Build settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)

add_subdirectory(gst-plugins)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStreamingParams.srv"
  "msg/Srtstat.msg"
)


add_library(video_streaming_nodes SHARED
  src/base_video_node.cpp
  src/input_node.cpp
  src/detect_node.cpp
  src/webrtc_node.cpp
  src/srt_node.cpp
)


target_include_directories(video_streaming_nodes PUBLIC
  "include" 
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
)

target_link_libraries(video_streaming_nodes
  PkgConfig::GSTREAMER
  PkgConfig::GST_APP
  PkgConfig::GST_VIDEO
)

ament_target_dependencies(video_streaming_nodes
  rclcpp
  rclcpp_components
  std_srvs
  std_msgs
  rosidl_default_runtime
)
rosidl_target_dependencies(video_streaming_nodes

  ${${PROJECT_NAME}_INSTALL_DIR}/share/${PROJECT_NAME}/cmake/rosidl_generator_c

  ${${PROJECT_NAME}_INSTALL_DIR}/share/${PROJECT_NAME}/cmake/rosidl_generator_cpp

) 


# If pkg-config provided extra compile flags (e.g., -D, -pthread)
target_compile_options(video_streaming_nodes PRIVATE ${GST_CFLAGS_OTHER})

rclcpp_components_register_nodes(video_streaming_nodes 
  "InputNode"
  "DetectNode"
  "WebRtcNode"
  "SrtNode"
)


install(
  TARGETS video_streaming_nodes 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_environment_hooks(
  hooks/gst_env_hook.sh
)


ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  rosidl_default_runtime
)
ament_export_libraries(video_streaming_nodes) 
ament_package()

