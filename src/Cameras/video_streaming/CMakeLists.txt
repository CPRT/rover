cmake_minimum_required(VERSION 3.16)
project(video_streaming)

# ---- Build settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
# find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)

# 1. GStreamer and GLib
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)
pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET glib-2.0)


# ---- Build Libraries
add_library(video_streaming_nodes SHARED
  src/base_video_node.cpp
  src/input_node.cpp
  src/detect_node.cpp
  src/webrtc_node.cpp
  src/srt_node.cpp
)


target_include_directories(video_streaming_nodes PUBLIC
  "include" 
  "include/${PROJECT_NAME}" 
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
)


target_link_libraries(video_streaming_nodes
  PkgConfig::GSTREAMER
  PkgConfig::GST_APP
  PkgConfig::GST_VIDEO
  PkgConfig::GLIB
)


ament_target_dependencies(video_streaming_nodes
  rclcpp
  rclcpp_components
  std_msgs
  interfaces
  # std_srvs
)


# Compiler flags form pkg-config
target_compile_options(video_streaming_nodes PRIVATE ${GST_CFLAGS_OTHER} ${GLIB_CFLAGS_OTHER})

# Register Component
rclcpp_components_register_nodes(video_streaming_nodes 
  "InputNode"
  "DetectNode"
  "WebRtcNode"
  "video_streaming::SrtNode"
)

# ---- Install
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

# ament_environment_hooks(hooks/gst_env_hook.sh)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  # std_srvs

)
ament_export_libraries(video_streaming_nodes) 
ament_package()