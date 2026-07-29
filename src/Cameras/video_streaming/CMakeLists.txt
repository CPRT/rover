cmake_minimum_required(VERSION 3.16)
project(video_streaming LANGUAGES CXX)

# ---- Build settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)

add_subdirectory(gst-plugins)

set(DS_PATH /opt/nvidia/deepstream/deepstream)

# ---- Targets
add_library(${PROJECT_NAME} SHARED
  src/base_video_node.cpp
  src/input_node.cpp
  src/detect_node.cpp
  src/webrtc_node.cpp
  src/mosaic_webrtc_node.cpp
  src/srt_node.cpp
  src/srt_node_client.cpp
  src/rtp_node.cpp
  src/rtp_client_node.cpp
  src/capture_node.cpp
)

include_directories(
  include/${PROJECT_NAME}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
  ${DS_PATH}/sources/includes
)

link_directories(${DS_PATH}/lib)

target_link_directories(${PROJECT_NAME} PRIVATE 
  ${GST_LIBRARY_DIRS}
  ${DS_PATH}/lib
)
target_link_libraries(${PROJECT_NAME}
  nvdsgst_meta
  nvds_meta
  PkgConfig::GSTREAMER
  PkgConfig::GST_APP
  PkgConfig::GST_VIDEO
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  interfaces
  std_msgs
)

# If pkg-config provided extra compile flags (e.g., -D, -pthread)
target_compile_options(${PROJECT_NAME} PRIVATE ${GST_CFLAGS_OTHER})

# Register both composable nodes
rclcpp_components_register_nodes(${PROJECT_NAME}
  "InputNode"
  "DetectNode"
  "WebRtcNode"
  "video_streaming::MosaicWebRtcNode"
  "VideoCaptureNode"
  "video_streaming::SrtNode"
  "video_streaming::SrtClientNode"
  "video_streaming::RtpNode"
  "video_streaming::RtpClientNode"
)

# ---- Install
install(
  TARGETS ${PROJECT_NAME}
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

# Install environment hooks so they are sourced by setup.bash
ament_environment_hooks(
  hooks/gst_env_hook.sh
)

# ---- Export
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_components interfaces std_msgs)
ament_export_libraries(${PROJECT_NAME})

ament_package()
