# gst-plugins/ros2image/CMakeLists.txt

find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GST_INCLUDE_DIRS}
  ${GST_BASE_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})

add_library(gstros2 SHARED
  plugin.cpp
  gstros2src.cpp
  gstros2overlay.cpp
  gstros2common.cpp
)
target_compile_definitions(gstros2 PRIVATE
  PACKAGE="\\\"gstros2\\\""
  VERSION="\\\"1.1\\\""
  PACKAGE_NAME="\\\"gstros2\\\""
  PACKAGE_ORIGIN="\\\"https://github.com/CPRT/rover/tree/main/src/Cameras/video_streaming\\\""
)

target_link_libraries(gstros2
  ${GST_LIBRARIES}
  ${GST_BASE_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Remove 'lib' prefix for GStreamer plugin naming
set_target_properties(gstros2 PROPERTIES PREFIX "")

# Optional: install to the package's lib dir
install(TARGETS gstros2
  LIBRARY DESTINATION lib/gstreamer-1.0
)


