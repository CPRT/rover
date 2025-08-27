cmake_minimum_required(VERSION 3.5)
project(camera_streaming)

# Set the C++ standard to C++17 (or higher, if needed)
set(CMAKE_CXX_STANDARD 20)

# Find dependencies (ROS2, GStreamer, and PkgConfig)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake REQUIRED)

# Use pkg-config to find the necessary GStreamer libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

add_subdirectory(gst-plugins)

# Include the necessary directories for GStreamer
include_directories(
  include/${PROJECT_NAME}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GST_VIDEO_INCLUDE_DIRS}
)


# Link directories for the necessary libraries
link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  ${GST_VIDEO_LIBRARY_DIRS}
)

# Add the executable target
add_executable(webrtc_node src/webrtc_node.cpp)

# Link with the required libraries
target_link_libraries(webrtc_node
  rclcpp::rclcpp
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
)

# Ensure that the pkg-config variables are used
pkg_check_modules(GLIB REQUIRED glib-2.0)

# Set up any necessary ROS2 specific configurations
ament_target_dependencies(webrtc_node
  rclcpp
  interfaces
  std_srvs
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the executable (optional, depending on your needs)
install(TARGETS webrtc_node DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (if any)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install environment hooks so they are sourced by setup.bash
# The path provided to ament_environment_hooks is relative to the source directory.
# Ensure 'hooks/my_custom_env_hook.sh' exists and is executable.
ament_environment_hooks(
  hooks/gst_env_hook.sh
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()