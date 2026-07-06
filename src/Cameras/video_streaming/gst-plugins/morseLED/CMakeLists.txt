# gst-plugins/morseLED/CMakeLists.txt

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GST_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})

add_library(gstmorseLED SHARED
  plugin.cpp
  gstmorse_led.cpp
  gstmorse_led_sim.cpp
  morse_decoder.cpp
)

target_compile_definitions(gstmorseLED PRIVATE
  PACKAGE="\\\"gstmorseLED\\\""
  VERSION="\\\"1.0\\\""
  PACKAGE_NAME="\\\"gstmorseLED\\\""
  PACKAGE_ORIGIN="\\\"https://github.com/CPRT/rover/tree/main/src/Cameras/video_streaming\\\""
)

target_link_libraries(gstmorseLED
  ${GST_LIBRARIES}
)

set_target_properties(gstmorseLED PROPERTIES PREFIX "")

install(TARGETS gstmorseLED
  LIBRARY DESTINATION lib/gstreamer-1.0
)
