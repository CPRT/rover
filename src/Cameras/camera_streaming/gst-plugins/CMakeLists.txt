# gstreamer_plugins/CMakeLists.txt

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})

add_library(gstarucomarker SHARED
  gstarucomarker.cpp
)
target_compile_definitions(gstarucomarker PRIVATE
  PACKAGE="\\\"gstarucomarker\\\""
  VERSION="\\\"1.0\\\""
  PACKAGE_NAME="\\\"gstarucomarker\\\""
  PACKAGE_ORIGIN="\\\"https://github.com/yourname/gstarucomarker\\\""
)

target_link_libraries(gstarucomarker
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
)

# Remove 'lib' prefix for GStreamer plugin naming
set_target_properties(gstarucomarker PROPERTIES PREFIX "")

# Optional: install to the package's lib dir
install(TARGETS gstarucomarker
  LIBRARY DESTINATION lib/gstreamer-1.0
)


