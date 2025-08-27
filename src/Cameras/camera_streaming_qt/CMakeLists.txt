cmake_minimum_required(VERSION 3.16)

project(camera_streaming_qt VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

# Include directories required for GStreamer
include_directories(
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
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

set(SOURCES
    src/mainwindow.cpp
    src/widgets/capture_image_widget.cpp
    src/widgets/main_widget.cpp
    src/widgets/preset_widget.cpp
    src/widgets/source_widget.cpp
)

set(HEADERS
    include/${PROJECT_NAME}/camera_client.h
    include/${PROJECT_NAME}/mainwindow.h
    include/${PROJECT_NAME}/tests/test_camera_node.h
    include/${PROJECT_NAME}/widgets/capture_image_widget.h
    include/${PROJECT_NAME}/widgets/main_widget.h
    include/${PROJECT_NAME}/widgets/preset_widget.h
    include/${PROJECT_NAME}/widgets/source_widget.h
)

# Camera client node
add_executable(camera_client_node src/camera_client.cpp ${SOURCES} ${HEADERS})

# Test nodes
add_executable(test_camera_node src/tests/test_camera_node.cpp)

# Link with the required libraries
target_link_libraries(
    camera_client_node 
    rclcpp::rclcpp
    Qt${QT_VERSION_MAJOR}::Widgets
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
)

target_link_libraries(
 test_camera_node
 rclcpp::rclcpp
)

ament_target_dependencies(camera_client_node 
    rclcpp 
    std_msgs
    interfaces
    std_srvs
)

ament_target_dependencies(test_camera_node
    rclcpp 
    std_msgs
    interfaces
    std_srvs
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
    camera_client_node
    test_camera_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
