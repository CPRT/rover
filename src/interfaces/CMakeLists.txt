cmake_minimum_required(VERSION 3.5)
project(interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoBoard.msg"
  "msg/ArucoMarkers.msg"
  "msg/Distance.msg"
  "msg/EspSensorReadings.msg"
  "msg/GasSensorReading.msg"
  "msg/MoveGroupStatus.msg"
  "msg/ObjectDetected.msg"
  "msg/PolarimeterSweep.msg"
  "msg/PwmCommand.msg"
  "msg/RtpStats.msg"
  "msg/SrtStats.msg"
  "msg/SvinStatus.msg"
  "msg/SystemTelemetry.msg"
  "msg/VideoSource.msg"
  "srv/FilterGoals.srv"
  "srv/GetCameras.srv"
  "srv/GetNamedTargets.srv"
  "srv/GoToCamCoord.srv"
  "srv/GoToNamedPose.srv"
  "srv/GoToPose.srv"
  "srv/GraphDot.srv"
  "srv/NavToGPSGeopose.srv"
  "srv/RunPolarimeter.srv"
  "srv/SaveCurrentPose.srv"
  "srv/VideoCapture.srv"
  "srv/VideoOut.srv"
  DEPENDENCIES 
  builtin_interfaces
  geometry_msgs 
  geographic_msgs
  sensor_msgs
  std_msgs
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()