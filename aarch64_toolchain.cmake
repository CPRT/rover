
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()