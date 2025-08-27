cmake_minimum_required(VERSION 3.8)
project(cprt_gridmap_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

pkg_check_modules(TBB "tbb")
if(TBB_FOUND)
add_definitions(
  ${TBB_CFLAGS}
  ${TBB_CFLAGS_OTHER}
)
else()
message([FATAL_ERROR] "tbb module not found")
endif()

grid_map_package()

set(dependencies
  TBB
  filters
  grid_map_core
  grid_map_msgs
  grid_map_ros
  filters
  pluginlib
)

set(filter_libs
  preserve_cost_inflation_filter
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${TBB_INCLUDE_DIRS}
    ${filters_INCLUDE_DIRS}
)

## Declare cpp libraries
add_library(preserve_cost_inflation_filter SHARED src/PreserveCostInflationFilter.cpp)

foreach(lib_name ${filter_libs})
  ament_target_dependencies(${lib_name} SYSTEM
    ${dependencies}
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${filter_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # Linting is setup this way to add a filter
  # to ament_cpplint to ignore the lack of
  # copyright messages at the top of files.
  # Copyright messages are being checked for by both
  # ament_cmake_cpplint & ament_cmake_copyright.

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # exclude copyright checks
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_cpplint
      ament_cmake_copyright
      ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()

    # run cpplint without copyright filter
    find_package(ament_cmake_cpplint)
    ament_cpplint(
      FILTERS -legal/copyright -build/include_order
    )

    # run uncrustify except for EigenLab.hpp
    find_package(ament_cmake_uncrustify)
    set(
      _linter_excludes
      include/EigenLab/EigenLab.hpp
    )
    ament_uncrustify(
      EXCLUDE ${_linter_excludes}
      LANGUAGE c++
    )
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
  ${TBB_INCLUDE_DIRS}
)
ament_export_dependencies(${dependencies})
ament_export_libraries(${filter_libs})
pluginlib_export_plugin_description_file(filters filter_plugins.xml)
ament_package()
