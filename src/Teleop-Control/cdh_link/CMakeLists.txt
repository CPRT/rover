cmake_minimum_required(VERSION 3.5)
project(cdh_link LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}
  "src/science_node.cpp"
  "src/main.cpp"
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "interfaces"
)

# Install Components
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include/
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()