#!/bin/bash
rootPath=$(pwd)

# Run build script to generate compile_commands.json for each CMake project
./build.sh

# List of excluded projects
# TODO: Read these projects from a file
excludedProjects=("third-party" "elevation_mapping" "interfaces" "kindr_msgs" "kindr_ros" "ouster_ros" "ouster_sensor_msgs" "ublox" "ublox_gps" "ublox_msgs" "ublox_serialization" "zed_components" "zed_ros2" "zed_wrapper")
excludedProjectsArgs=()

# Format the excluded projects into an argument list that is read by find
for excludedProject in "${excludedProjects[@]}"; do
    excludedProjectsArgs+=("-not" "-path" "*/$excludedProject/*")
done

# Concatenate all compile_commands.json files into one file
find build -name compile_commands.json -exec jq '.[]' {} + \
    | jq -s '.' > build/compile_commands.json

# Run clang-tidy
# The find command filters the excluded projects out
# All .cpp files in the included projects are linted (header files are also implicitly linted when included in a source file)
run-clang-tidy -p build $(find src -type f -name "*.cpp" "${excludedProjectsArgs[@]}") -extra-arg-before=-std=c++17 -j $(nproc)