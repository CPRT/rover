#!/bin/bash
rootPath=$(pwd)

# Run build script to generate compile_commands.json for each CMake project
#./build.sh

# Find all the CMake projects that have compile_commands.json
# After, put into a Bash list using mapfile, so it can be iterated
# Use -o -path to exclude directories
mapfile -t projectPaths < <(find $rootPath \
    \( -path "*/src/third-party" -o -path "*/src/Nav/kindr_ros" -o -path "*/build" \) \
    -prune -o -name "CMakeLists.txt" -printf '%h\n')

# Perform clang-tidy on each project
for projectPath in "${projectPaths[@]}"; do
    echo "Running clang-tidy on: $projectPath"

    currDirectoryName="${projectPath##*/}"

    cd $projectPath

    # Use the build directory as the build path since that directory has the compile_commands.json file
    run-clang-tidy -p $rootPath/build/$currDirectoryName
done