#!/bin/bash

# Check for help flag
if [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
    echo "Usage: ./build.sh [colcon build options]"
    echo ""
    echo "This script formats code, installs dependencies, and builds the ROS2 workspace."
    echo ""
    echo "Examples:"
    echo "  ./build.sh"
    echo "  ./build.sh --parallel-workers 3"
    echo "  ./build.sh --packages-select navigation localization gps bringup"
    echo "  ./build.sh --packages-select navigation --parallel-workers 1"
    echo ""
    exit 0
fi

# Ensure script is run from the workspace root
if [ ! -d "src" ]; then
    echo "Error: 'src' directory not found. This script must be run from the workspace root."
    exit 1
fi

# Python format
black . --exclude "src/third-party/|build|install|\.tox|dist"

# C++ format
find ./src -path ./src/third-party -prune -o \
    \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -print \
  | xargs clang-format -i --Werror

rosdep install --from-paths src -i -r -y

# Pass all arguments directly to colcon build with sensible defaults
colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$(nproc)" "$@"
