#!/bin/bash

# Ensure script is run from the workspace root
if [ ! -d "src" ]; then
    echo "Error: 'src' directory not found. This script must be run from the workspace root."
    exit 1
fi

# Defaults
PARALLEL_WORKERS=$(nproc)
PACKAGES_SELECT=()

# Argument parsing
while [[ $# -gt 0 ]]; do
  case $1 in
    -w|--parallel-workers)
      PARALLEL_WORKERS="$2"
      shift 2
      ;;
    -s|--packages-select)
      PACKAGES_SELECT=()
      shift
      # Consumes args until the next flag (starts with -) or end of args
      while [[ $# -gt 0 ]] && ! [[ "$1" =~ ^- ]]; do
        PACKAGES_SELECT+=("$1")
        shift
      done
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Python format
black . --exclude "src/third-party/|build|install|\.tox|dist"

# C++ format
find ./src -path ./src/third-party -prune -o \
    \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -print \
  | xargs clang-format -i --Werror

rosdep install --from-paths src -i -r -y

# Run colcon build based on whether specific packages were selected
if [ ${#PACKAGES_SELECT[@]} -eq 0 ]; then
    colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $PARALLEL_WORKERS
else
    colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $PARALLEL_WORKERS --packages-select "${PACKAGES_SELECT[@]}"
fi