#!/bin/bash

# Python format
black . --exclude "src/third-party/|build|install|\.tox|dist"

# C++ format
find ./src -path ./src/third-party -prune -o \
    \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -print \
  | xargs clang-format -i --Werror

rosdep install --from-paths src -i -r -y

colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)