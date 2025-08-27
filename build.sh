#!/bin/bash

# Find all files tracked by git that are not part of a submodule
files=$(git ls-files -- ':!:(*/)' ':!:(*/**/*)' ':!:(*/**/*/*)')

# Filter out python files into one variable
python_files=$(
  for file in $files; do
    if [[ $file == *.py ]]; then
      echo "$file"
    fi
  done
)

# Launch one instance of `black` for all python files
IFS=$'\n' black $python_files

# Format C/C++ source & header files
for file in $files; do
  if [[ $file == *.cpp || $file == *.hpp || $file == *.c || $file == *.h ]]; then
    clang-format -i "$file"
  fi
done

rosdep install --from-paths src -i -r -y

aarch=$(uname -m)

colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)