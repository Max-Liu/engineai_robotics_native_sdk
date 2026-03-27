#!/bin/bash

# Exits on error
set -e

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
mujoco_dir="$root_dir/simulation/mujoco"
deps_archive="$mujoco_dir/mujoco_deps_x86.tar.xz"
deps_dir="$mujoco_dir/_deps"

has_local_mujoco_deps() {
  [ -f "$deps_dir/glfw3-src/CMakeLists.txt" ] && [ -f "$deps_dir/lodepng-src/lodepng.cpp" ]
}

if ! has_local_mujoco_deps && [ -f "$deps_archive" ]; then
  echo "Extracting local MuJoCo deps from $deps_archive ..."
  tar -xf "$deps_archive" -C "$mujoco_dir"
fi

cmake_args=(-DBUILD_RELEASE=ON)
if has_local_mujoco_deps; then
  echo "Using local MuJoCo deps from $deps_dir"
  cmake_args+=(-DMUJOCO_USE_LOCAL_DEPS=ON)
fi

# Builds the project
build_dir="$mujoco_dir/build"
mkdir -p $build_dir && cd $build_dir
cmake "${cmake_args[@]}" ..

# Compiles with 2 threads less than the number of cores
num_cores=$(expr $(nproc) - 2)
if [ $num_cores -lt 1 ]; then
  num_cores=$(nproc)
fi
make -j$num_cores

