#!/usr/bin/env bash

MYPWD=$(pwd)

set -x
set -e

GCC=gcc
GXX=g++

BUILD_TYPE=RelWithDebInfo

if [ -n "$1" ]; then
BUILD_TYPE=$1
fi

# https://stackoverflow.com/a/45181694
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`

NUM_PARALLEL_BUILDS=$((NUM_CORES - 2 < 1 ? 1 : NUM_CORES - 2))

CXX_MARCH=native

EIGEN_DIR="$MYPWD/thirdparty/eigen"

COMMON_CMAKE_ARGS=(
    -DCMAKE_C_COMPILER=${GCC}
    -DCMAKE_CXX_COMPILER=${GXX}
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
    -DCMAKE_CXX_FLAGS="-march=$CXX_MARCH -O3 -Wno-deprecated-declarations -Wno-null-pointer-arithmetic -Wno-unknown-warning-option -Wno-unused-function" #  -Wno-int-in-bool-context
)

BUILD_CERES=thirdparty/build-ceres-solver
BUILD_SPDLOG=thirdparty/build-spdlog
BUILD_OPENMESH=thirdparty/build-openmesh
BUILD_DLIB=thirdparty/build-dlib

git submodule sync --recursive
git submodule update --init --recursive

sudo rm -rf "$BUILD_CERES"
sudo rm -rf "$BUILD_SPDLOG"
sudo rm -rf "$BUILD_OPENMESH"
sudo rm -rf "$BUILD_DLIB"

mkdir -p "$BUILD_SPDLOG"
pushd "$BUILD_SPDLOG"
cmake ../spdlog "${COMMON_CMAKE_ARGS[@]}" \
-DCMAKE_BUILD_TYPE=Release \
-DSPDLOG_BUILD_SHARED=OFF \
-DSPDLOG_FMT_EXTERNAL=OFF \
-DSPDLOG_BUILD_EXAMPLE=OFF \
-DSPDLOG_BUILD_TESTS=OFF \
-DCMAKE_INSTALL_PREFIX=install
make -j$NUM_PARALLEL_BUILDS install
popd

mkdir -p "$BUILD_CERES"
pushd "$BUILD_CERES"
cmake ../ceres-solver "${COMMON_CMAKE_ARGS[@]}" \
    "-DEIGEN_INCLUDE_DIR_HINTS=$EIGEN_DIR" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DEXPORT_BUILD_DIR=ON
make -j$NUM_PARALLEL_BUILDS ceres
popd

mkdir -p "$BUILD_OPENMESH"
pushd "$BUILD_OPENMESH"
cmake ../OpenMesh "${COMMON_CMAKE_ARGS[@]}" 
sudo make -j$NUM_PARALLEL_BUILDS install
popd

mkdir -p "$BUILD_DLIB"
pushd "$BUILD_DLIB"
cmake ../dlib "${COMMON_CMAKE_ARGS[@]}"
sudo make -j$NUM_PARALLEL_BUILDS install
popd
