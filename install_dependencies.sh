#!/usr/bin/env bash

set -e

# should work for Ubuntu 16.04 and 18.04
sudo apt-get install -y \
    build-essential \
    gdb \
    cmake \
    git \
    libtbb-dev \
    libeigen3-dev \
    libglew-dev \
    ccache \
    libjpeg-dev \
    libpng-dev \
    openssh-client \
    liblz4-dev \
    libbz2-dev \
    libboost-regex-dev \
    libboost-filesystem-dev \
    libboost-date-time-dev \
    libboost-program-options-dev \
    libopencv-dev \
    libpython2.7-dev \
    gfortran \
    libc++-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    wget \
    valgrind \
    clang-format \
    mesa-common-dev \
    libgl1-mesa-dev \
    qt5-default \
    qt5-qmake \
    libblas-dev \
    liblapack-dev
