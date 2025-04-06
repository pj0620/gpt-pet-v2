#!/bin/bash

export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=~/sysroot  # This should be the manually copied Raspberry Pi root filesystem
export ROS2_INSTALL_PATH=/home/gptpetclient2/gpt-pet-v2/ros2_ws/src/real_ws/install
export PYTHON_SOABI=cpython-310-aarch64-linux-gnu

export CMAKE_PREFIX_PATH="$SYSROOT/usr"
export OpenCV_DIR="$SYSROOT/usr/share/opencv4"

# ROS 2
export CMAKE_FIND_ROOT_PATH="$ROS2_INSTALL_PATH;$SYSROOT"

export CMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER
export CMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY
export CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY
export CMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY

export TinyXML2_DIR="../../../tinyxml2-quickstart/tinyxml2"
# export CMAKE_PREFIX_PATH="$TinyXML2_DIR:$CMAKE_PREFIX_PATH"
# export AMENT_PREFIX_PATH="$TinyXML2_DIR:$AMENT_PREFIX_PATH"

# $SYSROOT/opt/ros/humble/share

export THREADS_PTHREAD_ARG="0" 

cd ros2_ws/src/real_ws
colcon build \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_FIND_ROOT_PATH="$TinyXML2_DIR;$ROS2_INSTALL_PATH;$SYSROOT" \
        -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
        -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY \
        -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
        -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY \
        -DTinyXML2_DIR="$TinyXML2_DIR" \
        -Dtinyxml2_DIR="$TinyXML2_DIR" \
        -DCMAKE_PREFIX_PATH="$TinyXML2_DIR;$SYSROOT/usr/lib/aarch64-linux-gnu/cmake;$SYSROOT/opt/ros/humble/share" \
        -DOpenCV_DIR="$SYSROOT/usr/share/opencv4" \
        -DPYTHON_SOABI=$PYTHON_SOABI \
        -DTHREADS_PTHREAD_ARG=$THREADS_PTHREAD_ARG \
        -DCMAKE_TOOLCHAIN_FILE="../../../cross_compile/cmake-toolchains/generic_linux.cmake"
