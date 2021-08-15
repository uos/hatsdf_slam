#!/usr/bin/env bash

if [[ ! "$OSTYPE" -eq "linux-gnu" ]]; then echo "WARNING: this script is tested on Ubuntu 18.04 only"; fi

set -e

CPPZMQ_VERSION="4.4.1"
CPPZMQ_ZIP="cppzmq-${CPPZMQ_VERSION}.zip" 

LIBZMQ_VERSION="4.3.2"
LIBZMQ_ZIP="cppzmq-${LIBZMQ_VERSION}.zip" 


wget https://github.com/zeromq/libzmq/releases/download/v"${LIBZMQ_VERSION}"/zeromq-"${LIBZMQ_VERSION}".zip -O "$LIBZMQ_ZIP"
unzip "$LIBZMQ_ZIP" && cd "zeromq-${LIBZMQ_VERSION}"

# patch for this specific version! See https://github.com/zeromq/libzmq/issues/3056
#wget https://github.com/zeromq/libzmq/raw/v4.2.5/builds/cmake/clang-format-check.sh.in -O builds/cmake/clang-format-check.sh.in

mkdir build && cd build
cmake .. && sudo make -j4 install
cd ../..

wget https://github.com/zeromq/cppzmq/archive/v"${CPPZMQ_VERSION}".zip -O "$CPPZMQ_ZIP"
unzip "$CPPZMQ_ZIP" && cd "cppzmq-${CPPZMQ_VERSION}"
mkdir build && cd build
cmake -DCPPZMQ_BUILD_TESTS=OFF .. && sudo make -j4 install
cd ../..

sudo apt install libhdf5-dev ros-melodic-imu-filter-madgwick 

rm -rf *mq-*



