#!/usr/bin/env bash

X_PATH="/media/julian/ssdext/xilinx"
X_VERSION="2019.2"
ROS_VERSION="melodic"
TICK="[✓]"
CROSS="[✗]"
INFO="[i]"
SUCCESS="${TICK} done"
FAIL="${CROSS} FAIL!"

. "${X_PATH}/Vitis/${X_VERSION}/settings64.sh" && . "/opt/ros/${ROS_VERSION}/setup.bash" && . /opt/xilinx/xrt/setup.sh &> /dev/null

compile_test() {
    local TARGET="$1"
    echo "$INFO Compile target '$TARGET'"
    LOG_OUT="./test/${TARGET}.log" && touch "$LOG_OUT"
    make "$TARGET" &> "$LOG_OUT"

    if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
    echo "${SUCCESS}" && sleep 1
}

ros_compile_test() {
    echo "$INFO Compile ROS test nodes"
    LOGS='test/ros_test_nodes.log' && touch "$LOGS"
    make clean_ros_nodes ros_test_nodes &> "$LOGS"
    if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
    echo "${SUCCESS}" && sleep 1
}

make clean_software

compile_test software 

compile_test test_sensor_sync

compile_test test_receiver

compile_test test_sender

compile_test test_global_map

ros_compile_test