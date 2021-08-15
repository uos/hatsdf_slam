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

make clean_software

echo "$INFO Test target 'software'"
LOGS='test/test_software.log' && touch "$LOGS"
make software &> "$LOGS"

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"

sleep 1

make clean_tests

echo "$INFO Test target 'test_sensor_sync'"
LOGS='test/test_sensor_sync.log' && touch "$LOGS"
make test_sensor_sync HW_TARGET=sw_emu &> ""$LOGS""

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"

sleep 1

echo "$INFO Test target 'test_zmq_client'"
LOGS='test/test_zmq_client.log' && touch "$LOGS"
make test_zmq_client HW_TARGET=sw_emu &> "$LOGS"

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"

sleep 1

echo "$INFO Test target 'test_hdf5'"
LOGS='test/test_hdf5.log' && touch "$LOGS"
make test_hdf5 HW_TARGET=sw_emu &> "$LOGS"

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"

sleep 1

echo "$INFO Test target 'test_global_map'"
LOGS='test/test_global_map.log' && touch "$LOGS"
make test_global_map HW_TARGET=sw_emu &> "$LOGS"

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"

sleep 1

echo "$INFO Test ROS test nodes"
LOGS='test/ros_test_nodes.log' && touch "$LOGS"
make clean_ros_nodes ros_test_nodes &> "$LOGS"

if [ ! $? -eq 0 ]; then echo "$FAIL" && exit 1; fi
echo "${SUCCESS}"