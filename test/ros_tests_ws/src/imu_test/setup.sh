#!/usr/bin/env bash

set -e

mkdir -p /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/driver/imu/api
mkdir -p /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/driver/imu/msg
mkdir -p /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/util
ln -s /home/julian/uni/pg/ms02/fastsense/src/util/*.{h,tcc} /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/util
ln -s /home/julian/uni/pg/ms02/fastsense/src/driver/imu/msg/*.{h,cpp} /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/driver/imu/msg
ln -s /home/julian/uni/pg/ms02/fastsense/src/driver/imu/api/*.{h,cpp} /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/driver/imu/api
ln -s /home/julian/uni/pg/ms02/fastsense/src/driver/imu/*.{h,cpp} /home/julian/uni/pg/ms02/test_ws/src/fastsense_imu/fastsense_mirror/src/driver/imu/
