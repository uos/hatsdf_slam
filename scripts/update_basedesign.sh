#!/usr/bin/env bash

set -Eeuo pipefail

url=$1
call_dir=${PWD}

[[ ! -d "${call_dir}/base_design/" ]] && echo "Please run in fastsense dir" && exit 1

echo "Downloading Base Design"

wget $1
mkdir -p "${call_dir}/base_design/platform/FastSense_platform/export/"
cd base_design/platform/FastSense_platform/export/
rm -rf *
wget "$url" -O FastSense_platform.tar.gz
tar -zxvf FastSense_platform.tar.gz && rm FastSense_platform.tar.gz

cd "${call_dir}"
