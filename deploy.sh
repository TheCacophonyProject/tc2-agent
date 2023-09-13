#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail
set -o xtrace

# Set the name of your development raspberry pi here
readonly TARGET_HOST=pi@pi4
#readonly TARGET_HOST=pi@cp-rpi
#readonly TARGET_HOST=pi@p2
readonly TARGET_PATH=/home/pi/tc2-agent
readonly SOURCE_PATH=./target/armv7-unknown-linux-musleabihf/release/tc2-agent
readonly TARGET_ARCH=armv7-unknown-linux-musleabihf

cargo build --release --target=${TARGET_ARCH}
rsync ${SOURCE_PATH} ${TARGET_HOST}:${TARGET_PATH}
# Run on raspberry pi with realtime priority

# NOTE: To start in wifi frame serve mode with a custom spi speed:
# ssh -t ${TARGET_HOST} sudo chrt -f 99 ${TARGET_PATH} --use-wifi --spi-speed 10
ssh -t ${TARGET_HOST} sudo chrt -f 99 ${TARGET_PATH} --use-wifi
