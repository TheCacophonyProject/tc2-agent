#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail
set -o xtrace

if [[ $# -lt 1 ]]; then
  echo "please provide target host. Usage \`deploy.sh target_host [--deb, --wifi]\`"
  exit 1
fi

TARGET_HOST=$1
readonly TARGET_PATH=/home/pi/tc2-agent
readonly SOURCE_PATH=./target/aarch64-unknown-linux-musl/release/tc2-agent
readonly DEB_SOURCE_DIR=./target/aarch64-unknown-linux-musl/debian/
readonly TARGET_ARCH=aarch64-unknown-linux-musl

DEB_OPTION=false
WIFI_OPTION=false

# Iterate over the arguments, starting from index 2
for i in "${@:2}"; do
  case $i in
    --deb)
      DEB_OPTION=true
      ;;
    --wifi)
      WIFI_OPTION=true
      ;;
    *)
      echo "Unknown option: $i"
      exit 1
      ;;
  esac
done

cargo build --release --target=${TARGET_ARCH}

if [ "$DEB_OPTION" = true ]; then
  cargo deb --target=${TARGET_ARCH}
  deb=$(cd ${DEB_SOURCE_DIR} && ls *.deb)
  echo $deb
  scp ${DEB_SOURCE_DIR}${deb} ${TARGET_HOST}:
  ssh -t ${TARGET_HOST} sudo dpkg -i ${deb}

else
  scp ${SOURCE_PATH} ${TARGET_HOST}:${TARGET_PATH}
  if [ "$WIFI_OPTION" = true ]; then
    # NOTE: To start in wifi frame serve mode:
    ssh -t ${TARGET_HOST} sudo chrt -f 99 ${TARGET_PATH} --use-wifi
  else
    ssh -t ${TARGET_HOST} sudo chrt -f 99 ${TARGET_PATH}
  fi
fi
