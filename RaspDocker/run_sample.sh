#!/bin/bash

source /home/dji/.bashrc

echo "Running DJI OSDK Linux Sample..."
cd /home/dji/Onboard-SDK/build/bin

./djiosdk-flightcontrol-sample /home/dji/Onboard-SDK/sample/platform/linux/common/UserConfig.txt