#!/bin/bash
# This script builds the SDK and gets it running for the system. Refer to the README for pre-requisites.

set -e

ROS_RELEASE="noetic"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

run_cmd() {
    echo -e "${YELLOW}==> $*${NC}"
    eval "$@"
}

run_cmd sudo usermod -a -G dialout "$USER"

echo -e "${YELLOW}==> Creating udev rule for DJI device${NC}"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2ca3", MODE="0666", GROUP="dialout"' | sudo tee /etc/udev/rules.d/DJIDevice.rules

run_cmd sudo apt-get update
run_cmd sudo apt-get install -y "ros-${ROS_RELEASE}-nmea-comms"
run_cmd sudo apt-get install -y libavcodec-dev libswresample-dev libusb-1.0-0-dev

if [ -d "Onboard-SDK" ]; then
    echo -e "${YELLOW}==> Building Onboard-SDK${NC}"
    cd Onboard-SDK
    run_cmd mkdir -p build
    cd build
    run_cmd cmake ..
    run_cmd sudo make -j3 install
    cd ../..
else
    echo -e "${RED}Onboard-SDK directory not found!${NC}"
    exit 1
fi

echo -e "${YELLOW}==> Sourcing ROS setup.bash${NC}"
source "/opt/ros/${ROS_RELEASE}/setup.bash"

if [ -d "$HOME/catkin_ws" ]; then
    echo -e "${YELLOW}==> Building catkin workspace${NC}"
    cd "$HOME/catkin_ws"
    run_cmd catkin_make -j3
else
    echo -e "${RED}~/catkin_ws directory not found!${NC}"
    exit 1
fi

echo -e "${GREEN}SDK installation and build complete.${NC}"
if [ -d "$HOME/Low-Cost-Sensors/ros_workspace" ]; then
    echo -e "${YELLOW}==> Building Low-Cost-Sensors ros_workspace${NC}"
    cd "$HOME/Low-Cost-Sensors/ros_workspace"
    run_cmd catkin_init_workspace
    run_cmd catkin_make -j3
else
    echo -e "${RED}~/Low-Cost-Sensors/ros_workspace directory not found!${NC}"
    exit 1
fi


