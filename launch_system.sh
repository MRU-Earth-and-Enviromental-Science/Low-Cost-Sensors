#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/Low-Cost-Sensors/ros_workspace/devel/setup.bash

roslaunch dji_osdk_ros dji_vehicle_node.launch &
sleep 2
exec roslaunch sensor_monitor full_system.launch