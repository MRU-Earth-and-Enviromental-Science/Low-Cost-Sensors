#!/bin/bash
set -e

# Source workspaces in order
source /opt/ros/noetic/setup.bash
source /home/shivamwalia/catkin_ws/devel/setup.bash
source /home/shivamwalia/Low-Cost-Sensors/ros_workspace/devel/setup.bash

echo "Launching DJI OSDK ROS node in background..."
roslaunch dji_osdk_ros dji_vehicle_node.launch &

echo "Waiting 3 seconds for DJI node to initialize..."
sleep 3

echo "Launching sensor monitor system..."
roslaunch sensor_monitor full_system.launch