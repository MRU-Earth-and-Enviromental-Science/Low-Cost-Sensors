#!/bin/bash

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Optional: Set serial permissions inside container
chmod 666 /dev/ttyUSB0 || true
chmod 666 /dev/serial0 || true

# Launch OSDK node
roslaunch dji_osdk_ros dji_vehicle_node.launch &
sleep 3

# Launch GPS forwarder node
rosrun UART_Forwarder UART_Forwarder.py