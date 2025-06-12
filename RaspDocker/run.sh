#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

roscore &
ROSCORE_PID=$!

sleep 5

rosrun dji_osdk_ros dji_osdk_ros_node \
  _app_id:=${APP_ID:-1163880} \
  _app_key:=${APP_KEY:-559e776a2a67e8e2e0f015842813272eed634a81a0df29cd25d364ea04738303} \
  _serial_name:=/dev/ttyUSB0 \
  _baud_rate:=115200 \
  _core_connection:=true &
DJI_NODE_PID=$!

sleep 5

rosrun drone_data data_publisher &
DATA_PUB_PID=$!

wait $DATA_PUB_PID
