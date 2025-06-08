#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash
roscore &
sleep 3
rosrun drone_data data_publisher
exec "$@"