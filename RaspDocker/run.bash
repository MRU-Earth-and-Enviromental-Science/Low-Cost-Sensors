#!/bin/bash

# Source ROS setup files
source /opt/ros/noetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash

# Start roscore in the background
roscore &
ROSCORE_PID=$!

# Wait for roscore to start
echo "Waiting for roscore..."
sleep 5

# Run the data publisher node
rosrun drone_data data_publisher

# Optional: Kill roscore when done
kill $ROSCORE_PID