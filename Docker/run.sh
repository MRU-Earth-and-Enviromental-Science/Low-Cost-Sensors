#!/bin/bash

set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash

# Function to check for roscore
check_roscore() {
    if ! rostopic list > /dev/null 2>&1; then
        echo "[ERROR] roscore not running or not accessible."
        exit 1
    fi
}

# Handle shutdown gracefully
cleanup() {
    echo "[INFO] Shutting down ROS nodes..."
    if [ -n "$DJI_SDK_NODE_PID" ]; then
        kill $DJI_SDK_NODE_PID
    fi
    if [ -n "$DATA_PUBLISHER_PID" ]; then
        kill $DATA_PUBLISHER_PID
    fi
    if [ -n "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID
    fi
    wait $DJI_SDK_NODE_PID $DATA_PUBLISHER_PID $ROSCORE_PID 2>/dev/null
    echo "[INFO] Shutdown complete."
}

trap cleanup SIGINT SIGTERM EXIT

# Start roscore in the background
echo "[INFO] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5 # Give roscore time to initialize

# Verify roscore is running
check_roscore

# Launch DJI SDK node
echo "[INFO] Starting dji_sdk_node..."
rosrun dji_sdk dji_sdk_node &
DJI_SDK_NODE_PID=$!
sleep 5 # Give the node time to initialize

# Launch your data publisher node
echo "[INFO] Starting data_publisher..."
rosrun drone_data data_publisher &
DATA_PUBLISHER_PID=$!

# Keep the container running
echo "[INFO] All nodes launched. Monitoring..."
tail -f /dev/null
