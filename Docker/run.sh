#!/bin/bash

set -e

ENV_FILE="/home/dji/.env"
if [ -f "$ENV_FILE" ]; then
    echo "[INFO] Loading environment variables from $ENV_FILE"
    export $(grep -v '^#' "$ENV_FILE" | xargs)
else
    echo "[ERROR] .env file not found at $ENV_FILE"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "[INFO] Starting roscore..."
roscore &
ROSCORE_PID=$!

sleep 5

echo "[INFO] Starting dji_sdk_node..."
rosrun dji_sdk dji_sdk_node \
    _app_id:=$APP_ID \
    _app_key:=$APP_KEY \
    _serial_name:=$SERIAL_DEVICE \
    _baud_rate:=$BAUD_RATE \
    _core_connection:=true &
DJI_SDK_NODE_PID=$!

sleep 5

echo "[INFO] Starting drone_data_node..."
rosrun drone_data drone_data_node &
DRONE_DATA_NODE_PID=$!

echo "[INFO] All ROS nodes started. Container will remain active."

cleanup() {
    echo "[INFO] Shutting down ROS nodes..."
    kill $DRONE_DATA_NODE_PID
    kill $DJI_SDK_NODE_PID
    kill $ROSCORE_PID
    wait $DRONE_DATA_NODE_PID $DJI_SDK_NODE_PID $ROSCORE_PID 2>/dev/null
    echo "[INFO] Shutdown complete."
}
trap cleanup SIGINT SIGTERM EXIT

tail -f /dev/null