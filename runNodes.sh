#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/Low-Cost-Senors/rosWorkspace/devel/setup.bash
source ~/rosSerial_ws/devel/setup.bash

DJI_LAUNCH_LOG="/tmp/dji_osdk_ros.log"
FAILURE_STRING="cannot connect with drone block at here"

publish_simulated_gps() {
  echo "[INFO] Starting GPS simulation via rostopic pub..."
  rostopic pub -r 1 /dji_osdk_ros/gps_position sensor_msgs/NavSatFix "{
    header: {frame_id: 'base_link'}, 
    status: {status: 0, service: 1}, 
    latitude: 43.6532, 
    longitude: -79.3832, 
    altitude: 150.0, 
    position_covariance: [0,0,0,0,0,0,0,0,0], 
    position_covariance_type: 0
  }"
}

launch_dji_osdk() {
  echo "[INFO] Launching DJI OSDK node..."
  gnome-terminal -- bash -c "roslaunch dji_osdk_ros dji_osdk_ros.launch > $DJI_LAUNCH_LOG 2>&1; exec bash" &
  DJI_PID=$!

  echo "[INFO] Monitoring DJI OSDK logs for failure..."

  # Use tail to monitor logs
  tail -Fn0 "$DJI_LAUNCH_LOG" | while read -r line; do
    echo "$line"
    if echo "$line" | grep -q "$FAILURE_STRING"; then
      echo "[ERROR] Detected DJI handshake failure."
      kill $DJI_PID 2>/dev/null || true
      publish_simulated_gps
      exit 0
    fi
  done

  wait $DJI_PID
}

launch_dji_osdk &

sleep 10

echo "[INFO] Launching GPS node..."
gnome-terminal -- bash -c "rosrun gps_node_pi gps_node; exec bash"
sleep 3

echo "[INFO] Connecting to ESP32 via rosserial..."
gnome-terminal -- bash -c "rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0; exec bash"

echo "[SUCCESS] All ROS nodes launched!"

wait