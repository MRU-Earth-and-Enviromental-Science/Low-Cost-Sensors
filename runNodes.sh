#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# Fix the typo in the path
source ~/Low-Cost-Sensors/rosWorkspace/devel/setup.bash

DJI_LAUNCH_LOG="/tmp/dji_osdk_ros.log"
FAILURE_STRING="Cannot connect with drone, block at here"
SUCCESS_STRING="status : 3"

# Function to publish simulated GPS data
publish_simulated_gps() {
  echo "[INFO] Starting simulated GPS publisher..."
  rostopic pub -r 1 /dji_osdk_ros/gps_position sensor_msgs/NavSatFix \
    '{header: {frame_id: "base_link"}, status: {status: 0, service: 1}, latitude: 43.6532, longitude: -79.3832, altitude: 150.0, position_covariance: [0,0,0,0,0,0,0,0,0], position_covariance_type: 0}' &
  GPS_SIM_PID=$!
  echo "[INFO] Simulated GPS publisher started with PID: $GPS_SIM_PID"
}

# Function to try launching DJI OSDK
launch_dji_osdk() {
  echo "[INFO] Attempting to launch DJI OSDK..."
  echo "[INFO] DJI OSDK output will be shown below:"
  echo "=================================="
  
  # Launch DJI OSDK and capture output to both terminal and log file
  roslaunch dji_osdk_ros dji_osdk_ros.launch 2>&1 | tee $DJI_LAUNCH_LOG &
  DJI_PID=$!
  
  echo "[INFO] Monitoring DJI OSDK for 15 seconds..."
  sleep 15
  
  # Check if DJI process is still running and if it succeeded
  if kill -0 $DJI_PID 2>/dev/null; then
    if [ -f "$DJI_LAUNCH_LOG" ]; then
      if grep -q "$FAILURE_STRING" "$DJI_LAUNCH_LOG"; then
        echo "[ERROR] DJI handshake failed. Killing DJI process and starting GPS simulation..."
        kill $DJI_PID 2>/dev/null || true
        publish_simulated_gps
        return 1
      elif grep -q "$SUCCESS_STRING" "$DJI_LAUNCH_LOG"; then
        echo "[INFO] DJI OSDK started successfully!"
        return 0
      else
        echo "[INFO] DJI OSDK appears to be running (no failure detected)"
        return 0
      fi
    else
      echo "[INFO] DJI OSDK appears to be running (log file not found)"
      return 0
    fi
  else
    echo "[ERROR] DJI OSDK process died. Starting GPS simulation..."
    publish_simulated_gps
    return 1
  fi
}

# Function to handle cleanup on exit
cleanup() {
  echo "[INFO] Cleaning up background processes..."
  [ ! -z "$DJI_PID" ] && kill $DJI_PID 2>/dev/null || true
  [ ! -z "$GPS_PID" ] && kill $GPS_PID 2>/dev/null || true
  [ ! -z "$ESP32_PID" ] && kill $ESP32_PID 2>/dev/null || true
  [ ! -z "$GPS_SIM_PID" ] && kill $GPS_SIM_PID 2>/dev/null || true
  exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Clear any existing log file
rm -f "$DJI_LAUNCH_LOG"

# Start roscore if not already running
if ! pgrep -f "roscore" > /dev/null; then
  echo "[INFO] Starting roscore..."
  roscore &
  ROSCORE_PID=$!
  sleep 3
else
  echo "[INFO] roscore is already running"
fi

# Launch DJI OSDK (or GPS simulation on failure)
launch_dji_osdk

echo "[INFO] Launching GPS node..."
rosrun gps_node_pi gps_node &
GPS_PID=$!
echo "[INFO] GPS node started with PID: $GPS_PID"

sleep 3

echo "[INFO] Launching rosserial for ESP32..."
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 &
ESP32_PID=$!
echo "[INFO] ESP32 serial node started with PID: $ESP32_PID"

echo "[SUCCESS] All ROS nodes launched successfully and running in background!"
echo "[INFO] Process IDs:"
[ ! -z "$DJI_PID" ] && echo "  DJI OSDK: $DJI_PID"
[ ! -z "$GPS_SIM_PID" ] && echo "  GPS Simulation: $GPS_SIM_PID"
echo "  GPS Node: $GPS_PID"
echo "  ESP32 Serial: $ESP32_PID"
echo ""
echo "[INFO] All nodes are now running independently in the background"
echo "[INFO] To stop all processes, run: pkill -f 'roslaunch\|rosrun\|rostopic'"
echo "[INFO] Script exiting - nodes will continue running"