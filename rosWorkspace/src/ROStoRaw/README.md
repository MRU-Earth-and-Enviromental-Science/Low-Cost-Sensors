# ROStoRaw Data Processor Node

This ROS node processes raw sensor data and republishes it on processed topics.

## Build Instructions

1. Navigate to your catkin workspace:
   ```bash
   cd /path/to/your/catkin_workspace
   ```

2. Build the package:
   ```bash
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Running the Node

### Option 1: Using rosrun
```bash
rosrun ROStoRaw data_processor
```

### Option 2: Using the launch file
```bash
roslaunch ROStoRaw data_processor.launch
```

## Topics

### Subscribed Topics:
- `/sensors/temperature` (std_msgs/Float32) - Raw temperature data
- `/sensors/humidity` (std_msgs/Float32) - Raw humidity data
- `/sensors/ch4` (std_msgs/Float32) - Raw CH4 data
- `/sensors/co2` (std_msgs/Float32) - Raw CO2 data
- `/sensors/tvoc` (std_msgs/Float32) - Raw TVOC data
- `/sensors/co` (std_msgs/Float32) - Raw CO data
- `/sensors/nox` (std_msgs/Float32) - Raw NOx data
- `/sensors/pm1_0` (std_msgs/UInt16) - Raw PM1.0 data
- `/sensors/pm2_5` (std_msgs/UInt16) - Raw PM2.5 data
- `/sensors/pm10_0` (std_msgs/UInt16) - Raw PM10.0 data
- `/sensors/status` (std_msgs/String) - Sensor status messages

### Published Topics:
- `/processed/temperature` (std_msgs/Float32) - Processed temperature data
- `/processed/humidity` (std_msgs/Float32) - Processed humidity data
- `/processed/ch4` (std_msgs/Float32) - Processed CH4 data
- `/processed/co2` (std_msgs/Float32) - Processed CO2 data
- `/processed/tvoc` (std_msgs/Float32) - Processed TVOC data
- `/processed/co` (std_msgs/Float32) - Processed CO data
- `/processed/nox` (std_msgs/Float32) - Processed NOx data
- `/processed/pm1_0` (std_msgs/Float32) - Processed PM1.0 data
- `/processed/pm2_5` (std_msgs/Float32) - Processed PM2.5 data
- `/processed/pm10_0` (std_msgs/Float32) - Processed PM10.0 data

## Processing Applied

- **Temperature**: Adds 0.5°C calibration offset
- **Humidity**: Clamps values between 0-100%
- **CH4**: Applies 5% scaling correction (multiply by 1.05)
- **CO2**: Temperature compensation based on current temperature
- **TVOC**: Pass-through (no processing currently)
- **CO**: Applies 2% sensitivity correction (multiply by 0.98)
- **NOx**: Pass-through (no processing currently)
- **PM1.0/PM2.5/PM10.0**: Applies density correction factor (multiply by 1.02)

## Dependencies

- roscpp
- std_msgs
