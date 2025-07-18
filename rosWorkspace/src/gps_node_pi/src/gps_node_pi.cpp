#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

void temperatureCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("Temperature: %.2f °C", msg->data);
}

void humidityCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("Humidity: %.2f %%", msg->data);
}

void ch4Callback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("CH4: %.2f ppm", msg->data);
}

void co2Callback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("CO2: %.2f ppm", msg->data);
}

void tvocCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("TVOC: %.2f ppb", msg->data);
}

void coCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("CO: %.2f ppm", msg->data);
}

void noxCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO("NOx: %.2f ppm", msg->data);
}

void pm1Callback(const std_msgs::UInt16::ConstPtr &msg)
{
  ROS_INFO("PM1.0: %d µg/m³", msg->data);
}

void pm25Callback(const std_msgs::UInt16::ConstPtr &msg)
{
  ROS_INFO("PM2.5: %d µg/m³", msg->data);
}

void pm10Callback(const std_msgs::UInt16::ConstPtr &msg)
{
  ROS_INFO("PM10.0: %d µg/m³", msg->data);
}

void statusCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("Sensor Status: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_monitor_node");
  ROS_INFO("Starting Sensor Monitor Node");
  ros::NodeHandle nh;

  // Subscribe to all sensor topics
  ros::Subscriber temp_sub = nh.subscribe("/sensors/temperature", 10, temperatureCallback);
  ros::Subscriber humid_sub = nh.subscribe("/sensors/humidity", 10, humidityCallback);
  ros::Subscriber ch4_sub = nh.subscribe("/sensors/ch4", 10, ch4Callback);
  ros::Subscriber co2_sub = nh.subscribe("/sensors/co2", 10, co2Callback);
  ros::Subscriber tvoc_sub = nh.subscribe("/sensors/tvoc", 10, tvocCallback);
  ros::Subscriber co_sub = nh.subscribe("/sensors/co", 10, coCallback);
  ros::Subscriber nox_sub = nh.subscribe("/sensors/nox", 10, noxCallback);
  ros::Subscriber pm1_sub = nh.subscribe("/sensors/pm1_0", 10, pm1Callback);
  ros::Subscriber pm25_sub = nh.subscribe("/sensors/pm2_5", 10, pm25Callback);
  ros::Subscriber pm10_sub = nh.subscribe("/sensors/pm10_0", 10, pm10Callback);
  ros::Subscriber status_sub = nh.subscribe("/sensors/status", 10, statusCallback);

  ROS_INFO("Sensor Monitor Node ready. Waiting for sensor data...");
  ros::spin();
  return 0;
}
