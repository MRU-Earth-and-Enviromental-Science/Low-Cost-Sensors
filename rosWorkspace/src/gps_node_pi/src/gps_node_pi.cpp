#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  ROS_INFO("Lat: %f, Lon: %f, Alt: %f", msg->latitude, msg->longitude, msg->altitude);
}

void gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg) {
  ROS_INFO("GPS Health: %d", msg->data);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_node");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("/dji_osdk_ros/gps_position", 10, gpsCallback);
  ros::Subscriber health_sub = nh.subscribe("/dji_osdk_ros/gps_health", 10, gpsHealthCallback);

  ros::spin();
  return 0;
}
