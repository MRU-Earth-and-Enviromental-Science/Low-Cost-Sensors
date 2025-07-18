#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"

ros::Publisher gps_pub;
ros::Publisher health_pub;

bool espConnected = true;

bool espConnection() {
  return gps_pub.getNumSubscribers() > 0 && health_pub.getNumSubscribers() > 0;
}

bool droneConnection(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  return msg->status.status >= 0 && !std::isnan(msg->latitude) && !std::isnan(msg->longitude);
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  sensor_msgs::NavSatFix clean_msg = *msg;

  if (!droneConnection(msg)) {
    clean_msg.latitude = 10.0;
    clean_msg.longitude = 10.0;
    clean_msg.altitude = 10.0;
    ROS_WARN("Drone GPS not valid. Sending zeros.");
  }

  if (espConnection()) {
    gps_pub.publish(clean_msg);
    espConnected = true;
  } else {
    if (espConnected) {
      ROS_WARN("ESP32 disconnected — GPS data not sent.");
      espConnected = false;
    }
  }

  ROS_INFO("Lat: %f, Lon: %f, Alt: %f", clean_msg.latitude, clean_msg.longitude, clean_msg.altitude);
}

void gpsHealthCallback(const std_msgs::UInt8::ConstPtr &msg)
{
  if (espConnection()) {
    health_pub.publish(*msg);
    espConnected = true;
  } else {
    if (espConnected) {
      ROS_WARN("ESP32 disconnected — GPS health not sent.");
      espConnected = false;
    }
  }

  ROS_INFO("GPS Health: %d", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_node");
  ROS_INFO("Starting GPS Node");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("/dji_osdk_ros/gps_position", 10, gpsCallback);
  ros::Subscriber health_sub = nh.subscribe("/dji_osdk_ros/gps_health", 10, gpsHealthCallback);

  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_node/gps_position", 10);
  health_pub = nh.advertise<std_msgs::UInt8>("/gps_node/gps_health", 10);

  ros::spin();
  return 0;
}
