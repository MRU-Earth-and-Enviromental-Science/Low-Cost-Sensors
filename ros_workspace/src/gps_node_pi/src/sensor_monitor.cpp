#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"

ros::Publisher gps_pub;
ros::Publisher health_pub;

bool esp_connected = true;

bool esp_connection()
{
  return gps_pub.getNumSubscribers() > 0 && health_pub.getNumSubscribers() > 0;
}

bool drone_connection(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  return msg->status.status >= 0 && !std::isnan(msg->latitude) && !std::isnan(msg->longitude);
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  sensor_msgs::NavSatFix clean_msg = *msg;

  if (!drone_connection(msg))
  {
    clean_msg.latitude = 10.0;
    clean_msg.longitude = 10.0;
    clean_msg.altitude = 10.0;
    ROS_WARN("Drone GPS not valid. Sending zeros.");
  }

  if (esp_connection())
  {
    gps_pub.publish(clean_msg);
    esp_connected = true;
  }
  else
  {
    if (esp_connected)
    {
      ROS_WARN("ESP32 disconnected — GPS data not sent.");
      esp_connected = false;
    }
  }

  ROS_INFO("Lat: %f, Lon: %f, Alt: %f", clean_msg.latitude, clean_msg.longitude, clean_msg.altitude);
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  if (esp_connection())
  {
    health_pub.publish(*msg);
    esp_connected = true;
  }
  else
  {
    if (esp_connected)
    {
      ROS_WARN("ESP32 disconnected — GPS health not sent.");
      esp_connected = false;
    }
  }

  ROS_INFO("GPS Health: %d", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_node");
  ROS_INFO("Starting GPS Node");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("/dji_osdk_ros/gps_position", 10, gps_callback);
  ros::Subscriber health_sub = nh.subscribe("/dji_osdk_ros/gps_health", 10, gps_health_callback);

  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_node/gps_position", 10);
  health_pub = nh.advertise<std_msgs::UInt8>("/gps_node/gps_health", 10);

  ros::spin();
  return 0;
}
