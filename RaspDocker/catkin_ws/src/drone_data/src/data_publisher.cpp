#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

int serial_port;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "LAT:%.6f LNG:%.6f\n", msg->latitude, msg->longitude);
    write(serial_port, buffer, strlen(buffer));
    ROS_INFO("Sent to ESP32: %s", buffer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_data_node");
    ros::NodeHandle nh;

    // Open serial port to ESP32
    serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port < 0)
    {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_port, TCSANOW, &tty);

    ros::Subscriber sub = nh.subscribe("/dji_sdk/gps_position", 10, gpsCallback);
    ros::spin();

    close(serial_port);
    return 0;
}