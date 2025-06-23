#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>

int serial_port;
bool gps_received = false;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps_received = true;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "LAT:%.6f LNG:%.6f\n", msg->latitude, msg->longitude);
    if (serial_port != -1)
    {
        write(serial_port, buffer, strlen(buffer));
    }
    ROS_INFO("Sent to ESP32: %s", buffer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_data_node");
    ros::NodeHandle nh;

    serial_port = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (serial_port < 0)
    {
        perror("Warning: Unable to open serial port. Continuing without serial output.");
        serial_port = -1;
    }
    // initalizing the port, setting baud rate and other parameters
    ROS_INFO("Serial port opened successfully.");

    struct termios tty;
    tcgetattr(serial_port, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem control lines and enable receiver
    tty.c_cflag &= ~CSIZE;           // Clear the current data size setting
    tty.c_cflag |= CS8;              // 8 data bits
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;         // No hardware flow control
    tty.c_lflag = 0;                 // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                 // No remapping, no delays
    tty.c_cc[VMIN] = 0;              // No minimum number of characters to read
    tty.c_cc[VTIME] = 10;            // 1 second timeout for read operations
    tcsetattr(serial_port, TCSANOW, &tty);

    ros::Subscriber sub = nh.subscribe("/dji_sdk/gps_position", 10, gpsCallback); // Subscribe to GPS position topic
    ros::Rate rate(1);                                                            // 1 Hz rate

    while (ros::ok())
    {
        if (!gps_received && serial_port != -1)
        {
            const char *default_msg = "LAT:-1.000000 LNG:-1.000000\n";
            write(serial_port, default_msg, strlen(default_msg));
            ROS_WARN("No GPS fix yet. Sending default value.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    if (serial_port != -1)
    {
        close(serial_port);
    }

    return 0;
}
