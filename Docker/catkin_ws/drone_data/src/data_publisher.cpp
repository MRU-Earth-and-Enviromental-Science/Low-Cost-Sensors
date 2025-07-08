#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>

// Global variables
int serial_port = -1;
bool gps_fix_received = false;

// Callback function for GPS data
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Check if the GPS fix is valid
    if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
    {
        gps_fix_received = true;
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "LAT:%.6f,LNG:%.6f,ALT:%.2f\n", 
                 msg->latitude, msg->longitude, msg->altitude);

        if (serial_port != -1)
        {
            int bytes_written = write(serial_port, buffer, strlen(buffer));
            if (bytes_written < 0)
            {
                ROS_ERROR("Failed to write to serial port.");
            }
            else
            {
                ROS_INFO("Sent to ESP32: %s", buffer);
            }
        }
    }
    else
    {
        gps_fix_received = false;
        ROS_WARN("No GPS fix. Status: %d", msg->status.status);
    }
}

// Function to initialize the serial port
void initialize_serial_port(const std::string& port_name, int baud_rate)
{
    serial_port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0)
    {
        ROS_ERROR("Unable to open serial port: %s", port_name.c_str());
        return;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        ROS_ERROR("Failed to get terminal attributes.");
        close(serial_port);
        serial_port = -1;
        return;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        ROS_ERROR("Failed to set terminal attributes.");
        close(serial_port);
        serial_port = -1;
        return;
    }

    ROS_INFO("Serial port initialized: %s at %d baud", port_name.c_str(), baud_rate);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_data_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string serial_port_name;
    int baud_rate;
    private_nh.param("serial_port", serial_port_name, std::string("/dev/ttyAMA0"));
    private_nh.param("baud_rate", baud_rate, 115200);

    initialize_serial_port(serial_port_name, baud_rate);

    ros::Subscriber sub = nh.subscribe("/dji_sdk/gps_position", 10, gpsCallback);
    ros::Rate rate(1); // 1 Hz

    while (ros::ok())
    {
        if (!gps_fix_received && serial_port != -1)
        {
            const char* waiting_msg = "Waiting for GPS fix...\n";
            write(serial_port, waiting_msg, strlen(waiting_msg));
            ROS_WARN("No GPS fix yet. Sending waiting message.");
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
