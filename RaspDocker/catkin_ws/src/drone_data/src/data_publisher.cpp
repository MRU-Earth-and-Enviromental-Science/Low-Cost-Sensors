#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "drone_data_node");
    NodeHandle nh;

    Publisher pub = nh.advertise<std_msgs::Float32>("co2_ppm", 10);
    Rate rate(1);
    float dummy = 400.0;

    while (ok())
    {
        std_msgs::Float32 msg;
        msg.data = dummy;
        ROS_INFO("CO2 Reading: %f", msg.data);
        pub.publish(msg);
        dummy += 0.1;
        rate.sleep();
    }

    return 0;
}