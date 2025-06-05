#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import serial


def gps_callback(msg):
    if ser is None:
        return
    data_str = f"{msg.latitude:.6f},{msg.longitude:.6f}\n"
    ser.write(data_str.encode('utf-8'))


if __name__ == '__main__':
    rospy.init_node('uart_forwarder', anonymous=True)
    try:
        ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port: {e}")
        ser = None

    rospy.Subscriber('/dji_osdk_ros/gps_position', NavSatFix, gps_callback)
    rospy.spin()
    if ser:
        ser.close()
