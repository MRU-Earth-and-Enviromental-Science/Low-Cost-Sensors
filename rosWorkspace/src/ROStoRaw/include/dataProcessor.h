#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cstring>

class DataProcessor
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber temp_sub_;
    ros::Subscriber humid_sub_;
    ros::Subscriber ch4_sub_;
    ros::Subscriber co2_sub_;
    ros::Subscriber tvoc_sub_;
    ros::Subscriber co_sub_;
    ros::Subscriber nox_sub_;
    ros::Subscriber pm1_sub_;
    ros::Subscriber pm25_sub_;
    ros::Subscriber pm10_sub_;
    ros::Subscriber status_sub_;

    ros::Publisher temp_processed_pub_;
    ros::Publisher humid_processed_pub_;
    ros::Publisher ch4_processed_pub_;
    ros::Publisher co2_processed_pub_;
    ros::Publisher tvoc_processed_pub_;
    ros::Publisher co_processed_pub_;
    ros::Publisher nox_processed_pub_;
    ros::Publisher pm1_processed_pub_;
    ros::Publisher pm25_processed_pub_;
    ros::Publisher pm10_processed_pub_;

    float temperature_;
    float humidity_;

    serial::Serial serial_port_;
    std::string serial_port_name_;
    int baud_rate_;
    bool serial_enabled_;
    bool binary_format_;  // true for binary float, false for string

    struct SensorData {
        float temperature;
        float humidity;
        float ch4;
        float co2;
        float tvoc;
        float co;
        float nox;
        float pm1;
        float pm25;
        float pm10;
        uint32_t timestamp;
    } current_data_;

public:
    DataProcessor();
    ~DataProcessor() = default;

    void temperatureCallback(const std_msgs::Float32::ConstPtr &msg);
    void humidityCallback(const std_msgs::Float32::ConstPtr &msg);
    void ch4Callback(const std_msgs::Float32::ConstPtr &msg);
    void co2Callback(const std_msgs::Float32::ConstPtr &msg);
    void tvocCallback(const std_msgs::Float32::ConstPtr &msg);
    void coCallback(const std_msgs::Float32::ConstPtr &msg);
    void noxCallback(const std_msgs::Float32::ConstPtr &msg);
    void pm1Callback(const std_msgs::UInt16::ConstPtr &msg);
    void pm25Callback(const std_msgs::UInt16::ConstPtr &msg);
    void pm10Callback(const std_msgs::UInt16::ConstPtr &msg);
    void statusCallback(const std_msgs::String::ConstPtr &msg);

    bool initializeSerial();
    void sendDataOverSerial();
    std::string formatDataForSerial();
    std::vector<uint8_t> formatDataAsBinary();
    void closeSerial();

private:
    float processTemperature(float raw_temp);
    float processHumidity(float raw_humidity);
    float processCH4(float raw_ch4);
    float processCO2(float raw_co2);
    float processTVOC(float raw_tvoc);
    float processCO(float raw_co);
    float processNOx(float raw_nox);
    float processPM1(float raw_pm1);
    float processPM25(float raw_pm25);
    float processPM10(float raw_pm10);
};

#endif
