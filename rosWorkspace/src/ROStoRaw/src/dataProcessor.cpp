#include "../include/dataProcessor.h"
#include <algorithm>

DataProcessor::DataProcessor() : nh_("~"), temperature_(25.0), humidity_(50.0)
{
    temp_sub_ = nh_.subscribe("/sensors/temperature", 10, &DataProcessor::temperatureCallback, this);
    humid_sub_ = nh_.subscribe("/sensors/humidity", 10, &DataProcessor::humidityCallback, this);
    ch4_sub_ = nh_.subscribe("/sensors/ch4", 10, &DataProcessor::ch4Callback, this);
    co2_sub_ = nh_.subscribe("/sensors/co2", 10, &DataProcessor::co2Callback, this);
    tvoc_sub_ = nh_.subscribe("/sensors/tvoc", 10, &DataProcessor::tvocCallback, this);
    co_sub_ = nh_.subscribe("/sensors/co", 10, &DataProcessor::coCallback, this);
    nox_sub_ = nh_.subscribe("/sensors/nox", 10, &DataProcessor::noxCallback, this);
    pm1_sub_ = nh_.subscribe("/sensors/pm1_0", 10, &DataProcessor::pm1Callback, this);
    pm25_sub_ = nh_.subscribe("/sensors/pm2_5", 10, &DataProcessor::pm25Callback, this);
    pm10_sub_ = nh_.subscribe("/sensors/pm10_0", 10, &DataProcessor::pm10Callback, this);
    status_sub_ = nh_.subscribe("/sensors/status", 10, &DataProcessor::statusCallback, this);

    temp_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/temperature", 10);
    humid_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/humidity", 10);
    ch4_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/ch4", 10);
    co2_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/co2", 10);
    tvoc_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/tvoc", 10);
    co_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/co", 10);
    nox_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/nox", 10);
    pm1_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/pm1_0", 10);
    pm25_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/pm2_5", 10);
    pm10_processed_pub_ = nh_.advertise<std_msgs::Float32>("/processed/pm10_0", 10);

    ROS_INFO("Data Processor Node initialized");
}

void DataProcessor::temperatureCallback(const std_msgs::Float32::ConstPtr &msg)
{
    temperature_ = msg->data;

    std_msgs::Float32 processed_msg;
    processed_msg.data = processTemperature(temperature_);
    temp_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed Temperature: %.2f °C (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::humidityCallback(const std_msgs::Float32::ConstPtr &msg)
{
    humidity_ = msg->data;

    std_msgs::Float32 processed_msg;
    processed_msg.data = processHumidity(humidity_);
    humid_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed Humidity: %.2f %% (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::ch4Callback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCH4(msg->data);
    ch4_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed CH4: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::co2Callback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCO2(msg->data);
    co2_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed CO2: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::tvocCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processTVOC(msg->data);
    tvoc_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed TVOC: %.2f ppb (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::coCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCO(msg->data);
    co_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed CO: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::noxCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processNOx(msg->data);
    nox_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed NOx: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::pm1Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM1(raw_value);
    pm1_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed PM1.0: %.2f µg/m³ (Raw: %d)", processed_msg.data, msg->data);
}

void DataProcessor::pm25Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM25(raw_value);
    pm25_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed PM2.5: %.2f µg/m³ (Raw: %d)", processed_msg.data, msg->data);
}

void DataProcessor::pm10Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM10(raw_value);
    pm10_processed_pub_.publish(processed_msg);

    ROS_INFO("Processed PM10.0: %.2f µg/m³ (Raw: %d)", processed_msg.data, msg->data);
}

void DataProcessor::statusCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Sensor Status: %s", msg->data.c_str());
}

// Data processing functions implementation
float DataProcessor::processTemperature(float raw_temp)
{
    // Apply calibration offset and convert to raw float
    return raw_temp + 0.5; // Add 0.5°C calibration offset
}

float DataProcessor::processHumidity(float raw_humidity)
{
    // Clamp humidity between 0-100% and return raw float
    return std::max(0.0f, std::min(100.0f, raw_humidity));
}

float DataProcessor::processCH4(float raw_ch4)
{
    // Apply scaling factor and return raw float
    return raw_ch4 * 1.05; // 5% scaling correction
}

float DataProcessor::processCO2(float raw_co2)
{
    // Temperature compensation (simplified)
    float temp_compensation = 1.0 + (temperature_ - 25.0) * 0.001;
    return raw_co2 * temp_compensation;
}

float DataProcessor::processTVOC(float raw_tvoc)
{
    // Convert units or apply filter - return raw float
    return raw_tvoc; // Pass through for now
}

float DataProcessor::processCO(float raw_co)
{
    // Apply sensitivity correction
    return raw_co * 0.98; // 2% sensitivity correction
}

float DataProcessor::processNOx(float raw_nox)
{
    // Apply environmental correction - return raw float
    return raw_nox; // Pass through for now
}

float DataProcessor::processPM1(float raw_pm1)
{
    // Apply density correction and return raw float
    return raw_pm1 * 1.02; // Density correction factor
}

float DataProcessor::processPM25(float raw_pm25)
{
    // Apply density correction and return raw float
    return raw_pm25 * 1.02; // Density correction factor
}

float DataProcessor::processPM10(float raw_pm10)
{
    // Apply density correction and return raw float
    return raw_pm10 * 1.02; // Density correction factor
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_processor");

    DataProcessor processor;

    ROS_INFO("Data Processor Node started. Waiting for sensor data...");

    ros::spin();

    return 0;
}
