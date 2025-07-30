#include "../include/dataProcessor.h"
#include <algorithm>
#include <ctime>

DataProcessor::DataProcessor() : nh_("~"), temperature_(25.0), humidity_(50.0), 
                                serial_enabled_(false), baud_rate_(115200), binary_format_(true)
{
    nh_.param<std::string>("serial_port", serial_port_name_, "/dev/ttyUSB1");
    nh_.param<int>("baud_rate", baud_rate_, 115200);
    nh_.param<bool>("enable_serial", serial_enabled_, true);
    nh_.param<bool>("binary_format", binary_format_, true);  // true for binary floats, false for string

    memset(&current_data_, 0, sizeof(current_data_));

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

    if (serial_enabled_) {
        if (initializeSerial()) {
            ROS_INFO("Serial communication initialized on port: %s at %d baud", 
                     serial_port_name_.c_str(), baud_rate_);
            ROS_INFO("Serial format: %s", binary_format_ ? "Binary (float)" : "String (JSON)");
        } else {
            ROS_WARN("Failed to initialize serial communication. Continuing without serial output.");
            serial_enabled_ = false;
        }
    }

    ROS_INFO("Data Processor Node initialized");
}

void DataProcessor::temperatureCallback(const std_msgs::Float32::ConstPtr &msg)
{
    temperature_ = msg->data;

    std_msgs::Float32 processed_msg;
    processed_msg.data = processTemperature(temperature_);
    temp_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.temperature = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed Temperature: %.2f °C (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::humidityCallback(const std_msgs::Float32::ConstPtr &msg)
{
    humidity_ = msg->data;

    std_msgs::Float32 processed_msg;
    processed_msg.data = processHumidity(humidity_);
    humid_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.humidity = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed Humidity: %.2f %% (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::ch4Callback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCH4(msg->data);
    ch4_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.ch4 = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed CH4: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::co2Callback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCO2(msg->data);
    co2_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.co2 = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed CO2: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::tvocCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processTVOC(msg->data);
    tvoc_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.tvoc = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed TVOC: %.2f ppb (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::coCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processCO(msg->data);
    co_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.co = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed CO: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::noxCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std_msgs::Float32 processed_msg;
    processed_msg.data = processNOx(msg->data);
    nox_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.nox = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed NOx: %.2f ppm (Raw: %.2f)", processed_msg.data, msg->data);
}

void DataProcessor::pm1Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM1(raw_value);
    pm1_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.pm1 = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed PM1.0: %.2f µg/m³ (Raw: %d)", processed_msg.data, msg->data);
}

void DataProcessor::pm25Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM25(raw_value);
    pm25_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.pm25 = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

    ROS_INFO("Processed PM2.5: %.2f µg/m³ (Raw: %d)", processed_msg.data, msg->data);
}

void DataProcessor::pm10Callback(const std_msgs::UInt16::ConstPtr &msg)
{
    float raw_value = static_cast<float>(msg->data);

    std_msgs::Float32 processed_msg;
    processed_msg.data = processPM10(raw_value);
    pm10_processed_pub_.publish(processed_msg);

    // Store in current data structure
    current_data_.pm10 = processed_msg.data;
    current_data_.timestamp = static_cast<uint32_t>(ros::Time::now().sec);

    // Send data over serial if enabled
    if (serial_enabled_) {
        sendDataOverSerial();
    }

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

// Serial Communication Implementation
bool DataProcessor::initializeSerial()
{
    try {
        serial_port_.setPort(serial_port_name_);
        serial_port_.setBaudrate(baud_rate_);
        serial_port_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
        serial_port_.open();
        
        if (serial_port_.isOpen()) {
            ROS_INFO("Serial port opened successfully: %s", serial_port_name_.c_str());
            return true;
        } else {
            ROS_ERROR("Failed to open serial port: %s", serial_port_name_.c_str());
            return false;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("Serial port exception: %s", e.what());
        return false;
    }
}

void DataProcessor::sendDataOverSerial()
{
    if (!serial_port_.isOpen()) {
        return;
    }

    try {
        if (binary_format_) {
            // Send binary float data
            std::vector<uint8_t> binary_data = formatDataAsBinary();
            serial_port_.write(binary_data);
        } else {
            // Send string data
            std::string data = formatDataForSerial();
            serial_port_.write(data);
        }
        serial_port_.flushOutput();
    } catch (serial::IOException& e) {
        ROS_ERROR("Serial write error: %s", e.what());
        serial_enabled_ = false;
    }
}

std::string DataProcessor::formatDataForSerial()
{
    std::ostringstream oss;
    
    // Format 1: JSON-like format
    oss << std::fixed << std::setprecision(2);
    oss << "{"
        << "\"timestamp\":" << current_data_.timestamp << ","
        << "\"temperature\":" << current_data_.temperature << ","
        << "\"humidity\":" << current_data_.humidity << ","
        << "\"ch4\":" << current_data_.ch4 << ","
        << "\"co2\":" << current_data_.co2 << ","
        << "\"tvoc\":" << current_data_.tvoc << ","
        << "\"co\":" << current_data_.co << ","
        << "\"nox\":" << current_data_.nox << ","
        << "\"pm1\":" << current_data_.pm1 << ","
        << "\"pm25\":" << current_data_.pm25 << ","
        << "\"pm10\":" << current_data_.pm10
        << "}\n";

    // Alternative Format 2: CSV format (uncomment if preferred)
    /*
    oss << std::fixed << std::setprecision(2);
    oss << current_data_.timestamp << ","
        << current_data_.temperature << ","
        << current_data_.humidity << ","
        << current_data_.ch4 << ","
        << current_data_.co2 << ","
        << current_data_.tvoc << ","
        << current_data_.co << ","
        << current_data_.nox << ","
        << current_data_.pm1 << ","
        << current_data_.pm25 << ","
        << current_data_.pm10 << "\n";
    */

    // Alternative Format 3: NMEA-like format (uncomment if preferred)
    /*
    oss << std::fixed << std::setprecision(2);
    oss << "$SENSOR,"
        << current_data_.timestamp << ","
        << current_data_.temperature << ","
        << current_data_.humidity << ","
        << current_data_.ch4 << ","
        << current_data_.co2 << ","
        << current_data_.tvoc << ","
        << current_data_.co << ","
        << current_data_.nox << ","
        << current_data_.pm1 << ","
        << current_data_.pm25 << ","
        << current_data_.pm10 
        << "*XX\r\n";  // XX would be checksum in real NMEA
    */

    return oss.str();
}

std::vector<uint8_t> DataProcessor::formatDataAsBinary()
{
    std::vector<uint8_t> binary_data;
    
    // Define a packet structure for binary transmission
    struct __attribute__((packed)) SensorPacket {
        uint8_t header[4];        // Packet header: "SENS"
        uint32_t timestamp;       // 4 bytes
        float temperature;        // 4 bytes
        float humidity;          // 4 bytes
        float ch4;               // 4 bytes
        float co2;               // 4 bytes
        float tvoc;              // 4 bytes
        float co;                // 4 bytes
        float nox;               // 4 bytes
        float pm1;               // 4 bytes
        float pm25;              // 4 bytes
        float pm10;              // 4 bytes
        uint16_t checksum;       // 2 bytes CRC
        uint8_t footer[2];       // Packet footer: "\r\n"
    };
    
    SensorPacket packet;
    
    // Fill packet header
    packet.header[0] = 'S';
    packet.header[1] = 'E';
    packet.header[2] = 'N';
    packet.header[3] = 'S';
    
    // Fill data
    packet.timestamp = current_data_.timestamp;
    packet.temperature = current_data_.temperature;
    packet.humidity = current_data_.humidity;
    packet.ch4 = current_data_.ch4;
    packet.co2 = current_data_.co2;
    packet.tvoc = current_data_.tvoc;
    packet.co = current_data_.co;
    packet.nox = current_data_.nox;
    packet.pm1 = current_data_.pm1;
    packet.pm25 = current_data_.pm25;
    packet.pm10 = current_data_.pm10;
    
    // Simple checksum calculation (sum of all data bytes)
    uint16_t checksum = 0;
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&packet.timestamp);
    for (size_t i = 0; i < sizeof(SensorPacket) - sizeof(packet.checksum) - sizeof(packet.footer) - sizeof(packet.header); i++) {
        checksum += data_ptr[i];
    }
    packet.checksum = checksum;
    
    // Fill packet footer
    packet.footer[0] = '\r';
    packet.footer[1] = '\n';
    
    // Convert struct to byte vector
    binary_data.resize(sizeof(SensorPacket));
    std::memcpy(binary_data.data(), &packet, sizeof(SensorPacket));
    
    return binary_data;
}

void DataProcessor::closeSerial()
{
    if (serial_port_.isOpen()) {
        serial_port_.close();
        ROS_INFO("Serial port closed");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_processor");

    DataProcessor processor;

    ROS_INFO("Data Processor Node started. Waiting for sensor data...");

    // Setup signal handler for clean shutdown
    ros::spin();

    // Clean up serial connection
    processor.closeSerial();

    return 0;
}
