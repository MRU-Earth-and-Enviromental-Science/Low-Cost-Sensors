#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Wire.h>
#include "../include/OLED.h"
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/K30.h"
#include "../include/SGP.h"
#include "../include/NOx.h"
#include "../include/PM25.h"

Plantower_PMS7003 pms7003;

bool senderConnected = false;

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// Objects
Adafruit_SGP30 sgp;
bool sgp_initialized = false;

// Pi to ESP32 UART - COMMENTED OUT: Using USB-C port for ROS communication
// constexpr int RX_PIN = 4;
// constexpr int TX_PIN = 5;
// constexpr uint32_t BAUD_UART = 115200;

String lineBuffer;

// MAC Address receiver
uint8_t broadcastAddress[] = {0x88, 0x13, 0xbf, 0x82, 0x19, 0x94}; // Replace with correct receiver MAC
//88:13:bf:82:19:94
// Struct to hold sensor data
typedef struct __attribute__((packed))
{
    float temp;
    float humid;
    float ch4;
    float co2;
    float tvoc;
    float co;
    float nox;
    uint16_t pm_1_0;
    uint16_t pm_2_5;
    uint16_t pm_10_0;
    float lat;
    float lon;
} SensorData;

// Global sensor data object
SensorData sensorData;

// Add ROS connection tracking
bool ros_connected = false;
unsigned long last_connection_check = 0;

ros::NodeHandle nh;

sensor_msgs::NavSatFix gps_data;
std_msgs::UInt8 gps_health;

void gpsCallback(const sensor_msgs::NavSatFix &msg)
{
    gps_data = msg;
}

void gpsHealthCallback(const std_msgs::UInt8 &msg)
{
    gps_health = msg;
}

ros::Subscriber<sensor_msgs::NavSatFix> gps_sub("/gps_node/gps_position", gpsCallback);
ros::Subscriber<std_msgs::UInt8> health_sub("/gps_node/gps_health", gpsHealthCallback);

// static double dmToDeg(const String &dm, char hemi)
// {
//     if (dm.length() < 4)
//         return 0.0;
//     double val = dm.toDouble();
//     int deg = int(val / 100);
//     double min = val - deg * 100;
//     double dec = deg + min / 60.0;
//     if (hemi == 'S' || hemi == 'W')
//         dec = -dec;
//     return dec;
// }

// void readPiData()
// {
//     while (Serial2.available())
//     {
//         char c = Serial2.read();

//         if (c == '\n' || c == '\r')
//         {
//             if (lineBuffer.length())
//             {
//                 Serial.println("RAW: " + lineBuffer);

//                 if (lineBuffer.startsWith("$GPRMC"))
//                 {
//                     int idx = 0;
//                     String fields[12];
//                     for (int i = 0; i < 12 && idx != -1; ++i)
//                     {
//                         int next = lineBuffer.indexOf(',', idx);
//                         fields[i] = (next == -1) ? lineBuffer.substring(idx)
//                                                  : lineBuffer.substring(idx, next);
//                         idx = (next == -1) ? -1 : next + 1;
//                     }

//                     if (fields[2] == "A")
//                     {
//                         String latStr = fields[3];
//                         char latHem = fields[4].charAt(0);
//                         String lonStr = fields[5];
//                         char lonHem = fields[6].charAt(0);

//                         double lat = dmToDeg(latStr, latHem);
//                         double lon = dmToDeg(lonStr, lonHem);

//                         sensorData.lat = (float)lat;
//                         sensorData.lon = (float)lon;

//                         Serial.printf("✓ GPS  Lat: %.6f  Lon: %.6f\n", lat, lon);
//                     }
//                 }
//             }
//             lineBuffer = "";
//         }
//         else if (isPrintable(c))
//         {
//             lineBuffer += c;
//         }
//     }
// }

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Serial.print("ESP-NOW send status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    if (!senderConnected && status == ESP_NOW_SEND_SUCCESS)
    {
        senderConnected = true;
        oledPrint("ESP-NOW Connected");
    }
}

void readSensors()
{
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h))
        h = -1.0;
    if (isnan(t))
        t = -1.0;
    int co2ppm = -1;
    //  if (k30.readCO2(co2ppm) != 0)
    // Serial.println("CO2 read failed");

    int TVOC = -1;
    if (sgp_initialized && sgp.IAQmeasure())
        TVOC = sgp.TVOC;

    sensorData.temp = t;
    sensorData.humid = h;
    sensorData.ch4 = readMQ4();
    sensorData.nox = readMQ135();
    sensorData.co2 = (float)co2ppm;
    sensorData.co = readMQ7();
    sensorData.tvoc = (float)TVOC;
    sensorData.pm_1_0 = pms7003.getPM_1_0();
    sensorData.pm_2_5 = pms7003.getPM_2_5();
    sensorData.pm_10_0 = pms7003.getPM_10_0();
}

void sendData()
{
    readSensors();

    if (isnan(gps_data.latitude) || isnan(gps_data.longitude))
    {
        Serial.println("GPS invalid or not received. Defaulting to 5.0, 5.0");
        sensorData.lat = 5.0;
        sensorData.lon = 5.0;
    }
    else
    {
        sensorData.lat = gps_data.latitude;
        sensorData.lon = gps_data.longitude;
    }

    Serial.println(sensorData.lat);
    Serial.println(sensorData.lon);

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
    // if (result == ESP_OK)
    // Serial.println("ESP-NOW send success");
    // Serial.printf("ESP-NOW send error: %d\n", result);
}

void checkRosConnection()
{
    static bool last_state = false;
    ros_connected = nh.connected();
    
    if (ros_connected != last_state)
    {
        if (ros_connected)
        {
            Serial.println("✓ ROS Connected");
            oledPrint("ROS Connected");
        }
        else
        {
            Serial.println("✗ ROS Disconnected");
            oledPrint("ROS Disconnected");
        }
        last_state = ros_connected;
    }
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    // Serial.println("ESP-NOW in Long-Range mode");

    if (esp_now_init() != ESP_OK)
    {
        // Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 6;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        // Serial.println("Failed to add peer");
        return;
    }

    Wire.begin(SDA_pin, SCL_pin);
    initOLED();
    oledPrint("Booting...");

    delay(100);
    // Serial.println("Starting PMS7003 on Serial1 (GPIO4)...");
    Serial1.begin(9600, SERIAL_8N1, 4, -1);

    // REMOVED: Serial2.begin() - Using USB-C port (Serial) for ROS communication

    if (!pms7003.init(&Serial1))
        oledPrint("PMS FAIL");

    if (sgp.begin())
    {
        sgp_initialized = true;
        // Serial.println("SGP30 initialized");
        oledPrint("SGP30 OK");
    }
    else
    {
        // Serial.println("SGP30 init FAILED");
        oledPrint("SGP30 FAILED");
        delay(500);
    }

    dht.begin();
    oledPrint("DHT11 OK");
    delay(100);

    Ro_MQ4 = calibrateSensorMQ4();
    Ro_MQ7 = calibrateSensorMQ7();
    Ro_MQ135 = calibrateSensorMQ135();

    oledPrint("CH4 CO NOx OK");
    delay(500);
    oledPrint("System Ready");

    Serial.println("Initializing ROS node...");
    nh.initNode();
    Serial.println("Subscribing to GPS topics...");
    nh.subscribe(gps_sub);
    nh.subscribe(health_sub);
    
    // Wait for initial ROS connection
    Serial.println("Waiting for ROS connection...");
    oledPrint("Waiting for ROS...");
    unsigned long start_time = millis();
    while (!nh.connected() && (millis() - start_time < 10000)) // 10 second timeout
    {
        nh.spinOnce();
        delay(100);
    }
    
    if (nh.connected())
    {
        Serial.println("✓ ROS Initial connection established");
        oledPrint("ROS Ready");
    }
    else
    {
        Serial.println("✗ ROS connection timeout");
        oledPrint("ROS Timeout");
    }
}

void loop()
{
    // readPiData();
    nh.spinOnce();
    
    // Check ROS connection status every 5 seconds
    if (millis() - last_connection_check > 5000)
    {
        checkRosConnection();
        last_connection_check = millis();
        
        // Debug output
        Serial.printf("ROS Connected: %s, GPS Valid: %s\n", 
                     ros_connected ? "YES" : "NO",
                     (!isnan(gps_data.latitude) && !isnan(gps_data.longitude)) ? "YES" : "NO");
    }
    
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000)
    {
        sendData();
        lastSend = millis();
    }
}
        lastSend = millis();
    }
}
