#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
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

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// Objects
Adafruit_SGP30 sgp;
bool sgp_initialized = false;

ros::NodeHandle nh;

// ROS Publishers
std_msgs::Float32 temp_msg, humid_msg, ch4_msg, co2_msg, tvoc_msg, co_msg, nox_msg;
std_msgs::UInt16 pm1_msg, pm25_msg, pm10_msg;
std_msgs::String status_msg;

ros::Publisher temp_pub("/sensors/temperature", &temp_msg);
ros::Publisher humid_pub("/sensors/humidity", &humid_msg);
ros::Publisher ch4_pub("/sensors/ch4", &ch4_msg);
ros::Publisher co2_pub("/sensors/co2", &co2_msg);
ros::Publisher tvoc_pub("/sensors/tvoc", &tvoc_msg);
ros::Publisher co_pub("/sensors/co", &co_msg);
ros::Publisher nox_pub("/sensors/nox", &nox_msg);
ros::Publisher pm1_pub("/sensors/pm1_0", &pm1_msg);
ros::Publisher pm25_pub("/sensors/pm2_5", &pm25_msg);
ros::Publisher pm10_pub("/sensors/pm10_0", &pm10_msg);
ros::Publisher status_pub("/sensors/status", &status_msg);

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

    // Update message data
    temp_msg.data = t;
    humid_msg.data = h;
    ch4_msg.data = readMQ4();
    nox_msg.data = readMQ135();
    co2_msg.data = (float)co2ppm;
    co_msg.data = readMQ7();
    tvoc_msg.data = (float)TVOC;
    pm1_msg.data = pms7003.getPM_1_0();
    pm25_msg.data = pms7003.getPM_2_5();
    pm10_msg.data = pms7003.getPM_10_0();

    status_msg.data = "Sensors OK";

    // Publish all sensor data
    temp_pub.publish(&temp_msg);
    humid_pub.publish(&humid_msg);
    ch4_pub.publish(&ch4_msg);
    co2_pub.publish(&co2_msg);
    tvoc_pub.publish(&tvoc_msg);
    co_pub.publish(&co_msg);
    nox_pub.publish(&nox_msg);
    pm1_pub.publish(&pm1_msg);
    pm25_pub.publish(&pm25_msg);
    pm10_pub.publish(&pm10_msg);
    status_pub.publish(&status_msg);
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_pin, SCL_pin);
    initOLED();
    oledPrint("Booting...");

    delay(100);
    // Serial.println("Starting PMS7003 on Serial1 (GPIO4)...");
    Serial1.begin(9600, SERIAL_8N1, 4, -1);

    delay(100);

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

    // Initialize ROS node and publishers
    nh.initNode();
    nh.advertise(temp_pub);
    nh.advertise(humid_pub);
    nh.advertise(ch4_pub);
    nh.advertise(co2_pub);
    nh.advertise(tvoc_pub);
    nh.advertise(co_pub);
    nh.advertise(nox_pub);
    nh.advertise(pm1_pub);
    nh.advertise(pm25_pub);
    nh.advertise(pm10_pub);
    nh.advertise(status_pub);

    oledPrint("ROS Ready");
    delay(500);
    oledPrint("System Ready");
}

void loop()
{
    nh.spinOnce();
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000)
    {
        readSensors();
        lastSend = millis();
    }
}
