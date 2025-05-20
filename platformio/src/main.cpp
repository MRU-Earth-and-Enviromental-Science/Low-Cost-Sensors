// includes
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_SGP30.h>
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/K30.h"
#include "../include/PM25.h"
#include "../include/LCD.h"
#include "../include/SGP.h"
#include "../include/dashboard.h"

// create objects
K30_I2C k30(K30_ADDRESS);
Adafruit_SGP30 sgp;

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// wifi
const char ssid[] = "Air Quality Monitor";
const char password[] = "Neon2017";

// global
WebServer server(80);
float Ro_MQ4 = 0.33;
float Ro_MQ9 = 0.33;

// export data
void sendData()
{
  Serial.println("[HTTP] /data endpoint hit");
  calibrateSensorPM25();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float ch4 = readMQ4();
  float co = readMQ7();
  int co2ppm = 0;
  int co2status = k30.readCO2(co2ppm);

  if (co2status == 1)
  {
    Serial.println("CO2 read failed, using -1");
    co2ppm = -1;
  }
  int TVOC = -1;
  if (sgp.IAQmeasure())
  {
    TVOC = sgp.TVOC;
  }
  else
  {
    Serial.println("SGP30 Measurement failed, using -1");
  }

  // json serialization
  String json = "{";
  json += "\"temperature\":" + String(t, 2) + ",";
  json += "\"humidity\":" + String(h, 2) + ",";
  json += "\"ch4_ppm\":" + String(ch4, 2) + ",";
  json += "\"co2_ppm\":" + String(co2ppm) + ",";
  json += "\"dust_density\":" + String(dustDensity) + ",";
  json += "\"tvoc_ppb\":" + String(TVOC) + ",";
  json += "\"co_ppm\":" + String(co, 2);
  json += "}";

  Serial.println("Sending JSON:");
  Serial.println(json);

  server.send(200, "application/json", json);

  delay(1000);
}

// functions
void initWifi()
{
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  printToSerialAndLCD("Access Point started. IP: " + IP.toString());

  server.on("/", HTTP_GET, []()
            { server.send(200, "text/html", htmlPage); });

  server.on("/data", HTTP_GET, sendData);

  server.begin();
  printToSerialAndLCD("Web server start");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor System Ready");
}

// setup
void setup()
{
  Serial.begin(115200);

  // Initialize LCD
  initLCD();

  analogReadResolution(10);
  Serial.println("***** ESP32 Dust Sensor Booting *****");

  sgp.begin();
  Serial.println("SGP30 initialized");

  Wire.begin();
  dht.begin();

  Ro_MQ4 = calibrateSensorMQ4();
  Ro_MQ7 = calibrateSensorMQ7();

  printToSerialAndLCD("Calibrated Ro_MQ4 = " + String(Ro_MQ4));
  delay(2000);
  printToSerialAndLCD("Calibrated Ro_MQ7 = " + String(Ro_MQ7));
  delay(2000);
  initWifi();
}

// main loop
void loop()
{
  server.handleClient();
}
