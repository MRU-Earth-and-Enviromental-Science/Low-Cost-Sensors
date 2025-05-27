#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Wire.h>
#include "../include/OLED.h"
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/K30.h"
#include "../include/SGP.h"
#include "../include/dashboard.h"
#include "../include/NOx.h"

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// WiFi
const char ssid[] = "Air Quality Monitor";
const char password[] = "LebronJames";

// Objects
Adafruit_SGP30 sgp;
bool sgp_initialized = false;

WebServer server(80);

// -- Function: Send data as JSON ----
void sendData()
{
  Serial.println("[HTTP] /data endpoint hit");

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h))
    h = -1.0;
  if (isnan(t))
    t = -1.0;

  float ch4 = readMQ4();
  float co = readMQ7();
  float nox = readMQ135();

  int co2ppm = -1;
  if (k30.readCO2(co2ppm) != 0)
  {
    Serial.println("CO2 read failed");
    co2ppm = -1;
  }

  int TVOC = -1;
  if (sgp_initialized && sgp.IAQmeasure())
  {
    TVOC = sgp.TVOC;
  }
  else
  {
    Serial.println("SGP30 read failed or not initialized");
  }

  String json = "{";
  json += "\"temperature\":" + String(t, 2) + ",";
  json += "\"humidity\":" + String(h, 2) + ",";
  json += "\"ch4_ppm\":" + String(ch4, 2) + ",";
  json += "\"co2_ppm\":" + String(co2ppm) + ",";
  json += "\"tvoc_ppb\":" + String(TVOC) + ",";
  json += "\"co_ppm\":" + String(co, 2) + ",";
  json += "\"nox_ppm\":" + String(nox, 2);
  json += "}";

  Serial.println("Sending JSON:");
  Serial.println(json);
  server.send(200, "application/json", json);
}

// ---- Function: Initialize Wi-Fi ----
void initWifi()
{
  WiFi.softAP(ssid, password);
  delay(1000);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Access Point started. IP: " + IP.toString());
  oledPrint("AP IP: " + IP.toString());

  server.on("/", HTTP_GET, []()
            {
              Serial.println("[HTTP] / page hit - serving dashboard");
              server.send(200, "text/html", htmlPage); });

  server.on("/data", HTTP_GET, sendData);

  server.begin();
  oledPrint("Web server started");
  Serial.println("Web server started");
}

// ---- Setup ---- (initialize when started)
void setup()
{
  Serial.begin(115200);
  initOLED();

  // Added rectangle above triangle
  display.fillRect(40, 8, 48, 16, SH110X_WHITE);

  Wire.begin(SDA_pin, SCL_pin);

  oledPrint("Booting...");

  delay(100);

  if (sgp.begin())
  {
    sgp_initialized = true;
    Serial.println("SGP30 initialized");
    oledPrint("SGP30 OK");
  }
  else
  {
    Serial.println("SGP30 init FAILED");
    oledPrint("SGP30 FAILED");
    delay(500);
  }

  dht.begin();
  oledPrint("DHT11 OK");
  delay(500);

  Ro_MQ4 = calibrateSensorMQ4();
  Ro_MQ7 = calibrateSensorMQ7();
  Ro_MQ135 = calibrateSensorMQ135();

  oledPrint("CH4 CO NOx OK");

  delay(500);
  initWifi();
  oledPrint("System Ready");
}

// ---- Loop ----
void loop()
{
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000)
  {
    if (WiFi.softAPgetStationNum() == 0)
    {
      Serial.println("No clients connected."); // debug message
    }
    lastCheck = millis();
  }

  server.handleClient();
}