// includes
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Adafruit_SH110X.h>
#include "../include/OLED.h"
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/K30.h"
#include "../include/SGP.h"
#include "../include/dashboard.h"
#include "../include/NOx.h"

// create objects
K30_I2C k30(K30_ADDRESS);
extern Adafruit_SH1106G display;
Adafruit_SGP30 sgp;

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// wifi
const char ssid[] = "Air Quality Monitor";
const char password[] = "LebronJames";

// global
WebServer server(80);
float Ro_MQ4 = 0.33;
float Ro_MQ9 = 0.33;
float Ro_MQ135 = 0.33;

// export data
void sendData()
{
  Serial.println("[HTTP] /data endpoint hit");
  oledPrintln("[HTTP] /data endpoint hit");
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float ch4 = readMQ4();
  float co = readMQ7();
  float nox = readMQ135();
  int co2ppm = 0;
  int co2status = k30.readCO2(co2ppm);

  if (co2status == 1)
  {
    Serial.println("CO2 read failed, using -1");
    oledPrintln("CO2 read failed, using -1");
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
    oledPrintln("SGP30 Measurement failed");
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
  oledPrintln("Sending JSON");
  Serial.println(json);

  server.send(200, "application/json", json);

  yield(); // prevent watchdog timeout
}

// functions
void initWifi()
{
  delay(500);
  WiFi.softAP(ssid, password);
  delay(500);
  IPAddress IP = WiFi.softAPIP();
  if (IP)
  {
    Serial.print("Access Point started. IP: ");
    Serial.println(IP);
    oledPrint("Access Point IP: " + IP.toString());
  }
  else
  {
    Serial.println("Failed to get IP. Restarting WiFi...");
    WiFi.softAPdisconnect(true);
    delay(1000);
    WiFi.softAP(ssid, password);
    delay(1000);
  }

  server.on("/", HTTP_GET, []()
            {
              Serial.println("[HTTP] / page hit - serving dashboard");
              oledPrintln("[HTTP] / page hit - serving dashboard");
              server.send(200, "text/html", htmlPage); });

  server.on("/data", HTTP_GET, sendData);

  server.begin();
  Serial.println("Web server started");
  oledPrintln("Web server started");
  oledPrint("Web server start");

  oledPrint("Sensor System Ready");
  Serial.println("Sensor System Ready");
  oledPrintln("Sensor System Ready");
}

// setup
void setup()
{
  Serial.begin(115200);
  initOLED();

  analogReadResolution(10);
  Serial.println("***** ESP32 Dust Sensor Booting *****");
  oledPrintln("ESP32 Dust Sensor Booting");
  delay(2000);

  Serial.println("SGP30 initialized");
  delay(100);
  oledPrintln("SGP30 initialized");
  // Consolidate OLED status messages to reduce flicker and unnecessary refreshes
  oledPrint("SGP30 initialized\nI2C initialized\nDHT11 initialized");
  Wire.begin(SDA_pin, SCL_pin);
  delay(100);
  dht.begin();
  delay(200);
  float testHum = dht.readHumidity();
  float testTemp = dht.readTemperature();

  Serial.println("Calibrating MQ4...");
  Ro_MQ4 = calibrateSensorMQ4();
  delay(100);
  Serial.println("Calibrating MQ7...");
  Ro_MQ7 = calibrateSensorMQ7();
  delay(100);
  Serial.println("Calibrating MQ135...");
  Ro_MQ135 = calibrateSensorMQ135();

  delay(3000);
  initWifi();
  Serial.println("Setup complete. System ready.");
}

// main loop
void loop()
{
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000)
  {
    if (WiFi.softAPgetStationNum() == 0)
    {
      Serial.println("No clients connected. Still running...");
    }
    lastCheck = millis();
  }
  server.handleClient();
}
