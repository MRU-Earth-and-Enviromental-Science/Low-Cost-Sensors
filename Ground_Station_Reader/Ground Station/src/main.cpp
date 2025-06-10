#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Wire.h>
#include "../include/dashboard.h"

const char *ssid = "Air-Quality-Station";
const char *password = "Neon_2017";
const int channel = 1;

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
} SensorData;

SensorData latestData;
bool logging_enabled = false;

WebServer server(80);

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (!logging_enabled || len != sizeof(SensorData))
    return;
  memcpy(&latestData, incomingData, sizeof(SensorData));
  Serial.println("[ESP-NOW] Data received.");
}

void handleRoot()
{
  server.send(200, "text/html", htmlPage);
}

void handleData()
{
  String json = "{";
  json += "\"temperature\":" + String(latestData.temp, 2) + ",";
  json += "\"humidity\":" + String(latestData.humid, 2) + ",";
  json += "\"ch4_ppm\":" + String(latestData.ch4, 2) + ",";
  json += "\"co2_ppm\":" + String(latestData.co2, 2) + ",";
  json += "\"tvoc_ppb\":" + String(latestData.tvoc, 2) + ",";
  json += "\"co_ppm\":" + String(latestData.co, 2) + ",";
  json += "\"nox_ppm\":" + String(latestData.nox, 2) + ",";
  json += "\"pm_1_0\":" + String(latestData.pm_1_0) + ",";
  json += "\"pm_2_5\":" + String(latestData.pm_2_5) + ",";
  json += "\"pm_10_0\":" + String(latestData.pm_10_0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleStart()
{
  logging_enabled = true;
  Serial.println("[HTTP] Logging started.");
  server.send(200, "text/plain", "started");
}

void handleStop()
{
  logging_enabled = false;
  Serial.println("[HTTP] Logging stopped.");
  server.send(200, "text/plain", "stopped");
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, channel);
  IPAddress ip = WiFi.softAPIP();
  Serial.println("[WiFi] SoftAP IP: " + ip.toString());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.begin();
  Serial.println("[HTTP] Web server started");
}

void loop()
{
  server.handleClient();
}
