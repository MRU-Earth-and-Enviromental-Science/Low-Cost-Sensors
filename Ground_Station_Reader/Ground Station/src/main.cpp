#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
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
  float lat;
  float lon;
} SensorData;

SensorData latestData;
bool logging_enabled = false;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len != sizeof(SensorData))
    return;
  memcpy(&latestData, incomingData, sizeof(SensorData));
  Serial.println("[ESP-NOW] Data received:");
  Serial.printf("Temp: %.2f, Humid: %.2f, CH4: %.2f, CO2: %.2f, TVOC: %.2f, CO: %.2f, NOx: %.2f\n",
                latestData.temp, latestData.humid, latestData.ch4, latestData.co2,
                latestData.tvoc, latestData.co, latestData.nox);
  Serial.printf("PM1.0: %d, PM2.5: %d, PM10.0: %d\n",
                latestData.pm_1_0, latestData.pm_2_5, latestData.pm_10_0);
  Serial.printf("Lat: %.6f, Lon: %.6f\n", latestData.lat, latestData.lon);
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
  Serial.println("ESP-NOW started");
}

void loop()
{
}