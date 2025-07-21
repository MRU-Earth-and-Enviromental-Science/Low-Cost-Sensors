#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

const int channel = 6;

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

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len != sizeof(SensorData))
    return;

  memcpy(&latestData, incomingData, sizeof(SensorData));

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("[ESP-NOW] Data received from %s:\n", macStr);
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

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true); // Disconnect from any network

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&cfg) != ESP_OK)
  {
    Serial.println("[ERROR] Failed to init WiFi");
  }

  if (esp_wifi_start() != ESP_OK)
  {
    Serial.println("[ERROR] Failed to start WiFi");
  }

  // Enable Long Range mode (802.11 LR protocol)
  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK)
  {
    Serial.println("[ERROR] Failed to enable long-range mode");
  }
  else
  {
    Serial.println("[WiFi] Long-range mode enabled");
  }

  // Set ESP-NOW to channel 6
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[ERROR] ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW with long-range mode started");
}

void loop()
{
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 5000)
  {
    Serial.println("Waiting for ESP-NOW data...");
    lastLog = millis();
  }
}