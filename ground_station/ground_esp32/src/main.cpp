#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int channel = 6;

float latestResistance = 0.0;
unsigned long lastResistanceTime = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(float))
  {
    float resistance;
    memcpy(&resistance, incomingData, sizeof(float));
    latestResistance = resistance;
    lastResistanceTime = millis();
    
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    Serial.printf("[ESP-NOW] Resistance from %s: %.2f ohms\n", macStr, resistance);
  }
  else
  {
    Serial.printf("[ESP-NOW] Invalid data size: %d bytes (expected %d)\n", len, sizeof(float));
  }
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&cfg) != ESP_OK)
  {
    Serial.println("[ERROR] Failed to init WiFi");
  }

  if (esp_wifi_start() != ESP_OK)
  {
    Serial.println("[ERROR] Failed to start WiFi");
  }

  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK)
  {
    Serial.println("[ERROR] Failed to enable long-range mode");
  }
  else
  {
    Serial.println("[WiFi] Long-range mode enabled");
  }

  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[ERROR] ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW resistance receiver started");
}

void loop()
{
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 5000)
  {
    Serial.println("\n=== Status Report ===");
    
    if (lastResistanceTime > 0)
    {
      unsigned long timeSinceResistance = millis() - lastResistanceTime;
      Serial.printf("Latest Resistance: %.2f ohms (%.1fs ago)\n", 
                    latestResistance, timeSinceResistance / 1000.0);
      
      if (timeSinceResistance > 10000) // No data for 10 seconds
      {
        Serial.println("⚠️  WARNING: No recent resistance data!");
      }
    }
    else
    {
      Serial.println("No resistance data received yet");
    }
    
    Serial.println("Waiting for ESP-NOW data...");
    lastLog = millis();
  }
}