#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int channel = 6;

// Stored sensor values (support multiple message formats)
float latestResistance = 0.0;
float latestTemperature = 0.0;
float latestHumidity = 0.0;
float latestCO2 = NAN;

// Transistor peer (MAC of the ESP that runs transistor_control sketch)
const uint8_t transistorPeerAddr[6] = {0x10, 0x06, 0x1C, 0xF2, 0x01, 0x50};

unsigned long lastResistanceTime = 0;
unsigned long lastTempHumTime = 0;
unsigned long lastCO2Time = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // One-byte ACK from transistor ESP or simple command responses
  if (len == 1) {
    uint8_t b = incomingData[0];
    Serial.printf("[ESP-NOW] 1-byte msg from %s: 0x%02X\n", macStr, b);
    return;
  }

  // Full packet (4 floats: rs, temperature, humidity, co2)
  if (len == (int)(4 * sizeof(float))) {
    float vals[4];
    memcpy(vals, incomingData, sizeof(vals));
    latestResistance = vals[0];
    latestTemperature = vals[1];
    latestHumidity = vals[2];
    latestCO2 = vals[3];
    lastResistanceTime = millis();
    lastTempHumTime = millis();
    lastCO2Time = millis();
    Serial.printf("[ESP-NOW] Full data from %s: Rs=%.2f, Temp=%.2f C, Hum=%.2f %%, CO2=%.0f ppm\n", macStr, latestResistance, latestTemperature, latestHumidity, latestCO2);
    return;
  }

  // Full sensor message (3 floats: rs, temperature, humidity)
  if (len == (int)(3 * sizeof(float))) {
    float vals[3];
    memcpy(vals, incomingData, sizeof(vals));
    latestResistance = vals[0];
    latestTemperature = vals[1];
    latestHumidity = vals[2];
    lastResistanceTime = millis();
    lastTempHumTime = millis();
    Serial.printf("[ESP-NOW] Full data (no CO2) from %s: Rs=%.2f, Temp=%.2f C, Hum=%.2f %%\n", macStr, latestResistance, latestTemperature, latestHumidity);
    return;
  }

  // Temp+Humidity message (2 floats: temperature, humidity)
  if (len == (int)(2 * sizeof(float))) {
    float vals[2];
    memcpy(vals, incomingData, sizeof(vals));
    latestTemperature = vals[0];
    latestHumidity = vals[1];
    lastTempHumTime = millis();
    Serial.printf("[ESP-NOW] Temp/Hum from %s: %.2f C, %.2f %%\n", macStr, latestTemperature, latestHumidity);
    return;
  }

  // CO2-only message (int32 ppm) - backward compatible
  if (len == (int)sizeof(int32_t)) {
    int32_t co2 = 0;
    memcpy(&co2, incomingData, sizeof(co2));
    latestCO2 = (float)co2;
    lastCO2Time = millis();
    Serial.printf("[ESP-NOW] CO2 (int) from %s: %d ppm\n", macStr, (int)co2);
    return;
  }

  // Backwards-compat: single float (resistance)
  if (len == (int)sizeof(float)) {
    float resistance;
    memcpy(&resistance, incomingData, sizeof(float));
    latestResistance = resistance;
    lastResistanceTime = millis();
    Serial.printf("[ESP-NOW] Resistance (legacy) from %s: %.2f\n", macStr, resistance);
    return;
  }

  Serial.printf("[ESP-NOW] Unknown data size from %s: %d bytes\n", macStr, len);
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
  esp_now_register_send_cb(NULL);
  // add transistor peer so we can send single-byte commands
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, transistorPeerAddr, 6);
  peer.channel = channel;
  peer.encrypt = false;
  esp_err_t pr = esp_now_add_peer(&peer);
  if (pr == ESP_OK) Serial.println("Transistor peer added");
  else Serial.printf("Transistor peer add returned %d\n", pr);
  Serial.println("ESP-NOW resistance receiver started");
}

void loop()
{
  static unsigned long lastLog = 0;
  // Serial forwarding for transistor control
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("TRANSISTOR:")) {
      String v = line.substring(String("TRANSISTOR:").length());
      v.trim();
      uint8_t cmd = 0xFF;
      if (v == "1" || v.equalsIgnoreCase("ON")) cmd = 0x01;
      else if (v == "0" || v.equalsIgnoreCase("OFF")) cmd = 0x00;
      if (cmd != 0xFF) {
        esp_err_t r = esp_now_send(transistorPeerAddr, &cmd, 1);
        Serial.printf("Forwarded TRANSISTOR cmd=0x%02X to transistor ESP -> res=%d\n", cmd, r);
      } else {
        Serial.println("TRANSISTOR ERR");
      }
    }
  }
  if (millis() - lastLog > 5000)
  {
    Serial.println("\n=== Status Report ===");
  // Temp & Humidity
    if (lastTempHumTime > 0) {
      unsigned long age = millis() - lastTempHumTime;
      Serial.printf("Latest Temperature: %.2f C\n", latestTemperature);
      Serial.printf("Latest Humidity: %.2f %% (%.1fs ago)\n", latestHumidity, age / 1000.0);
      if (age > 10000) Serial.println("⚠️  WARNING: No recent Temp/Humidity data!");
    } else {
      Serial.println("No Temp/Humidity data received yet");
    }

    // CO2
    if (lastCO2Time > 0) {
      unsigned long age = millis() - lastCO2Time;
      if (isnan(latestCO2)) {
        Serial.println("Latest CO2: NAN");
      } else {
        Serial.printf("Latest CO2: %.0f ppm (%.1fs ago)\n", latestCO2, age / 1000.0);
      }
      if (age > 10000) Serial.println("⚠️  WARNING: No recent CO2 data!");
    } else {
      Serial.println("No CO2 data received yet");
    }

    // Resistance (legacy/full)
    if (lastResistanceTime > 0) {
      unsigned long age = millis() - lastResistanceTime;
      Serial.printf("Latest Resistance: %.2f ohms (%.1fs ago)\n", latestResistance, age / 1000.0);
      if (age > 10000) Serial.println("⚠️  WARNING: No recent resistance data!");
    } else {
      Serial.println("No resistance data received yet");
    }

    Serial.println("Waiting for ESP-NOW data...");
    lastLog = millis();
  }
}