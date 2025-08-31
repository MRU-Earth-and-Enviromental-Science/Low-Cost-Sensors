#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <Wire.h>
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/NOx.h"
#include "../include/K30.h"


float vcc = 3.3;
float loadResistor = 10000;

uint8_t peerAddress[] = {0x10, 0x06, 0x1C, 0xF2, 0x01, 0x50};

typedef struct __attribute__((packed)) {
    float rs;
    float temperature;
    float humidity;
    float co2; // CO2 ppm from K30
} SensorData;

#define TRANSISTOR_PIN 5

float calculateResistance() {

    static float filteredRs = 0;
    const float alpha = 0.15; // Smoothing factor (0 < alpha <= 1)
    int adcValue = analogRead(34);
    float voltage = adcValue * (vcc / 4095.0);
    float rs = loadResistor * (vcc - voltage) / voltage;
    if (filteredRs == 0) filteredRs = rs; // Initialize on first run
    filteredRs = alpha * rs + (1 - alpha) * filteredRs;
    return filteredRs;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
    (void)status; // silent callback - no serial output
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    // Handle simple transistor commands. Accept either a single byte 0x00/0x01
    // or an ASCII string like "TRANSISTOR:0" / "TRANSISTOR:1".
    if (len <= 0) return;

    // If single byte, treat as direct command
    if (len == 1) {
        uint8_t cmd = incomingData[0];
        if (cmd == 0x01) {
            digitalWrite(TRANSISTOR_PIN, HIGH);
            Serial.println("TRANSISTOR -> HIGH (single-byte)");
        } else if (cmd == 0x00) {
            digitalWrite(TRANSISTOR_PIN, LOW);
            Serial.println("TRANSISTOR -> LOW (single-byte)");
        }
        // echo back as ACK
        esp_now_send(mac, incomingData, 1);
        return;
    }

    // Otherwise try to parse ASCII command with provided length
    String s = String((const char *)incomingData, len);
    s.trim();
    if (s.startsWith("TRANSISTOR:")) {
        char v = s.charAt(11);
        if (v == '1') {
            digitalWrite(TRANSISTOR_PIN, HIGH);
            Serial.println("TRANSISTOR -> HIGH (ascii)");
        } else if (v == '0') {
            digitalWrite(TRANSISTOR_PIN, LOW);
            Serial.println("TRANSISTOR -> LOW (ascii)");
        }
        // send ACK (echo full message back)
        esp_now_send(mac, incomingData, len);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); // Enable long range
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); // Set channel (must match receiver)

    if (esp_now_init() != ESP_OK) {
        Serial.println("esp_now_init failed");
        while (1); // silent failure
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 6;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("esp_now_add_peer failed");
    while (1); // silent failure
    }

    // Initialize I2C for K30
    Wire.begin();

    // transistor pin
    pinMode(TRANSISTOR_PIN, OUTPUT);
    digitalWrite(TRANSISTOR_PIN, LOW);
}

void loop() {
    float rs = calculateResistance();
    float temperature = NAN, humidity = NAN;
    // read temperature & humidity from Temp module
    readTemperatureHumidity(temperature, humidity);

    // read CO2 from K30
    int co2Level = 0;
    float co2 = NAN;
    int k30status = k30.readCO2(co2Level);
    if (k30status == 0) {
        co2 = (float)co2Level;
    }

    SensorData data;
    data.rs = rs;
    data.temperature = temperature;
    data.humidity = humidity;
    data.co2 = co2;

    // send packet (silent)
    esp_now_send(peerAddress, (uint8_t *)&data, sizeof(data));

    // print only sensor values to serial (no ESP-NOW debug)
    Serial.print("Rs: ");
    Serial.print(rs, 6);
    Serial.print("  Temp: ");
    if (isnan(temperature)) Serial.print("NAN"); else Serial.print(temperature, 2);
    Serial.print("  Hum: ");
    if (isnan(humidity)) Serial.print("NAN"); else Serial.print(humidity, 2);

    Serial.print("  CO2: ");
    if (isnan(co2)) Serial.println("NAN"); else Serial.println(co2, 0);

    delay(1000);
}