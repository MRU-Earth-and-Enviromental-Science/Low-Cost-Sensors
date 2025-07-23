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


float vcc = 3.3;
float loadResistor = 10000;

uint8_t peerAddress[] = {0x10, 0x06, 0x1C, 0xF2, 0x01, 0x50};

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
    Serial.print("ESP-NOW send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); // Enable long range
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); // Set channel (must match receiver)

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1);
    }
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 6;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        while (1);
    }
}

void loop() {
    float rs = calculateResistance();
    Serial.print("Rs: ");
    Serial.println(rs, 6);
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&rs, sizeof(rs));
    if (result != ESP_OK) {
        Serial.print("ESP-NOW send error: ");
        Serial.println(result);
    }
    delay(1000);
}
