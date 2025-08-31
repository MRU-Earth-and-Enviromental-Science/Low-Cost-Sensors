// #include <Wire.h>
// #include <esp_now.h>
// #include <WiFi.h>
// #include <Arduino.h>

// #define K30_ADDR 0x68   // I2C address for K-30

// // MAC of the sender (laptop side ESP32)
// uint8_t senderMac[6] = {0x10, 0x06, 0x1C, 0xF2, 0x01, 0x50};

// void writeRegister(uint16_t reg, uint16_t value) {
//   Wire.beginTransmission(K30_ADDR);
//   Wire.write(reg >> 8);
//   Wire.write(reg & 0xFF);
//   Wire.write(value >> 8);
//   Wire.write(value & 0xFF);
//   byte err = Wire.endTransmission();

//   if (err == 0) {
//     Serial.print("Wrote ");
//     Serial.print(value);
//     Serial.print(" to register 0x");
//     Serial.println(reg, HEX);
//   } else {
//     Serial.print("I2C error: ");
//     Serial.println(err);
//   }
// }

// void disableABC() {
//   Serial.println("Disabling ABC...");
//   writeRegister(0x0010, 0x0000);
// }

// void forceCalibration2000() {
//   Serial.println("Calibrating at 2000 ppm...");
//   writeRegister(0x03EC, 2000);
//   Serial.println("Calibration command sent.");
// }

// // ESP-NOW receive callback
// void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//   String msg;
//   for (int i = 0; i < len; i++) msg += (char)incomingData[i];
//   Serial.print("Received: "); Serial.println(msg);

//   if (msg.startsWith("CALIB2000")) {
//     forceCalibration2000();

//     // Add sender as peer if not already
//     esp_now_peer_info_t peerInfo = {};
//     memcpy(peerInfo.peer_addr, senderMac, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;
//     esp_err_t r = esp_now_add_peer(&peerInfo);
//     if (r == ESP_OK) {
//       Serial.println("Added sender peer for confirmation.");
//     }

//     // Send confirmation back to sender
//     const char reply[] = "CONFIRMED";
//     esp_err_t result = esp_now_send(senderMac, (uint8_t *)reply, strlen(reply));
//     if (result == ESP_OK) {
//       Serial.println("Confirmation sent to sender.");
//     } else {
//       Serial.printf("Error sending confirmation: %d\n", result);
//     }
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(); // SDA=21, SCL=22
//   WiFi.mode(WIFI_STA);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   esp_now_register_recv_cb(onDataRecv);

//   disableABC(); // optional: disable ABC at startup

//   Serial.println("Receiver ready, waiting for calibration trigger...");
// }

// void loop() {
//   // nothing here, handled by callback
//   delay(10);
// }
