// // Minimal ESP32 sketch: accepts Serial and ESP-NOW transistor toggle commands.
// // Pin and behavior configurable here.
// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>

// const int TRANSISTOR_PIN = 2; // change to the GPIO driving your transistor/MOSFET
// const uint32_t SERIAL_BAUD = 115200;

// // apply transistor state and ACK on Serial
// void setTransistor(bool on) {
//   digitalWrite(TRANSISTOR_PIN, on ? HIGH : LOW);
//   Serial.printf("TRANSISTOR:%d OK\n", on ? 1 : 0);
// }

// // ESP-NOW receive callback
// void onEspNowRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//   if (len <= 0) return;
//   uint8_t cmd = incomingData[0];
//   if (cmd == 0x00 || cmd == 0x01) {
//     bool on = (cmd == 0x01);
//     setTransistor(on);

//     // send ACK back to sender (single byte identical to cmd)
//     uint8_t ack = cmd;
//     esp_err_t res = esp_now_send(mac, &ack, 1);

//     Serial.printf("[ESP-NOW] cmd=%d from %02X:%02X:%02X:%02X:%02X:%02X ack_sent=%d\n",
//                   cmd, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], res == ESP_OK ? 1 : 0);
//   } else {
//     Serial.println("TRANSISTOR ERR"); // invalid payload
//   }
// }

// // ESP-NOW send callback (optional small log)
// void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   (void)mac_addr;
//   (void)status;
// }

// // Read one line from Serial, returns empty String if none
// String readSerialLine() {
//   static String line = "";
//   while (Serial.available()) {
//     char c = (char)Serial.read();
//     if (c == '\r') continue;
//     if (c == '\n') {
//       String out = line;
//       line = "";
//       return out;
//     } else {
//       line += c;
//     }
//   }
//   return String();
// }

// void setup() {
//   pinMode(TRANSISTOR_PIN, OUTPUT);
//   digitalWrite(TRANSISTOR_PIN, LOW);

//   Serial.begin(SERIAL_BAUD);
//   while (!Serial && millis() < 2000) delay(1);
//   Serial.println("ESP32 ready - waiting for TRANSISTOR commands");

//   // Initialize WiFi/ESP-NOW
//   WiFi.mode(WIFI_STA);
//   WiFi.disconnect();

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW (but Serial still works)");
//   } else {
//     esp_now_register_recv_cb(onEspNowRecv);
//     esp_now_register_send_cb(onEspNowSent);
//     Serial.println("ESP-NOW initialized");
//   }
// }

// void loop() {
//   // Serial handler
//   String l = readSerialLine();
//   if (l.length()) {
//     l.trim();
//     if (l.startsWith("TRANSISTOR:")) {
//       String v = l.substring(String("TRANSISTOR:").length());
//       v.trim();
//       if (v == "1" || v.equalsIgnoreCase("ON")) {
//         setTransistor(true);
//       } else if (v == "0" || v.equalsIgnoreCase("OFF")) {
//         setTransistor(false);
//       } else {
//         Serial.println("TRANSISTOR ERR");
//       }
//     } else {
//       Serial.printf("UNKNOWN CMD: %s\n", l.c_str());
//     }
//   }

//   delay(10);
// }
