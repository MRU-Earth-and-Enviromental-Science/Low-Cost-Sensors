// #include <esp_now.h>
// #include <WiFi.h>
// #include <cstring>

// // MAC of the receiver (K-30 side ESP32)
// uint8_t receiverMac[] = {0xCC, 0x7B, 0x5C, 0x97, 0x46, 0x7C};

// // Default transistor peer MAC (placeholder - set via Serial with SETMAC:aa:bb:cc:dd:ee:ff)
// uint8_t transistorMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// bool transistorPeerAdded = false;

// // Callback to receive confirmation from receiver or ACK from transistor
// void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//   String msg;
//   for (int i = 0; i < len; i++) msg += (char)incomingData[i];
//   Serial.print("Received: "); Serial.println(msg);
// }

// // Helper to add an ESP-NOW peer
// esp_err_t addPeer(const uint8_t *peerAddr) {
//   esp_now_peer_info_t peerInfo = {};
//   memcpy(peerInfo.peer_addr, peerAddr, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;
//   return esp_now_add_peer(&peerInfo);
// }

// // Send single-byte command to transistor ESP
// void sendTransistorCommand(uint8_t v) {
//   if (!transistorPeerAdded) {
//     Serial.println("Transistor peer not added. Use SETMAC:aa:bb:cc:dd:ee:ff to set and add.");
//     return;
//   }
//   esp_err_t res = esp_now_send(transistorMac, &v, 1);
//   if (res == ESP_OK) {
//     Serial.printf("Sent transistor command %d\n", v);
//   } else {
//     Serial.printf("Failed to send transistor command: %d\n", res);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Register receive callback
//   esp_now_register_recv_cb(onDataRecv);

//   if (addPeer(receiverMac) != ESP_OK) {
//     Serial.println("Failed to add K30 receiver peer");
//   } else {
//     Serial.println("Added K30 receiver peer");
//   }

//   // If transistorMac was pre-configured (not all 0xFF), attempt to add
//   uint8_t allFF[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//   if (memcmp(transistorMac, allFF, 6) != 0) {
//     if (addPeer(transistorMac) == ESP_OK) transistorPeerAdded = true;
//   }

//   Serial.println("Sender ready. Commands:");
//   Serial.println("  ENTER -> send CALIB2000");
//   Serial.println("  TRANSISTOR:0 or TRANSISTOR:1 -> forward to transistor ESP");
//   Serial.println("  SETMAC:aa:bb:cc:dd:ee:ff -> set transistor MAC and add peer");
// }

// void loop() {
//   static String line;
//   while (Serial.available()) {
//     char c = Serial.read();
//     if (c == '\n' || c == '\r') {
//       if (line.length() == 0) { // ENTER -> trigger calibration
//         const char msg[] = "CALIB2000";
//         esp_err_t result = esp_now_send(receiverMac, (uint8_t *)msg, sizeof(msg));
//         if (result == ESP_OK) {
//           Serial.println("Sent calibration trigger.");
//         } else {
//           Serial.printf("Error sending message: %d\n", result);
//         }
//       } else {
//         if (line.startsWith("TRANSISTOR:")) {
//           String val = line.substring(strlen("TRANSISTOR:"));
//           val.trim();
//           if (val == "1") sendTransistorCommand(1);
//           else if (val == "0") sendTransistorCommand(0);
//           else Serial.println("Invalid TRANSISTOR value. Use 0 or 1.");
//         } else if (line.startsWith("SETMAC:")) {
//           String macs = line.substring(7);
//           // normalize
//           macs.replace(':', ' ');
//           int parts[6];
//           if (sscanf(macs.c_str(), "%x %x %x %x %x %x", &parts[0], &parts[1], &parts[2], &parts[3], &parts[4], &parts[5]) == 6) {
//             for (int i=0;i<6;i++) transistorMac[i] = (uint8_t)parts[i];
//             if (addPeer(transistorMac) == ESP_OK) {
//               transistorPeerAdded = true;
//               Serial.println("Transistor peer added.");
//             } else Serial.println("Failed to add transistor peer.");
//           } else Serial.println("Invalid MAC format. Use aa:bb:cc:dd:ee:ff");
//         } else {
//           Serial.print("Unknown command: "); Serial.println(line);
//         }
//       }
//       line = "";
//     } else {
//       line += c;
//     }
//   }
//   delay(10);
// }