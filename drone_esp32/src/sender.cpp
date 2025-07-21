// #include <Arduino.h> //done
// #include <esp_now.h>
// #include "esp_wifi.h"
// #include <WiFi.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>

// // LCD at I2C address 0x27, 16 chars, 2 lines
// LiquidCrystal_I2C lcd(0x3F, 16, 2);

// // Receiver MAC Address
// uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xF2, 0x02, 0xCC};

// typedef struct struct_message
// {
//   char a[32];
//   int b;
//   float c;
//   bool d;
// } struct_message;

// struct_message myData;
// esp_now_peer_info_t peerInfo;

// // callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
//   String statusStr = status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail";
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(statusStr);
//   lcd.setCursor(0, 1);
//   lcd.print("Send: " + statusStr + "   ");
// }

// void setup()
// {
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);

//   // Set long-range protocol on STA interface
//   esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

//   // Initialize I2C explicitly
//   Wire.begin(); // Initialize I2C explicitly
//   // Initialize LCD
//   lcd.init();
//   lcd.backlight();
//   lcd.setCursor(0, 0);
//   lcd.print("ESP-NOW TX Ready");

//   if (esp_now_init() != ESP_OK)
//   {
//     Serial.println("ESP-NOW init failed");
//     lcd.setCursor(0, 1);
//     lcd.print("Init failed     ");
//     return;
//   }

//   esp_now_register_send_cb(OnDataSent);

//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   if (esp_now_add_peer(&peerInfo) != ESP_OK)
//   {
//     Serial.println("Failed to add peer");
//     lcd.setCursor(0, 1);
//     lcd.print("Add peer failed ");
//     return;
//   }
// }

// void loop()
// {
//   strcpy(myData.a, "THIS IS A CHAR");
//   myData.b = random(1, 20);
//   myData.c = 1.2;
//   myData.d = false;

//   lcd.setCursor(0, 1);
//   lcd.print("Sending...      ");

//   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

//   if (result == ESP_OK)
//   {
//     Serial.println("Sent with success");
//   }
//   else
//   {
//     Serial.println("Error sending the data");
//     lcd.setCursor(0, 1);
//     lcd.print("Send error      ");
//   }

//   delay(2000);
// }