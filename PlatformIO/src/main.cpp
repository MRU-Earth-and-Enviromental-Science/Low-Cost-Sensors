#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Wire.h>
#include "../include/OLED.h"
#include "../include/Temp.h"
#include "../include/CH4.h"
#include "../include/CO.h"
#include "../include/K30.h"
#include "../include/SGP.h"
#include "../include/NOx.h"
#include "../include/PM25.h"

Plantower_PMS7003 pms7003;

// I2C Protocol
#define SDA_pin 21
#define SCL_pin 22

// Objects
Adafruit_SGP30 sgp;
bool sgp_initialized = false;

uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xF2, 0x02, 0xCC};

typedef struct __attribute__((packed))
{
  float temp;
  float humid;
  float ch4;        // done
  float co2;        // done
  float tvoc;       // done
  float co;         // done
  float nox;        // done
  uint16_t pm_1_0;  // done
  uint16_t pm_2_5;  // done
  uint16_t pm_10_0; // done
} SensorData;

SensorData sensorData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void readSensors()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h))
    h = -1.0;
  if (isnan(t))
    t = -1.0;
  int co2ppm = -1;
  if (k30.readCO2(co2ppm) != 0)
    Serial.println("CO2 read failed");

  int TVOC = -1;
  if (sgp_initialized && sgp.IAQmeasure())
    TVOC = sgp.TVOC;

  sensorData = {
    .temp : t,
    .humid : h,
    .ch4 : readMQ4(),
    .nox : readMQ135(),
    .co2 : (float)co2ppm,
    .co : readMQ7(),
    .tvoc : (float)TVOC,
    .pm_1_0 : pms7003.getPM_1_0(),
    .pm_2_5 : pms7003.getPM_2_5(),
    .pm_10_0 : pms7003.getPM_10_0()
  };
}

void sendData()
{
  readSensors();
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK)
    Serial.println("ESP-NOW send success");
  else
    Serial.printf("ESP-NOW send error: %d\n", result);
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  Serial.println("ESP-NOW in Long-Range mode");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  initOLED();
  Wire.begin(SDA_pin, SCL_pin);
  oledPrint("Booting...");

  delay(100);
  Serial.println("Starting PMS7003 on Serial1 (GPIO4)...");
  Serial1.begin(9600, SERIAL_8N1, 4, -1);

  if (!pms7003.init(&Serial1))
    oledPrint("PMS FAIL");

  if (sgp.begin())
  {
    sgp_initialized = true;
    Serial.println("SGP30 initialized");
    oledPrint("SGP30 OK");
  }
  else
  {
    Serial.println("SGP30 init FAILED");
    oledPrint("SGP30 FAILED");
    delay(500);
  }

  dht.begin();
  oledPrint("DHT11 OK");
  delay(100);

  Ro_MQ4 = calibrateSensorMQ4();
  Ro_MQ7 = calibrateSensorMQ7();
  Ro_MQ135 = calibrateSensorMQ135();

  oledPrint("CH4 CO NOx OK");
  delay(500);
  oledPrint("System Ready");
}

void loop()
{
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 10000)
  {
    sendData();
    lastSend = millis();
  }
}