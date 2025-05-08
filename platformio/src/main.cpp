#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <Wire.h>

// ========== CONFIG ==========
#define DHTPIN 4
#define DHTTYPE DHT11
#define MQ4_PIN 34
#define RL_VALUE 10.0
#define RO_CLEAN_AIR_FACTOR 4.4
#define SDA_pin 27
#define SCL_pin 26
#define K30_ADDRESS 0x68
const char ssid[] = "gasSensor";
const char password[] = "Neon2017";

// ========== GLOBALS ==========
WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);
float Ro = 10.0;

// ======= K30 CO₂ Class =======
class K30_I2C
{
public:
  K30_I2C(int i2c_address) { _i2c_address = i2c_address; }
  int readCO2(int &CO2level)
  {
    byte recValue[4] = {0};
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x22);
    Wire.write(0x00);
    Wire.write(0x08);
    Wire.write(0x2A);
    Wire.endTransmission();
    delay(30);
    Wire.requestFrom(_i2c_address, 4);
    delay(20);
    byte i = 0;
    while (Wire.available() && i < 4)
      recValue[i++] = Wire.read();
    CO2level = (recValue[1] << 8) + recValue[2];
    byte checkSum = recValue[0] + recValue[1] + recValue[2];
    if (i == 0)
      return 2;
    else if (checkSum == recValue[3])
      return 0;
    else
      return 1;
  }

private:
  int _i2c_address;
};

K30_I2C k30(K30_ADDRESS);

// ========== FUNCTION HEADERS ==========
void initWifi();
void sendData();
float readMQ4();
float calculateResistance(int adcValue);
float calibrateSensor();

// ========== SETUP ==========
void setup()
{
  Serial.begin(9600);
  Wire.begin(SDA_pin, SCL_pin);
  dht.begin();
  Ro = calibrateSensor();
  Serial.print("Calibrated Ro = ");
  Serial.println(Ro);
  initWifi();
}

// ========== LOOP ==========
void loop()
{
  server.handleClient();
}

// ========== WiFi Access Point ==========
void initWifi()
{
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point started. IP: ");
  Serial.println(IP);
  server.on("/data", HTTP_GET, sendData);
  server.begin();
  Serial.println("Web server started");
}

// ========== API Endpoint ==========
void sendData()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float ch4 = readMQ4();
  int co2ppm = 0;
  int co2status = k30.readCO2(co2ppm);

  if (isnan(h) || isnan(t))
  {
    server.send(500, "application/json", "{\"error\": \"DHT read failed\"}");
    return;
  }
  if (co2status == 1)
  {
    server.send(500, "application/json", "{\"error\": \"CO2 read failed\"}");
    return;
  }

  String json = "{";
  json += "\"temperature\":" + String(t, 2) + ",";
  json += "\"humidity\":" + String(h, 2) + ",";
  json += "\"ch4_ppm\":" + String(ch4, 2) + ",";
  json += "\"co2_ppm\":" + String(co2ppm);
  json += "}";

  server.send(200, "application/json", json);
}

// ========== MQ4 GAS FUNCTIONS ==========
float readMQ4()
{
  int adcValue = analogRead(MQ4_PIN);
  float Rs = calculateResistance(adcValue);
  float ratio = Rs / Ro;
  float m = -0.318;
  float b = 1.133;
  float ppm_log = (log10(ratio) - b) / m;
  return pow(10, ppm_log);
}

float calculateResistance(int adcValue)
{
  float voltage = adcValue * (3.3 / 4095.0);
  return RL_VALUE * (3.3 - voltage) / voltage;
}

float calibrateSensor()
{
  float val = 0.0;
  for (int i = 0; i < 50; i++)
  {
    val += calculateResistance(analogRead(MQ4_PIN));
    delay(100);
  }
  val /= 50.0;
  return val / RO_CLEAN_AIR_FACTOR;
}