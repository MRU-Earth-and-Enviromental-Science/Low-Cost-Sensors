#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// wifi
WebServer server(80);
const char ssid[] = "gasSensor";
const char password[] = "Neon2017";

// Temp Sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define MQ4_PIN 34
#define RL_VALUE 10.0           // Load resistance in kilo ohms
#define RO_CLEAN_AIR_FACTOR 4.4 // From MQ-4 datasheet
float Ro = 10.0;                // Calibrated in setup()

// functions
float readMQ4();
float calculateResistance(int adcValue);
float calibrateSensor();
void sendData();
void initWifi();

void setup()
{
  Serial.begin(9600);
  dht.begin();

  Ro = calibrateSensor();
  Serial.print("Calibrated Ro = ");
  Serial.println(Ro);

  initWifi();
}

void loop()
{
  server.handleClient(); // MUST call this to process HTTP requests
}

// ----- WiFi Access Point & Server Init -----
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

// webserver Handler
void sendData()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float ppm = readMQ4();

  if (isnan(h) || isnan(t))
  {
    server.send(500, "application/json", "{\"error\": \"DHT read failed\"}");
    return;
  }

  String json = "{";
  json += "\"temperature\":" + String(t, 3) + ",";
  json += "\"humidity\":" + String(h, 2) + ",";
  json += "\"ch4_ppm\":" + String(ppm, 2);
  json += "}";

  server.send(200, "application/json", json);
}

// ----- MQ4 Sensor Functions -----
float readMQ4()
{
  int adcValue = analogRead(MQ4_PIN);
  float Rs = calculateResistance(adcValue);
  float ratio = Rs / Ro;

  // log-log line fit from MQ-4 datasheet
  float m = -0.318;
  float b = 1.133;

  float ppm_log = (log10(ratio) - b) / m;
  return pow(10, ppm_log);
}

float calculateResistance(int adcValue)
{
  float voltage = adcValue * (3.3 / 4095.0); // 12-bit ADC
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