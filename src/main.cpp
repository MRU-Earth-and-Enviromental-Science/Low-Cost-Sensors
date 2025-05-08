#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// temp sensor
#define DHTPIN 4
#define DHTTYPE DHT11
#define MQ4_PIN 34

#define RL_VALUE 10.0           // Load resistance in kilo ohms
#define RO_CLEAN_AIR_FACTOR 4.4 // From MQ-4 datasheet

DHT dht(DHTPIN, DHTTYPE);
float Ro = 10.0; // Calibrated in setup()



float readMQ4();
float calculateResistance(int adcValue);
float calibrateSensor();

void setup()
{
  Serial.begin(9600);
  dht.begin();
  Ro = calibrateSensor();
  Serial.print("Calibrated Ro = ");
  Serial.println(Ro);
}

void loop()
{
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F(" %\tTemperature: "));
  Serial.print(t);
  Serial.println(F(" *C"));

  float ppm = readMQ4();
  Serial.print("CH₄ PPM: ");
  Serial.println(ppm);
}


float readMQ4()
{
  int adcValue = analogRead(MQ4_PIN);
  float Rs = calculateResistance(adcValue);
  float ratio = Rs / Ro;

  // log-log line fit from datasheet: y = mx + b for log(ppm) = m*log(ratio) + b
  float m = -0.318; // slope
  float b = 1.133;  // intercept

  float ppm_log = (log10(ratio) - b) / m;
  return pow(10, ppm_log);
}

float calculateResistance(int adcValue)
{
  float voltage = adcValue * (3.3 / 4095.0); // ESP32 ADC is 12-bit
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