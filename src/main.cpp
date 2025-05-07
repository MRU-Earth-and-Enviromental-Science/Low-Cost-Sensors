#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
float temperature = 24.7;

void handleRoot()
{
  String response = "{\"temperature\":" + String(temperature) + "}";
  server.send(200, "application/json", response);
}

void setup()
{
  Serial.begin(115200);
  WiFi.softAP("ESP32_Temp", "12345678");
  Serial.println("Access Point Started");

  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started at 192.168.4.1");
}

void loop()
{
  temperature += 0.01;

  server.handleClient();
  delay(1000);
}