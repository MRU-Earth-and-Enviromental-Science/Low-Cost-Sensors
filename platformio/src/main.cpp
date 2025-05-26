#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "ESP32-AP";
const char *password = "12345678";

WebServer server(80);

void handleData()
{
  String json = "{\"number\":" + String(random(0, 100)) + "}";
  server.send(200, "application/json", json);
}

void setup()
{
  Serial.begin(115200);
  randomSeed(analogRead(0));

  WiFi.softAP(ssid, password);
  delay(1000);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("ESP32 AP IP: ");
  Serial.println(IP);

  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started");
}

void loop()
{
  server.handleClient();
  yield(); // Give time to WiFi stack
}