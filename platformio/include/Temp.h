

// Temp.h
#ifndef TEMP_H
#define TEMP_H

#include <Arduino.h>
#include <DHT.h>

#define DHTPIN 4
#define DHTTYPE DHT11

extern DHT dht;

void readTemperatureHumidity(float &temperature, float &humidity);

#endif // TEMP_H
