#include "../include/Temp.h"

DHT dht(DHTPIN, DHTTYPE);

void readTemperatureHumidity(float &temperature, float &humidity)
{
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
        Serial.println("DHT read failed, using -1");
        h = -1.0;
        t = -1.0;
    }

    temperature = t;
    humidity = h;
}