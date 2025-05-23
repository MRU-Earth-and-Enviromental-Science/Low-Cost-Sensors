// sensor broken do not touch

#include "keyestudio(broken).h"

int ledPower = 2;
int samplingTime = 280;
int measurePin = 32;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void calibrateSensorPM25()
{
    pinMode(ledPower, OUTPUT);
    digitalWrite(ledPower, LOW);
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(measurePin);
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower, HIGH);
    delayMicroseconds(sleepTime);
    calcVoltage = voMeasured * (5.0 / 1024.0);
    dustDensity = 170 * calcVoltage - 0.1;
}
