#ifndef PM25_H
#define PM25_H
#include <Wire.h>

extern int ledPower;
extern int samplingTime;
extern int measurePin;
extern int deltaTime;
extern int sleepTime;
extern float voMeasured;
extern float calcVoltage;
extern float dustDensity;

void calibrateSensorPM25();

#endif