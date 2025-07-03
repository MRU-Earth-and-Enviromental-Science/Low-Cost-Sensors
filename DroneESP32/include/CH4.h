#ifndef CH4_H
#define CH4_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MQ4_PIN 34
#define RL_VALUE 0.33
#define RO_CLEAN_AIR_FACTOR_MQ4 4.4

extern float Ro_MQ4;

float calibrateSensorMQ4();
float readMQ4();

#endif // CH4_H