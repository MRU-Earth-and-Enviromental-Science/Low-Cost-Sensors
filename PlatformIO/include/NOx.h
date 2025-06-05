#ifndef NOx_H
#define NOx_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MQ135_PIN 32
#define RL_VALUE 0.33
#define RO_CLEAN_AIR_FACTOR_MQ135 3.0

extern float Ro_MQ135;

float calibrateSensorMQ135();
float readMQ135();

#endif