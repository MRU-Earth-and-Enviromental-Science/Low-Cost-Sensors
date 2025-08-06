#ifndef CO_H
#define CO_H

#include <Arduino.h>
#include <Wire.h>

#define MQ7_PIN 35
#define RL_VALUE_MQ7 0.33
#define RO_CLEAN_AIR_FACTOR_MQ7 4.4

extern float Ro_MQ7;

float calibrateSensorMQ7();
float readMQ7();

#endif // CO_H
