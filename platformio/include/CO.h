#ifndef CO_H
#define CO_H

#include <Arduino.h>
#include <Wire.h>

#define MQ9_PIN 35
#define RL_VALUE_MQ9 0.33
#define RO_CLEAN_AIR_FACTOR_MQ9 4.4

extern float Ro_MQ9;

float calibrateSensorMQ9();
float readMQ9();

#endif // CO_H
