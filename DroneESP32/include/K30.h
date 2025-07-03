#ifndef K30_H
#define K30_H
#define K30_ADDRESS 0x68
#include <Wire.h>
#include <Arduino.h>

class K30_I2C
{
    int _i2c_address;

public:
    K30_I2C(int i2c_address) { _i2c_address = i2c_address; }
    int readCO2(int &CO2level);
};

extern K30_I2C k30;

#endif