#ifndef K30_I2C_H
#define K30_I2C_H

#include <Arduino.h>
#include <Wire.h>

class K30_I2C
{
public:
    // Constructor: takes the I2C address of the sensor
    K30_I2C(int i2c_address);

    // Reads CO2 level in ppm. Returns:
    // 0 = success, 1 = checksum fail, 2 = no data
    int readCO2(int &CO2level);

private:
    int _i2c_address;
};

#endif // K30_I2C_H