#include "K30_I2C.h"

K30_I2C::K30_I2C(int i2c_address)
{
    _i2c_address = i2c_address;
}

int K30_I2C::readCO2(int &CO2level)
{
    byte recValue[4] = {0, 0, 0, 0};

    Wire.beginTransmission(_i2c_address);
    Wire.write(0x22); // Read CO2 command sequence
    Wire.write(0x00);
    Wire.write(0x08);
    Wire.write(0x2A);
    Wire.endTransmission();
    delay(30); // Allow sensor to process

    Wire.requestFrom(_i2c_address, 4);
    delay(20); // Small delay before reading

    byte i = 0;
    while (Wire.available() && i < 4)
    {
        recValue[i++] = Wire.read();
    }

    // Calculate checksum
    byte checkSum = recValue[0] + recValue[1] + recValue[2];
    CO2level = (recValue[1] << 8) + recValue[2];

    if (i == 0)
        return 2; // No data
    else if (checkSum == recValue[3])
        return 0; // Success
    else
        return 1; // Checksum fail
}