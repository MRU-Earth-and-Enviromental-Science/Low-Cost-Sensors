#include "K30.h"

int K30_I2C::readCO2(int &CO2level)
{
    byte recValue[4] = {0};
    Wire.beginTransmission(_i2c_address);
    Wire.write(0x22);
    Wire.write(0x00);
    Wire.write(0x08);
    Wire.write(0x2A);
    Wire.endTransmission();
    delay(30);

    int bytesRead = Wire.requestFrom(_i2c_address, 4);
    if (bytesRead != 4)
    {
        return 3; // Failed to read required bytes
    }

    byte i = 0;
    while (Wire.available() && i < 4)
        recValue[i++] = Wire.read();

    if (i < 4)
    {
        return 2; // Incomplete read
    }

    byte checkSum = recValue[0] + recValue[1] + recValue[2];
    if (checkSum != recValue[3])
    {
        return 1; // Checksum mismatch
    }

    CO2level = (recValue[1] << 8) + recValue[2];
    return 0; // Success
}