// #include "K30.h"

// int K30_I2C::readCO2(int &CO2level)
// {
//     byte recValue[4] = {0};
//     Wire.beginTransmission(_i2c_address);
//     Wire.write(0x22);
//     Wire.write(0x00);
//     Wire.write(0x08);
//     Wire.write(0x2A);
//     Wire.endTransmission();
//     delay(30);
//     Wire.requestFrom(_i2c_address, 4);
//     delay(20);
//     byte i = 0;
//     while (Wire.available() && i < 4)
//         recValue[i++] = Wire.read();
//     CO2level = (recValue[1] << 8) + recValue[2];
//     byte checkSum = recValue[0] + recValue[1] + recValue[2];
//     if (i == 0)
//         return 2;
//     else if (checkSum == recValue[3])
//         return 0;
//     else
//         return 1;
// }