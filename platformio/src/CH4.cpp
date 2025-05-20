// #include "CH4.h"

// float calculateResistanceMQ4(int adcValue)
// {
//     float voltage = adcValue * (3.3 / 4095.0);
//     return RL_VALUE * (3.3 - voltage) / voltage;
// }

// float readMQ4()
// {
//     int adcValue = analogRead(MQ4_PIN);
//     float rs1 = calculateResistanceMQ4(adcValue);
//     float ratio = rs1 / Ro_MQ4;
//     // Based on MQ-4 datasheet curve (RL=20kΩ), using log(ppm) = (log10(Rs/Ro) - b) / m
//     float m = -0.38;
//     float b = 1.3;
//     float ppm_log = (log10(ratio) - b) / m;
//     return pow(10, ppm_log);
// }

// float calibrateSensorMQ4()
// {
//     // Ro is the resistance in clean air at 1000ppm CH4, used as baseline
//     float val = 0.0;
//     for (int i = 0; i < 50; i++)
//     {
//         val += calculateResistanceMQ4(analogRead(MQ4_PIN));
//         delay(100);
//     }
//     val /= 50.0;
//     return val / RO_CLEAN_AIR_FACTOR_MQ4;
// }