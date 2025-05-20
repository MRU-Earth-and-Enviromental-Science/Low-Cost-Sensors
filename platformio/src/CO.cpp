#include "CO.h"

float Ro_MQ7 = 0.33;

float calculateResistanceMQ7(int adcValue)
{
    float voltage = adcValue * (3.3 / 4095.0);
    return RL_VALUE_MQ7 * (3.3 - voltage) / voltage;
}

float readMQ7()
{
    int adcValue = analogRead(MQ7_PIN);
    float rs1 = calculateResistanceMQ7(adcValue);
    float ratio = rs1 / Ro_MQ7;
    // Based on MQ-7 datasheet curve (RL=20kΩ), using log(ppm) = (log10(Rs/Ro) - b) / m
    float m = -0.38;
    float b = 1.3;
    float ppm_log = (log10(ratio) - b) / m;
    return pow(10, ppm_log);
}

float calibrateSensorMQ7()
{
    // Ro is the resistance in clean air at 0.33ppm CO, used as baseline
    float val = 0.0;
    for (int i = 0; i < 50; i++)
    {
        val += calculateResistanceMQ7(analogRead(MQ7_PIN));
        delay(100);
    }
    val /= 50.0;
    return val / RO_CLEAN_AIR_FACTOR_MQ7;
}