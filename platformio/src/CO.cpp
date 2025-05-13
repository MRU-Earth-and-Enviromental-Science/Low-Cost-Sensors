#include "CO.h"

float calculateResistanceMQ9(int adcValue)
{
    float voltage = adcValue * (3.3 / 4095.0);
    return RL_VALUE_MQ9 * (3.3 - voltage) / voltage;
}

float readMQ9()
{
    int adcValue = analogRead(MQ9_PIN);
    float rs1 = calculateResistanceMQ9(adcValue);
    float ratio = rs1 / Ro_MQ9;
    // Based on MQ-9 datasheet curve (RL=20kΩ), using log(ppm) = (log10(Rs/Ro) - b) / m
    float m = -0.38;
    float b = 1.3;
    float ppm_log = (log10(ratio) - b) / m;
    return pow(10, ppm_log);
}

float calibrateSensorMQ9()
{
    // Ro is the resistance in clean air at 0.33ppm CO, used as baseline
    float val = 0.0;
    for (int i = 0; i < 50; i++)
    {
        val += calculateResistanceMQ9(analogRead(MQ9_PIN));
        delay(100);
    }
    val /= 50.0;
    return val / RO_CLEAN_AIR_FACTOR_MQ9;
}