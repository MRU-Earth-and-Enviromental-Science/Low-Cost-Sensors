#include "NOx.h"

float calculateResistanceMQ135(int adcValue)
{
    float voltage = adcValue * (3.3 / 4095.0);
    return RL_VALUE * (3.3 - voltage) / voltage;
}

float readMQ135()
{
    int adcValue = analogRead(MQ135_PIN);
    float rs1 = calculateResistanceMQ135(adcValue);
    float ratio = rs1 / Ro_MQ135;
    // Based on MQ-135 datasheet curve for NOx (RL=20kΩ), using log(ppm) = (log10(Rs/Ro) - b) / m
    float m = -0.48;
    float b = 0.36;
    float ppm_log = (log10(ratio) - b) / m;
    return pow(10, ppm_log);
}

float calibrateSensorMQ135()
{
    // Ro is the resistance in clean air at 1000ppm CH4, used as baseline
    float val = 0.0;
    for (int i = 0; i < 50; i++)
    {
        val += calculateResistanceMQ135(analogRead(MQ135_PIN));
        delay(100);
    }
    val /= 50.0;
    return val / RO_CLEAN_AIR_FACTOR_MQ135;
}