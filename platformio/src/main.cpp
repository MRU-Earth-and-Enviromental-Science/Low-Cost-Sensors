#include <Arduino.h>
#include "PM25.h"

Plantower_PMS7003 pms7003;

void setup()
{
  Serial.begin(115200); // Debug via USB
  delay(1000);
  Serial.println("Starting PMS7003 on Serial1 (GPIO4)...");

  // Serial1 remapped: RX = GPIO4, TX not used
  Serial1.begin(9600, SERIAL_8N1, 4, -1);
  pms7003.init(&Serial1); // Use Serial1 for PMS7003
}

void loop()
{
  pms7003.updateFrame();

  if (pms7003.hasNewData())
  {
    Serial.printf("\nSensor Version: %d    Error Code: %d\n", pms7003.getHWVersion(), pms7003.getErrorCode());
    Serial.printf("    PM1.0 (ug/m3): %2d     [atmos: %d]\n", pms7003.getPM_1_0(), pms7003.getPM_1_0_atmos());
    Serial.printf("    PM2.5 (ug/m3): %2d     [atmos: %d]\n", pms7003.getPM_2_5(), pms7003.getPM_2_5_atmos());
    Serial.printf("    PM10  (ug/m3): %2d     [atmos: %d]\n", pms7003.getPM_10_0(), pms7003.getPM_10_0_atmos());
    Serial.printf("    RAW: %2d[>0.3] %2d[>0.5] %2d[>1.0] %2d[>2.5] %2d[>5.0] %2d[>10]\n",
                  pms7003.getRawGreaterThan_0_3(),
                  pms7003.getRawGreaterThan_0_5(),
                  pms7003.getRawGreaterThan_1_0(),
                  pms7003.getRawGreaterThan_2_5(),
                  pms7003.getRawGreaterThan_5_0(),
                  pms7003.getRawGreaterThan_10_0());
  }

  delay(1000);
}