#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "../include/OLED.h"

// ------------ RADIO / GPS CONFIG ------------------------------------
constexpr uint8_t PEER_ADDR[6] = {0x10, 0x06, 0x1C, 0xF2, 0x02, 0x50};
constexpr uint8_t WIFI_CH = 6;
constexpr int RX_PIN = 4;
constexpr int TX_PIN = 5;
constexpr uint32_t BAUD_UART = 115200;
// --------------------------------------------------------------------

struct __attribute__((packed)) GpsPacket
{
    float lat;
    float lon;
};

GpsPacket gps{NAN, NAN};

String nmeaLine;
bool lastSendOk = false;
bool gotFix = false;
uint32_t tLastSend = 0;

// --------------------------------------------------------------------
static double dmToDeg(const String &dm, char hemi)
{
    if (dm.length() < 4)
        return NAN;
    double v = dm.toDouble();
    int d = int(v / 100);
    double m = v - d * 100;
    double dec = d + m / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -dec : dec;
}

void parseGpsFromPi()
{
    while (Serial2.available())
    {
        char c = Serial2.read();

        if (c == '\n' || c == '\r')
        {
            if (nmeaLine.startsWith("$GPRMC"))
            {
                int idx = 0;
                String fld[12];
                for (int i = 0; i < 12 && idx != -1; ++i)
                {
                    int nxt = nmeaLine.indexOf(',', idx);
                    fld[i] = (nxt == -1) ? nmeaLine.substring(idx)
                                         : nmeaLine.substring(idx, nxt);
                    idx = (nxt == -1) ? -1 : nxt + 1;
                }

                if (fld[2] == "A")
                {
                    gps.lat = dmToDeg(fld[3], fld[4].charAt(0));
                    gps.lon = dmToDeg(fld[5], fld[6].charAt(0));
                    gotFix = true;
                    Serial.printf("GPS fix  %.6f, %.6f\n", gps.lat, gps.lon);
                    oledPrint("Lat: " + String(gps.lat, 6) + "\nLon: " + String(gps.lon, 6));
                }
            }
            nmeaLine = "";
        }
        else if (isPrintable(c))
        {
            nmeaLine += c;
        }
    }
}

// --------------------------------------------------------------------
void onDataSent(const uint8_t *, esp_now_send_status_t s)
{
    lastSendOk = (s == ESP_NOW_SEND_SUCCESS);
    Serial.printf("ESP-NOW send %s\n", lastSendOk ? "OK" : "FAIL");
    oledPrint(lastSendOk ? "TX OK" : "TX FAIL");
}

// --------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    Wire.begin(21, 22);
    initOLED();
    oledPrint("Booting...");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

    if (esp_now_init() != ESP_OK)
    {
        oledPrint("ESP-NOW init ERR");
        while (true)
        {
        }
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, PEER_ADDR, 6);
    p.channel = WIFI_CH;
    p.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&p));

    Serial2.begin(BAUD_UART, SERIAL_8N1, RX_PIN, TX_PIN);
    oledPrint("Listening NMEA...");
}

// --------------------------------------------------------------------
void loop()
{
    parseGpsFromPi();

    if (millis() - tLastSend >= 1000 && gotFix)
    {
        esp_now_send(PEER_ADDR, reinterpret_cast<uint8_t *>(&gps), sizeof(gps));
        tLastSend = millis();
    }
}