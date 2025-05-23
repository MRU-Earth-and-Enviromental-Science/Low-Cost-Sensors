#include "OLED.h"

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initOLED()
{
    display.begin();

    display.clearDisplay();
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds("Welcome to the Sensor System", 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
    delay(500);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.print("Welcome to the Sensor System");
    display.display();
    delay(2000);
}

void oledClear()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.display();
}

void oledPrint(const String &text)
{
    oledClear();
    display.setTextWrap(true);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.print(text);
    display.display();
}

void oledPrintln(const String &text)
{
    oledClear();
    display.setTextWrap(true);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println(text);
    display.display();
}