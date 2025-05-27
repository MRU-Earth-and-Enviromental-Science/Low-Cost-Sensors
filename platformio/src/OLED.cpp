#include "OLED.h"

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initOLED()
{
    display.begin();
    display.clearDisplay();

    display.fillRect(40, 20, 64, 20, SH110X_WHITE); // Added rectangle
    display.fillTriangle(64, 60, 88, 40, 40, 40, SH110X_WHITE);
    display.fillTriangle(64, 24, 32, 24, 32, 12, SH110X_WHITE); // Left triangle
    display.display();
    delay(5000);
    delay(1500);
    display.clearDisplay();

    // --- Slide-in text animation helper ---
    auto slideText = [](const String &text)
    {
        int16_t x1, y1;
        uint16_t w, h;
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

        for (int x = -w; x <= (SCREEN_WIDTH - w) / 2; x += 4)
        {
            display.clearDisplay();
            display.setCursor(x, (SCREEN_HEIGHT - h) / 2);
            display.print(text);
            display.display();
            delay(15);
        }
        delay(700);
        for (int x = (SCREEN_WIDTH - w) / 2; x <= SCREEN_WIDTH; x += 4)
        {
            display.clearDisplay();
            display.setCursor(x, (SCREEN_HEIGHT - h) / 2);
            display.print(text);
            display.display();
            delay(10);
        }
    };

    slideText("Mount Royal University");
    slideText("Sensor System Ready");

    display.clearDisplay();
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