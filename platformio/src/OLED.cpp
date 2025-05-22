#include "OLED.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initOLED()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        // You can also flash an LED here for error indication
        while (true)
            ; // halt if OLED fails
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.display();
}

void oledPrint(const String &text)
{
    display.print(text);
    display.display();
}

void oledPrintln(const String &text)
{
    display.println(text);
    display.display();
}

void oledClear()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.display();
}