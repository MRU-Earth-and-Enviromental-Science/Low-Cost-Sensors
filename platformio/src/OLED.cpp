#include "../include/OLED.h"
#include <Arduino.h>
#include <vector>

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void printWrappedText(const String &text, int x, int y)
{
    int lineHeight = 8;
    int charWidth = 6;
    std::vector<String> words;
    String word;
    for (char c : text)
    {
        if (c == ' ')
        {
            if (word.length() > 0)
                words.push_back(word);
            word = "";
        }
        else
        {
            word += c;
        }
    }
    if (word.length() > 0)
        words.push_back(word);

    std::vector<String> lines;
    String currentLine = "";
    for (const String &w : words)
    {
        if ((currentLine.length() + w.length()) * charWidth > SCREEN_WIDTH)
        {
            lines.push_back(currentLine);
            currentLine = w + " ";
        }
        else
        {
            currentLine += w + " ";
        }
    }
    if (currentLine.length() > 0)
        lines.push_back(currentLine);

    for (size_t i = 0; i < lines.size(); i++)
    {
        String &line = lines[i];
        int lineWidth = line.length() * charWidth;
        int lineX = (SCREEN_WIDTH - lineWidth) / 2;
        int lineY = y + i * lineHeight;
        display.setCursor(lineX, lineY);
        display.print(line);
    }
}

void initOLED()
{
    display.begin();
    display.clearDisplay();

    display.fillRect(48, 20, 33, 25, SH110X_WHITE); // Added rectangle
    display.fillTriangle(64, 60, 80, 45, 48, 45, SH110X_WHITE);
    display.fillTriangle(64, 20, 48, 20, 48, 8, SH110X_WHITE); // Left triangle
    display.fillTriangle(64, 20, 80, 20, 80, 8, SH110X_WHITE); // Right triangle

    display.display();
    delay(6000);
    display.clearDisplay();

    auto slideText = [](const String &text)
    {
        int16_t x1, y1;
        uint16_t w, h;
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

        int targetX = (SCREEN_WIDTH - w) / 2;
        int targetY = (SCREEN_HEIGHT - h) / 2;

        for (int i = 0; i <= 10; ++i)
        {
            display.clearDisplay();
            if (i % 2 == 0 || i > 7) // crude dither
            {
                printWrappedText(text, targetX, targetY);
            }
            display.display();
            delay(100);
        }

        delay(1200);
    };

    slideText("Mount Royal University");
    slideText("Welcome to the Gas Sensor System");
    slideText("Powered by ESP32");

    display.clearDisplay();
    slideText("Booting...");
    display.clearDisplay();
    slideText("Checking sensors...");
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
    display.setTextWrap(false);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    int x = (SCREEN_WIDTH - w) / 2;
    int y = (SCREEN_HEIGHT - h) / 2;
    display.setCursor(x, y);
    display.print(text);
    display.display();
}

void oledPrintln(const String &text)
{
    oledClear();
    display.setTextWrap(false);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    int x = (SCREEN_WIDTH - w) / 2;
    int y = (SCREEN_HEIGHT - h) / 2;
    display.setCursor(x, y);
    display.println(text);
    display.display();
}