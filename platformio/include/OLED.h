#ifndef OLED_H
#define OLED_H

#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

#include <Wire.h>
#include <Arduino.h>

extern Adafruit_SSD1306 display;

// Wrapper functions
void initOLED();
void oledPrint(const String &text);
void oledPrintln(const String &text);
void oledClear();

#endif