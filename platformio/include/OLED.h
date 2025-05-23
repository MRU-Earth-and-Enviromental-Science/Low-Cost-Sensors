#ifndef OLED_H
#define OLED_H

#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

extern Adafruit_SH1106G display;

void initOLED();
void oledPrint(const String &text);
void oledPrintln(const String &text);
void oledClear();

#endif