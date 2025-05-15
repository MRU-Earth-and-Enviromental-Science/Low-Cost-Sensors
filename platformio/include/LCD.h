#ifndef LCD_H
#define LCD_H

#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4

extern LiquidCrystal_I2C lcd;
void printToSerialAndLCD(const String &message);
void initLCD();

#endif