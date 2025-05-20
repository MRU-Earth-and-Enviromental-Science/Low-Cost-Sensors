// #include <LCD.h>
// LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// void printToSerialAndLCD(const String &message)
// {
//     Serial.println(message);

//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print(message.substring(0, 16));
//     if (message.length() > 16)
//     {
//         lcd.setCursor(0, 1);
//         lcd.print(message.substring(16, 32));
//     }
// }

// void initLCD()
// {
//     lcd.init();
//     lcd.backlight();
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("LCD Test OK");
// }