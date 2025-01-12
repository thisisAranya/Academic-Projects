/*
   This Code is provided by 'Tronics Lk' YouTube Channel. Visit the channel for more Tutorials.
   Channel Link 👇
   https://www.youtube.com/channel/UCYJa3gs8q49-N3TLm-7ygUw?sub_confirmation=1

   Enter the size of LCD at "lcd_Columns" and "lcd_Rows" Variables.
   You can edit "text", "rest_Time", "speed_Adjust" variables as your need.
   
   My previous tutorial will show you how to find the I2C address of a I2C connected LCD Display
   https://youtu.be/CvqHkXeXN3M

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

int lcd_Columns = 20; // Number of columns in display
int lcd_Rows = 4;     // Number of Rows in display

LiquidCrystal_I2C lcd(0x27, lcd_Columns, lcd_Rows); // set the LCD address and number of Rows and Columns

String text = "LCD Animation";  // Put your required Text here
int speed_Adjust = 300;   // Speed of moving Text
int rest_Time = 400;      // Resting time of Text animation at the edges of display

int text_Len;             // Variable to save text length

void setup()
{
  lcd.init();                      // initialize the lcd
  lcd.backlight();                 // Turn on the LCD backlight
  lcd.clear();                     // Clear the display

  text_Len = text.length(); // Finding the number of characters in the Text
}
void loop()
{
  for (int j = 0; j < lcd_Rows; j++) {
    for (int i = 0; i < (lcd_Columns - text_Len + 1);  i++) {
      lcd.clear();
      lcd.setCursor(i, j);
      lcd.print(text);
      delay(speed_Adjust);
    }
    
    delay(rest_Time);

    for (int i = (lcd_Columns - text_Len); i > -1;  i--) {
      lcd.clear();
      lcd.setCursor(i, j);
      lcd.print(text);
      delay(speed_Adjust);
    }
    
    delay(rest_Time);
  }

  for (int j = lcd_Rows - 1; j >= 0; j--) {
    lcd.clear();
    lcd.setCursor(0, j);
    lcd.print(text);
    delay(speed_Adjust);
  }
  
  delay(rest_Time);
}
