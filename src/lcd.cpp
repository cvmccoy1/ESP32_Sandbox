// Include the Serial Peripheral Interface (SPI) Library
#include <SPI.h>

// Include Arduino Framework Library
#include <Arduino.h>

// Include PCF8574 Library for I2C LCD
#include <LiquidCrystal_PCF8574.h>

// Include Wire Library for I2C Communications
#include <Wire.h>

// Include Assert Library
#include <assert.h>

#include "lcd.h"

// Define LCD object
LiquidCrystal_PCF8574 lcd(lcdI2Caddr); 

void SetupDisplay()
{
  // Setup the LCD Display
  lcd.begin(lcdColumns, lcdRows); 
  lcd.setBacklight(255);
}

// Function to update the LCD display
void UpdateDisplay(int servoDegree[NUM_SERVOS])
{
  assert(lcdRows <= NUM_SERVOS);  
  // Clear the display
  lcd.clear();
  lcd.home();

  for (int n = 0; n < NUM_SERVOS; n++)
  {  
    // Print on LCD
    lcd.setCursor(0, n);
    lcd.printf("Servo %d: %d", n, servoDegree[n]);

    // Print to Serial Monitor   
    //Serial.printf("Servo %d: %d", n, servoDegree[n]);
    //Serial.print("\t");
  }
  //Serial.print("\n");
}
