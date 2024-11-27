// Include Arduino Framework Library
#include <Arduino.h>

// Include Local Modules
#include "main_core1.h"
#include "lcd.h"
#include "servos.h"
#include "pots.h"
#include "utilities.h"


void setup() 
{
  // Start Serial Monitor                                                 
  Serial.begin(115200);  

  SetupDisplay();
  SetupServos();
  SetupPotentiometers();

  StartCore1();  // The Heartbeat LED runs on Core1 so that hang ups on Core0 don't effect it
}
 
void loop() 
{
  // If the LCD and/or Servo Driver don't appear to work, uncomment the following line of code
  //   in order to confirm the I2C addresses defined for these devices are correct or not.
  //   Once the addresses are found and corrected in the code, make sure to re-comment the line out.
  //FindI2CDevices();
  
  // Only if any of the servo positions have changed will the following function return a valid array
  int* servoDegreeArray = UpdateServos(GetPotInputValues(), GetMaxPotValue());
  if (servoDegreeArray != nullptr)
  {
    UpdateDisplay(servoDegreeArray);
  }
  delay(100);
}