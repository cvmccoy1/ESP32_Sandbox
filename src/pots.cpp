// Include Arduino Framework Library
#include <Arduino.h>

#include "servos.h"

// Define Potentiometer Inputs Ports
const int potInput[NUM_SERVOS] = { A6, A7 };  // GPIO34, GPIO35

int potValueArray[NUM_SERVOS] = {};

//  adcResolution should be set to the maximum resolution (in bits) of the ESP's ADC.
//  Since there is no built-in function to get the resolution, we set and track it manually.
const int adcResolution = 12;  
const int maxPotValue = (1 << adcResolution) - 1;  // equal to (2^adcResolution)-1

void SetupPotentiometers()
{
  // Set the ADC resolution
  analogReadResolution(adcResolution);
  printf("ADC Resolution = %d; Max Pot Value = %d\n", adcResolution, maxPotValue);
}

int (&GetPotInputValues())[NUM_SERVOS]
{
    for (int potNumber = 0; potNumber < NUM_SERVOS; potNumber++)
    {
        potValueArray[potNumber] = analogRead(potInput[potNumber]);
        //printf("Pot Value for %d = %d\n", potNumber, potValueArray[potNumber]);
    }
    return potValueArray;
}

int GetMaxPotValue()
{
    return maxPotValue;
}