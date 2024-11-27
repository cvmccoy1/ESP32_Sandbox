// Include Arduino Framework Library
#include <Arduino.h>

// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>

#include "servos.h"

// Define PWM object
Adafruit_PWMServoDriver servoDriver(servoDriverI2Caddr);

// Define current Motor position variables
int servoPositionArray[NUM_SERVOS];

// Forward References
int calculateServoPosition(int potInputControl, int maxPotValue);
void moveServoToPosition(int servoNumber, int position);


// Function to convert potentiometer value into an angle in drgees
int calculateServoPosition(int potInputValue, int maxPotValue)
{
  int servoPosition = map(potInputValue, 0, maxPotValue, MIN_ANGLE, MAX_ANGLE);
  //Serial.printf("Pot Value: %d: --> Position: %d \n", potInputValue, servoPosition);
  return servoPosition;
}

// Function to move motor to specific position
void moveServoToPosition(int servoNumber, int position)
{
  // Convert the angle (0-180) into a pulse length
  int pulseLength = map(position, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // Adjusted pulse length for 0-180 degrees
  //Serial.printf("moveServoToPosition(servoNumber: %d, position: %d) pulseLength = %d\n", servoNumber, position, pulseLength);

  // Set the pulse width of the specified servo
  servoDriver.setPWM(servoNumber, 0, pulseLength);
}

void SetupServos()
{
  // Fill the Servo Position Array with uninitialize values
  std::fill(servoPositionArray, servoPositionArray + NUM_SERVOS, -1);

  // Setup PWM Controller object
  servoDriver.begin();
  servoDriver.setPWMFreq(FREQUENCY); 
}

// Function to read the current Potentiometer values, convert to an angle and update the servos positions if changed
int* UpdateServos(int potInputValueArray[NUM_SERVOS], int maxPotValue)
{
  boolean servosUpdated = false;
  for (int servoNumber = 0; servoNumber < NUM_SERVOS; servoNumber++)
  {
    // Get get the current servo position based off the value of the associated potentiometer
    int currentServoPosition = calculateServoPosition(potInputValueArray[servoNumber], maxPotValue);
    //printf("Position for Servo %d = %d\n", servoNumber, currentServoPosition);

    // Move servo only if the position has changed
    if (currentServoPosition != servoPositionArray[servoNumber])
    {
      //printf("Position changed for Servo %d, old position = %d, new position = %d\n", servoNumber, servoPositionArray[servoNumber], potInputValueArray[servoNumber]);
      moveServoToPosition(servoNumber, currentServoPosition);
      servoPositionArray[servoNumber] = currentServoPosition;
      servosUpdated = true;
    }
  }
  return servosUpdated ? servoPositionArray : nullptr;
}