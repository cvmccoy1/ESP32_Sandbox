#pragma once

#define NUM_SERVOS  2

// Servo mimimum and maximum angles
#define MIN_ANGLE   0
#define MAX_ANGLE   180

// PWM Parameter Definitions - Change If Required

// The frequency might need to change to 50 MHz if you live outside the USA.
#define FREQUENCY             60   // in MHz

// You might need to experiment with the values used in the map() function to get the
//   exact range for your specific servo. Some servos may require slightly different pulse length ranges.
#define MIN_PULSE_LENGTH      150  // in ticks
#define MAX_PULSE_LENGTH      675

 // Define Servo Drive I2C Address - Change If Required
const int servoDriverI2Caddr = 0x40;

void SetupServos();
int* UpdateServos(int potInput[NUM_SERVOS], int maxPotValue);
