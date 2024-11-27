#pragma once

#include "servos.h"

// Define LCD I2C Address - Change If Required
const int lcdI2Caddr = 0x27;

// Set the LCD number of columns and rows
const int lcdColumns = 16;
const int lcdRows = 2;

void SetupDisplay();
void UpdateDisplay(int servoDegree[NUM_SERVOS]);