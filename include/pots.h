#pragma once

#include "servos.h"

extern int potInput[NUM_SERVOS];

void SetupPotentiometers();
int (&GetPotInputValues())[NUM_SERVOS];
int GetMaxPotValue();