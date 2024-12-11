// Include Arduino Framework Library
#include <Arduino.h>

#include "heartbeat.h"

// Declare a handle for the task running on Core 1
TaskHandle_t TaskCore1 = nullptr;

// Local Forward References
void setupCore1();
void loopCore1();
void IRAM_ATTR taskOnCore1(void *parameter);

void StartCore1()
{
  // Create a task that will run on Core 1
  // The task will use the TaskOnCore1 function and run on Core 1
  xTaskCreatePinnedToCore(
    taskOnCore1,       // Task function
    "TaskCore1",       // Task name
    10000,             // Stack size (in words)
    NULL,              // Task parameters
    1,                 // Task priority
    &TaskCore1,        // Task handle
    0                  // Core 0 (use Core 0 for this task, Main App is on Core 1)
  );
}

// Function that will run on Core 1
void IRAM_ATTR taskOnCore1(void *parameter)
{
  setupCore1();
  while(true) 
  {
    loopCore1();
  }
}

// Setup Function for Core1 - Add Setup code for Core1 here
void setupCore1()
{
  Heartbeat::Instance().StartHeartbeatLED();
}

// Loop Function for Core1 - Add Execution code for Core1 here
void loopCore1()
{
  //Serial.println("Running on Core 1");
}