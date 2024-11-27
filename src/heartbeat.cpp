#include <Arduino.h>

// Define the LED state flag 
volatile bool heartbeatFlag = LOW;

// Timer interrupt handler function
void IRAM_ATTR onTimer(void* arg) 
{
  heartbeatFlag = !heartbeatFlag;
  // Set the Heartbeat LED state
  digitalWrite(LED_BUILTIN, heartbeatFlag);
  //Serial.printf("One second has passed!  heartbeatFlag = %s (%d)\n", heartbeatFlag ? "HIGH" : "LOW", heartbeatFlag);
}

void SetupHeartbeatLED()
{  
  // Initialize the Heartbeat LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Create an ESP timer that triggers every 1 second (1000000 microseconds)
  esp_timer_handle_t timer;
  esp_timer_create_args_t timer_args =
  {
    .callback = onTimer,
    .arg = NULL,
    .name = "one_second_timer"
  };
  // Initialize the timer
  esp_timer_create(&timer_args, &timer);

  // Start the timer to trigger every 1 second (1,000,000 microseconds)
  esp_timer_start_periodic(timer, 1000000); // 1 second interval
}

