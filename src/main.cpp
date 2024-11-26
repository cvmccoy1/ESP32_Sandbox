/*
  Servo Motor Controller Demo for PlatformIO
  servo-control-demo.cpp
  Controls 2 servo motors, uses PCA9685 PWM Controller
  Displays status on 16x2 LCD
  Uses LiquidCrystal PCF8574 LCD Library
  Uses Adafruit PWM library
  Uses 2 potentiometers
  Uses I2C LCD Display

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Include the Serial Peripheral Interface (SPI)
#include <SPI.h>

// Include Arduino framework
#include <Arduino.h>

// Include PCF8574 Library for I2C LCD
#include <LiquidCrystal_PCF8574.h>

// Include Wire Library for I2C Communications
#include <Wire.h>

// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
// Define LCD I2C Address - change if required
const int lcdI2Caddr = 0x27;
// Set the LCD number of columns and rows
const int lcdColumns = 16;
const int lcdRows = 2;

// Define LCD object
LiquidCrystal_PCF8574 lcd = NULL; // = lcd(i2c_addr); 

// Servo mimimum and maximum angles
#define MIN_ANGLE   0
#define MAX_ANGLE   180

// PWM Parameter Definitions
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50


// Define Servo Drive I2C Address - change if required
const int servoMotorI2Caddr = 0x40;

// Define PWM object
Adafruit_PWMServoDriver servoDriver = NULL; // = Adafruit_PWMServoDriver(servoMotorI2Caddr)
#define NUM_MOTORS  2

// Define Potentiometer Inputs
int potInput[NUM_MOTORS] = { A6, A7 };  // GPIO34, GPIO35

// Define Motor Outputs on PCA9685 board
const int motor0 = 0;
const int motor1 = 1;

// Define current Motor position variables
int mtrDegree[NUM_MOTORS] = { 0, 0 };

// Define the interrupt flag 
volatile bool heartbeatFlag = LOW;
// Define the interrupt service routine (ISR)
volatile bool interruptFlag = false;

// Timer interrupt handler function
void IRAM_ATTR onTimer(void* arg) 
{
  interruptFlag = true;
}

extern void findI2CDevices();

// Function to move motor to specific position
void moveMotorDeg(int moveDegree, int motorOut)
{
   // Convert to pulse width
  int pulse_wide = map(moveDegree, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.printf("moveMotorDeg(%d, %d) pulse_wide = %d; pulse_width = %d \n", moveDegree, motorOut, pulse_wide, pulse_width);
  
  //Control Motor
  servoDriver.setPWM(motorOut, 0, pulse_width);
}

// Function to convert potentiometer position into servo angle
int getDegree(int controlIn)
{
  // Read values from potentiometer
  int potVal = analogRead(controlIn);
  
  // Calculate angle in degrees
  int srvDegree = map(potVal, 0, 4095, 0, 180);
  
  //Serial.printf("Pot %d: Value: %d: Angle: %d \n", controlIn, potVal, srvDegree);

  // Return angle in degrees
  return srvDegree;
}

// Function to read the current Potentiometer positions, convert to an angle and update the motor if changed
boolean updateMotors()
{
  boolean motorsUpdated = false;
  for (int n = 0; n < NUM_MOTORS; n++)
  {
    // Get get the current control position
    int newMtrDegree = getDegree(potInput[n]);

    // Move motor only if control position has changed
    if (newMtrDegree != mtrDegree[n]) {
      // Move motor
      moveMotorDeg(newMtrDegree, n);
      // Update motor moved variable
      motorsUpdated = true;
      // Update to the new position
      mtrDegree[n] = newMtrDegree;
    }
  }
  return motorsUpdated;
}

// Function to update the LCD display
void updateDisplay()
{
  // Clear the display
  lcd.clear();
  lcd.home();

  for (int n = 0; n < NUM_MOTORS; n++)
  {  
    // Print on first row of LCD
    lcd.setCursor(0, n);
    lcd.printf("Servo %d: %d", n, mtrDegree[n]);

    // Print to Serial Monitor   
    Serial.printf("Motor %d: %d", n, mtrDegree[n]);
    Serial.print("\t");
  }
  Serial.print("\n");
}

void setup() 
{
  // Start Serial Monitor                                                 
  Serial.begin(115200);  

   // Initialize the Heartbeat LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the LCD Display
  lcd = LiquidCrystal_PCF8574(lcdI2Caddr); 
  lcd.begin(lcdColumns, lcdRows); 
  lcd.setBacklight(255);

  // Setup PWM Controller object
  servoDriver = Adafruit_PWMServoDriver(servoMotorI2Caddr);
  servoDriver.begin();
  servoDriver.setPWMFreq(FREQUENCY);

  // Set the initial motor positions
  updateMotors();
  // Set the initial display content
  updateDisplay();

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
  esp_timer_start_periodic(timer, 1000000); // 1 second interva
}
 
void loop() 
{
  //findI2CDevices();

  // Add short delay
  delay(100);
  
  // Only if the motor positions have change will the following function return true
  if (updateMotors()) 
    updateDisplay();

  if (interruptFlag)
  {
    interruptFlag = false; // Reset the flag
    heartbeatFlag = !heartbeatFlag;
    // Set the Heartbeat LED
    digitalWrite(LED_BUILTIN, heartbeatFlag);
        // Print a message or perform any task you need
    //Serial.printf("One second has passed!  heartbeatFlag = %s (%d)\n", heartbeatFlag ? "HIGH" : "LOW", heartbeatFlag);
  }
}