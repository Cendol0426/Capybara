#include <PS2X_lib.h>  // Include the PS2 Controller Library
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "locomotion.h"

PS2X ps2x; // Create the controller instance

// Your custom pin definitions for the Arduino Mega
#define PS2_DAT 50    
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 52



// Configuration settings
#define pressures   false
#define rumble      false

int error = 0;
byte type = 0;
byte vibrate = 0;

enum ROBOT_STATE {AUTONOMOUS, MANUAL};
ROBOT_STATE current_state = AUTONOMOUS;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Wire.begin();
  delay(500); // Give the wireless receiver a moment to power up
  
  Serial.println("Initializing PS2 Controller...");

  // Configure the gamepad with your custom pins
  // config_gamepad(clock, command, attention, data, Pressures?, Rumble?)
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.println("Success: PS2 Controller found and configured!");
  }  
  else if(error == 1) {
    Serial.println("Error: No controller found. Check your pins (35, 36, 37, 38) and VCC/GND.");
  }
  else if(error == 2) {
    Serial.println("Error: Controller found but refusing to accept commands.");
  }
  else if(error == 3) {
    Serial.println("Error: Controller refusing to enter Pressures mode.");
  }

  // Locomotion setup
  leftMotor.begin();
  rightMotor.begin();
  
  leftEnc.begin();
  rightEnc.begin();

  Encoder::leftInstance = &leftEnc;
  Encoder::rightInstance = &rightEnc;
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), Encoder::leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), Encoder::rightISR, RISING);

  gyro.begin();
}

void loop() {
  if(error == 1) // If no controller is connected, skip the loop
    return; 
  
  // Read the controller state 
  ps2x.read_gamepad(false, vibrate); 
  
  // --- EXAMPLE 1: BUTTON PRESSES (Triggers once when pressed) ---
  if(ps2x.ButtonPressed(PSB_START))                   
    Serial.println("Start Button Pressed");
  if(ps2x.ButtonPressed(PSB_SELECT))
  {
    Serial.println("Select Button Pressed");
    if(current_state == AUTONOMOUS)
    {
      current_state = MANUAL;
      movement.stop();
      Serial.println("Switch to Manual Control");
    }
    else
    {
      current_state = AUTONOMOUS;
      Serial.println("Switch to Autonomous Control");
    }
  }

  if(current_state == AUTONOMOUS)
  {
    cmd.update();
  }
    
  // --- EXAMPLE 2: BUTTON HELD DOWN (Triggers continuously) ---
  if(ps2x.Button(PSB_PAD_UP)) {
    Serial.println("D-Pad Up Held");
  }
  if(ps2x.Button(PSB_PAD_DOWN)) {
    Serial.println("D-Pad Down Held");
  }
  if(ps2x.Button(PSB_PAD_LEFT)) {
    Serial.println("D-Pad Left Held");
  }
  if(ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.println("D-Pad Right Held");
  }
  
  // Action Buttons
  // if(ps2x.ButtonPressed(PSB_GREEN))
  //   Serial.println("Triangle Pressed");
  // if(ps2x.ButtonPressed(PSB_RED))
  //   Serial.println("Circle Pressed");
  // if(ps2x.ButtonPressed(PSB_BLUE))
  //   Serial.println("X Pressed");
  // if(ps2x.ButtonPressed(PSB_PINK))
  //   Serial.println("Square Pressed");

  // --- EXAMPLE 3: ANALOG STICKS (Values read from 0 to 255, center is ~128) ---
  int leftStickY = ps2x.Analog(PSS_LY);
  int leftStickX = ps2x.Analog(PSS_LX);
  int rightStickY = ps2x.Analog(PSS_RY);
  int rightStickX = ps2x.Analog(PSS_RX);

  // Only print analog values if they move past a small central "deadzone" (100 to 150)
  // if(leftStickY < 100 || leftStickY > 150) {
  //   Serial.print("Left Stick Stick Y: ");
  //   // // Serial.print(leftStickX);
  //   // Serial.print(" | Y: ");
  //   Serial.println(leftStickY);
  // }

  // if(rightStickX < 100 || rightStickX > 150) {
  //   Serial.print("Right Stick Stick X: ");
  //   Serial.println(rightStickX);
  //   // Serial.print(" | Y: ");
  //   // Serial.println(rightStickY);
  // }
  
  int throttle;
  int turn;
  if(current_state == MANUAL)
  {
    throttle = map(leftStickY, 255, 0, -255, 255);
    turn = map(rightStickX, 255, 0, 255, -255);

    if(abs(throttle) < 30)
    {
      throttle = 0;
    }
    if(abs(turn) < 30)
    {
      turn = 0;
    }

   

    drive.arcade(throttle, turn);

    // Pick-up
    if(ps2x.ButtonPressed(PSB_RED))
    {
      Serial.println("Circle Pressed - Gripper Activate");
      // Gripper Activate
    }
    if(ps2x.ButtonPressed(PSB_BLUE))
    {
      Serial.println("X Pressed - Gripper Release");
      // Gripper Release
    }
  }
  
  delay(50); // Small delay to prevent spamming the Serial Monitor
}