//-------------------------------------------------------------------------------------------------------------------------------//
// Filename:      ArduinoProject.ino
// Date Created:  26/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Main file for the Metasepia Honours Project
//-------------------------------------------------------------------------------------------------------------------------------//

/*************************************************** 
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 // (^^^ file has been changed significantly, leaving this comment in for now, maybe remove? ^^^)

// hardware includes
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// module includes
#include "robot.h"
#include "servo.h"

// library includes
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 25;
int analogVoltagePin = A0;

input_command inputs;

// setup function - baud rate, serial printing, initialising
void setup() {
  Serial.begin(115200);
  while(!Serial) {} 
  
  startMillis = millis();
  servo::init();

  delay(2000);
}

//main loop + variables
float time_milli = 0;
String message = "test";
bool status = true;

int8_t surge = 0;
int8_t sway = 0;
int8_t pitch = 0;
int8_t yaw = 0;

int voltage = 0;
byte voltage_low;
byte voltage_high;
// uint8_t send_voltage = 0;
float amp = 0;

float surge_ratio;
float sway_ratio;

void loop() {
  currentMillis = millis();

  voltage = analogRead(analogVoltagePin);
  voltage_high  = voltage >> 8;     // Extract the high byte (the upper 2 bits)
  voltage_low   = voltage & 0xFF;   // Extract the low byte (the lower 8 bits)
  
  if (Serial.available() >= 5) {
    if (Serial.read() == 1) {
      surge 	= Serial.read() - 128;
      sway 	  = Serial.read() - 128;
      pitch 	= Serial.read() - 128;
      yaw 	  = Serial.read() - 128;
        
      Serial.write(0b1);
      Serial.write(voltage_high);
      Serial.write(voltage_low);
    } else {
      Serial.println("dog");
    }
  }

  if (currentMillis - startMillis >= period) {
    // amp = 50;
    // update positions
    // drive_fins(surge, sway, pitch, yaw, amp);

    // Surge OR sway - rudimentry
    amp = 50;
    if (abs(surge) > abs(sway)) {
      servo::set_positions(amp, 240, time_milli, SINWAVE, B);
    } 
    else if (sway > 0) {
        servo::set_positions(amp, 480, time_milli, STANDINGWAVE, P);
      }
    else if (sway < 0) {
      servo::set_positions(amp, 480, time_milli, STANDINGWAVE, S);
    } 
    else {
      // reset time if no input
      time_milli = 0;
    }

    // increment time for waveform
    time_milli += 7;
    startMillis = currentMillis;
  }
}
