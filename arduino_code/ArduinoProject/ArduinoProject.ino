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

time_milli_t time_milli;

// setup function - baud rate, serial printing, initialising
void setup() {
  Serial.begin(115200);
  while(!Serial) {} 
  
  startMillis = millis();
  servo::init();

  time_milli.port = 0;
  time_milli.starboard = 0;

  delay(2000);
}


//main loop + variables
String message = "test";
bool status = true;

int8_t surge = 0;
int8_t sway = 0;
int8_t pitch = 0;
int8_t yaw = 0;

float surge_prop = 0;
float sway_prop = 0;
float pitch_prop = 0;
float yaw_prop = 0;

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
    surge_prop = (float)surge/128.0;
    sway_prop = (float)sway/128.0;
    pitch_prop = (float)pitch/128.0;
    yaw_prop = (float)yaw/128.0;

    amp = 70;
    if (surge || sway || pitch || yaw){
      // if there is any input, drive the motors and reset no input timer
      time_milli = servo::drive_fins(surge_prop, sway_prop, pitch_prop, yaw_prop, amp, time_milli);

      // if there is no input, drive a wave with amplitude 0
    } else {
      servo::drive_fins(surge_prop, sway_prop, pitch_prop, yaw_prop, 0, time_milli);
      time_milli.port = 0;
      time_milli.starboard = 0; 
    }
    
    startMillis = currentMillis;
  }
}
