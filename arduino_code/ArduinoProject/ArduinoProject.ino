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
unsigned long time_milli = 0;
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
    // update positions
    float amp = 50*((float)surge/128.0);

    if (surge > 0){
      servo::set_positions(amp, 240, time_milli, SINWAVE);
    }

    // increment time for waveform
    time_milli += 10;
    startMillis = currentMillis;
  }
}
