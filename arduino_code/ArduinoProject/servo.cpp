//------------------------------------------------------------------------------------------------------------------------//
// Filename:      servo.cpp
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Implementation file for the servo module. All direct control of servos are within this module
//------------------------------------------------------------------------------------------------------------------------//

#include "servo.h"
#include "robot.h"
#include "waveform.h"

#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// functino to initialise the servomotors, set required servomotor values and drive them to a neutral position
void servo::init()
{
  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // drive to a neutral position
  servo::set_neutrals();
}

// set servo positions based off angle required
void servo::set_positions(float amplitude, float wavelength, unsigned long time_milli, int wavetype)
{
  float angle; 
  float pulse;  
  float height;

  uint8_t servonum;
  
  for (servonum = 0; servonum < NUM_SERVOS; servonum++) 
  {
    switch(wavetype)
    {
      case SINWAVE:
      angle = waveform::calc_angle_sinwave(amplitude, wavelength, time_milli, servonum);
      break;

      case FLATWAVE:
      angle = waveform::calc_angle_flatwave(amplitude, wavelength, time_milli);
      break;

      case STANDINGWAVE:
      angle = waveform::calc_angle_standingwave(amplitude, wavelength, time_milli, servonum);

      case SINANDFLAT:
      angle = waveform::calc_angle_sinandflat(amplitude, wavelength, time_milli, servonum);

      default:
      break;
    }
    
    pulse = map(angle, -90, 90, SERVOMIN, SERVOMAX);


    pwm.setPWM(servonum, 0, pulse);
    //Serial.println(pulse);
  }
}

// set all servomotors to a neutral position
void servo::set_neutrals()
{
  uint8_t servonum;

  //loop through all servos and set to neutral point
  for(servonum = 0; servonum < NUM_SERVOS; servonum++)
  {
    pwm.setPWM(servonum,0,NEUTRALS[servonum]);
  }
}
