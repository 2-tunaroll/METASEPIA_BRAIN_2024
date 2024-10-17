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
float last_port_angle[5];
float last_starboard_angle[5];

// functino to initialise the servomotors, set required servomotor values and drive them to a neutral position
void servo::init()
{
  pwm.begin();
  /*s
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

  for (int i = 0; i < 5; i++) {
    last_port_angle[i] = 0.0;
    last_starboard_angle[i] = 0.0;
  }

  for (int i = 0; i < 30; i++){
    set_positions(0,240,0,SINWAVE,B);
    delay(100);
  }
}

// set servo positions based off angle required
void servo::set_positions(float amplitude, float wavelength, float time_milli, int wavetype, int side)
{
  float angle; 
  float pulse_port;
  float pulse_starboard;  
  float height;
  float port_angle;
  float starboard_angle;

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
      break;

      case SINANDFLAT:
      angle = waveform::calc_angle_sinandflat(amplitude, wavelength, time_milli, servonum);
      break;

      default:
      break;
    }
    
    
    // PORT side angle set
    if (side == B || side == P) {
      angle += NEUTRALS_PORT[servonum];

      if (angle > last_port_angle[servonum] + MAX_ANGLE_DELTA) {
        angle = last_port_angle[servonum] + MAX_ANGLE_DELTA;
      } else if (angle < last_port_angle[servonum] - MAX_ANGLE_DELTA) {
        angle = last_port_angle[servonum] - MAX_ANGLE_DELTA;
      }
      pulse_port = map(angle, -90, 90, SERVOMIN, SERVOMAX); 
      pwm.setPWM(servonum, 0, pulse_port);
      last_port_angle[servonum] = angle;
    }

    // STARBOARD side angle set
    if (side == B || side == S) {
      angle += NEUTRALS_STARBOARD[servonum];

      if (angle > last_starboard_angle[servonum] + MAX_ANGLE_DELTA) {
        angle = last_starboard_angle[servonum] + MAX_ANGLE_DELTA;
      } else if (angle < last_starboard_angle[servonum] - MAX_ANGLE_DELTA) {
        angle = last_starboard_angle[servonum] - MAX_ANGLE_DELTA;
      }
      pulse_starboard = map(-angle, -90, 90, SERVOMIN, SERVOMAX);
      pwm.setPWM(servonum + NUM_SERVOS, 0, pulse_starboard);
      last_starboard_angle[servonum] = angle;
    }
  }
}


time_milli_t servo::drive_fins(float surge, float sway, float pitch, float yaw, float amp, time_milli_t time){
  
  // set positions based off time from previous 
  servo::set_positions(amp, 480, time.port, SINWAVE, P);
  servo::set_positions(amp, 480, time.starboard, SINWAVE, S);

  // calculate time increments (metasepia forwards, fins backwards)
  float time_inc_P = -surge*MAX_TIME_INC;
  float time_inc_S = -surge*MAX_TIME_INC;

  // yaw is signed (works it self out)
  time_inc_P += yaw*MAX_TIME_INC;
  time_inc_S -= yaw*MAX_TIME_INC;
  

  // clamp both incrementors to within max and min
  time_inc_P = clamp(time_inc_P);
  time_inc_S = clamp(time_inc_S);

  // increment both times 
  time.port += time_inc_P;
  time.starboard += time_inc_S;  

  // return tuple of times
  return time;
}

float servo::clamp(float time_inc){
  if (time_inc > MAX_TIME_INC){
    time_inc = MAX_TIME_INC;
  } else if (time_inc < -MAX_TIME_INC){
    time_inc = -MAX_TIME_INC;
  }
  return time_inc;
}

