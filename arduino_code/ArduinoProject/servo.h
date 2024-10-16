//------------------------------------------------------------------------------------------------------------------------//
// Filename:      servo.h
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Header file for the servo module. All direct control of servos are within this module
//------------------------------------------------------------------------------------------------------------------------//

#ifndef SERVO_H
#define SERVO_H

#include "robot.h"

typedef struct {
  float port;
  float starboard;
} time_milli_t;

namespace servo 
{
  void init();
  void set_positions(float amplitude, float wavelength, float time_milli, int wavetype, int side);
  void set_neutrals(int side);
  time_milli_t drive_fins(float surge, float sway, float pitch, float yaw, float amp, time_milli_t time_milli);
  float clamp(float time_inc);
  float set_rudder(float pitch);
}



#endif // SERVO_H
