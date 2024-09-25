//------------------------------------------------------------------------------------------------------------------------//
// Filename:      servo.h
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Header file for the servo module. All direct control of servos are within this module
//------------------------------------------------------------------------------------------------------------------------//

#ifndef SERVO_H
#define SERVO_H

namespace servo 
{
  void init();
  void set_positions(float amplitude, float wavelength, float time_milli, int wavetype, int side);
  void set_neutrals(int side);
}

#endif // SERVO_H
