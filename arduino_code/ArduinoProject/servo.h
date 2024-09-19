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
  void set_positions(float amplitude, float wavelength, unsigned long time_milli, int wavetype);
  void set_neutrals();
}

#endif // SERVO_H
