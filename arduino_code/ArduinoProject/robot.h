//-------------------------------------------------------------------------------------------------------------------------------//
// Filename:      robot.h
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Constant values within different elements of the system
//-------------------------------------------------------------------------------------------------------------------------------//

#ifndef ROBOT_H
#define ROBOT_H

// servomotor constants
const int SERVOMIN  = 102; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX  = 512;// This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// neutral point for each servomotor (estimated at 300 each, refine later)
const int NEUTRALS[5] = { 300, 300, 300, 300, 300};

// Design constants
#define NUM_SERVOS 5
#define SPOKE_LENGTH_MM 140
#define FIN_LENGTH_MM 240

// Coding constants
enum Wavetypes {SINWAVE, FLATWAVE, STANDINGWAVE, SINANDFLAT};

// direction controls
struct input_command {
  // translational movements
  float surge;
  float sway;
  //float heave;

  // rotational movements
  float pitch;
  float yaw;
  //float roll
};

#endif //ROBOT_H