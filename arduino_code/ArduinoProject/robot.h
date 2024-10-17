//-------------------------------------------------------------------------------------------------------------------------------//
// Filename:      robot.h
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Constant values within different elements of the system
//-------------------------------------------------------------------------------------------------------------------------------//

#ifndef ROBOT_H
#define ROBOT_H

// servomotor constants
const int SERVOMIN  = 110; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX  = 450;// This is the 'maximum' pulse length count (out of 4096)

#define USMIN  500 // This i`s the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2500 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// neutral point for each servomotor (estimated at 300 each, refine later)
const int NEUTRALS_PORT[5] = { -3, -16, -9, 0 , 0};
const int NEUTRALS_STARBOARD[5] = { 3, -10, -16, 0, 5};

// Design constants
#define NUM_SERVOS 5
#define SPOKE_LENGTH_MM 140
#define FIN_LENGTH_MM 240

// Safety Thresholds
#define MAX_TIME_INC 7
#define MAX_ANGLE_DELTA 2
#define MAX_AMPLITUDE 70
#define MAX_ANGLE_NEIGHBOUR 30
#define MAX_ANGLE 35 

// Coding constants
enum Wavetypes {SINWAVE, FLATWAVE, STANDINGWAVE, SINANDFLAT, STANDINGWAVESHIFTED};
enum sides {P,S,B};

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