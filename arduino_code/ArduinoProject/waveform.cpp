//----------------------------------------------------------------------------------------------------------------------------------------//
// Filename:      waveform.cpp
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Implementation file for the waveform module. All functions determining the shape of the waveform are within this module
//----------------------------------------------------------------------------------------------------------------------------------------//

// module includes
#include "waveform.h"
#include "robot.h"

// library includes
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// calc_angle_sinwave function - takes sinusoidal parameters and servonum, and calculates the angle required for a given servomotor to achieve waveform
float waveform::calc_angle_sinwave(float amplitude, float wavelength, float time, int servonum)
{  
    // sample the waveform to get height of wave at a calculated point based on servonum 
    float height = amplitude * sin( 2*M_PI/wavelength * (time + FIN_LENGTH_MM*servonum/(NUM_SERVOS-1)));

    // calculate angle required of servomotor to achieve given height (basic trig)
    float angle = 360/(2*M_PI)*asin( (float) height / SPOKE_LENGTH_MM);

    return angle;
}

// calc_angle_flatwave function - takes sinusoidal parameters and servonum, and calculates the angle required for a given servomotor to achieve waveform
float waveform::calc_angle_flatwave(float amplitude, float wavelength, float time)
{  
    // sample the waveform to get height of wave at a calculated point based on servonum 
    float height = amplitude * sin( 2*M_PI/wavelength * time);

    // calculate angle required of servomotor to achieve given height (basic trig)
    float angle = 360/(2*M_PI)*asin( (float) height / SPOKE_LENGTH_MM);

    return angle;
}

// calc_angle_standingwave2 function - takes sinusoidal parameters and servonum, and calculates the angle required for a given servomotor to achieve waveform
float waveform::calc_angle_standingwave(float amplitude, float wavelength, float time, int servonum)
{  

    // sample the waveform to get height of wave at a calculated point based on servonum 
    float height1 = amplitude * sin( (2*M_PI/wavelength) * (time + FIN_LENGTH_MM*servonum/(NUM_SERVOS-1)));
    float height2 = amplitude * sin( (2*M_PI/wavelength) * (-time + FIN_LENGTH_MM*servonum/(NUM_SERVOS-1)));

    float height = (height1 + height2)/2;

    // calculate angle required of servomotor to achieve given height (basic trig)
    float angle = 360/(2*M_PI)*asin( (float) height / SPOKE_LENGTH_MM);

    return angle;
}

// calc_angle_sinandflat function - takes sinusoidal parameters and servonum, and calculates the angle required for a given servomotor to achieve waveform
// TEMPORARY FUNCTION for testing compound functions
float waveform::calc_angle_sinandflat(float amplitude, float wavelength, float time, int servonum)
{
    // sample the waveform to get height of wave at a calculated point based on servonum 
    float sinheight = amplitude * sin( 2*M_PI/wavelength * (time + FIN_LENGTH_MM*servonum/(NUM_SERVOS-1)));
    
    // sample the waveform to get height of wave at a calculated point based on servonum 
    float flatheight = 140 * sin( 2*M_PI/960 * time);

    float height = (sinheight + flatheight)/2;

    // calculate angle required of servomotor to achieve given height (basic trig)
    float angle = 360/(2*M_PI)*asin( (float) height / SPOKE_LENGTH_MM);

    return angle;
}
