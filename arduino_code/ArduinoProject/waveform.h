//-------------------------------------------------------------------------------------------------------------------------------//
// Filename:      waveform.h
// Date Created:  28/06/2024
// Author(s):     Matt Beahan, Peppe Grasso
// Description:   Header file for the waveform module. All functions determining the shape of the waveform are within this module
//-------------------------------------------------------------------------------------------------------------------------------//

#ifndef WAVEFORM_H
#define WAVEFORM_H

namespace waveform 
{
    float calc_angle_sinwave(float amplitude, float wavelength, unsigned long time, int servonum);
    float calc_angle_flatwave(float amplitude, float wavelength, unsigned long time);
    float calc_angle_standingwave(float amplitude, float wavelength, unsigned long time, int servonum);
    float calc_angle_sinandflat(float amplitude, float wavelength, unsigned long time, int servonum);
}

#endif //WAVEFORM_H
