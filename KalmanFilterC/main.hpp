/*
============================================================================
Name        : kf.h
Author      : Jason Popich, Bennett Grow, Kaylie Rick
Version     : 0.1
Copyright   : 
Description : This file is the main include for the main source file
============================================================================
*/

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdlib.h>						    // Standard library
#include <stdint.h>						    // For uint8_t, uint16_t and uint16_t
extern "C" {
    #include "CControl/Headers/Functions.h"         // Matrix Math Library
}

// GAINS includes
#include "operations.hpp"
#include "kf.hpp"

float calculate_mean_motion(float mu, float rad_attractor, float orbit_alt)
{
    // Calculate the mean motion
    return sqrt(mu/(pow(orbit_alt + rad_attractor,3)));
}

void set_cw_ics(float x_n_n[], float alpha, float beta, float deviation, float n)
{
    float alpha_rad = (alpha * 3.14) / 180;                 // Phase Angle Alpha of Circular Orbit [rad]
    float beta_rad = (beta * 3.14) / 180;                   // Phase Angle Beta of Circular Orbit [rad]
    float B_0 = 2*deviation;                                // Out of plane sinusoidal amplitude [m]

    float x_0 = deviation*cos(alpha_rad);                   // Initial X Position [m] Hill Frame
    float y_0 = -2*deviation*sin(alpha_rad);                // Initial Y Position [m] Hill Frame
    float z_0 = 0;                                          // Initial Z Position [m] Hill Frame

    float x_dot_0 = -x_0*n*sin(alpha_rad);                  // Initial Orbital X velocity [m/s] Hill Frame
    float y_dot_0 =  -2*n*x_0*cos(alpha_rad);               // Initial Orbital Y velocity [m/s] Hill Frame
    float z_dot_0 =  0;                                     // Initial Orbital Z velocity [m/s] Hill Frame

    // Set the initial state vector
    x_n_n[0] = x_0;
    x_n_n[1] = y_0;
    x_n_n[2] = z_0;

    x_n_n[3] = x_dot_0;
    x_n_n[4] = y_dot_0;
    x_n_n[5] = z_dot_0;
}

void set_p_ic(uint8_t state_size, float P_n_n[], float sigma[])
{
    // Populate the diagonals based on the sigma vector given
    for(int i = 0; i < state_size; i++)
    {
        P_n_n[(state_size + 1) * i] = pow(sigma[i], 2);
    }
}

#endif