/*
============================================================================
Name        : operations.h
Author      : Bennett Grow, Kaylie Rick, Jason Popich
Version     : 0.1
Copyright   : 
Description : This file contains some mathematical operation function declarations 
               that could be needed in the main KF source code
============================================================================
*/

#ifndef _OPERATIONS_H_
#define _OPERATIONS_H_

#include <stdlib.h>						// Standard library
#include <stdint.h>						// For uint8_t, uint16_t and uint16_t
#include <CControl.h>

// Matrix Math
void copy(float A[], float B[], uint16_t len);
void add(float A[], float B[], float C[], uint16_t len);
void sub(float A[], float B[], float C[], uint16_t len);
void eye(float A[], uint16_t N);

// KF Helpers
float calculate_mean_motion(float mu, float rad_attractor, float orbit_alt);
void set_cw_ics(float x_n_n[], float alpha, float beta, float deviation, float n);
void set_p_ic(uint8_t state_size, float P_n_n[], float sigma[]);

// Sensor Helpers
float accel(int AO_P, int AO_N);
float temp(float V_T);

#endif