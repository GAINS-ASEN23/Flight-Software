/*
============================================================================
Name        : operations.h
Author      : Bennett Grow, Kaylie Rick, Jason Popich
Version     : 0.1
Copyright   : 
Description : This file contains some mathematical operation function definitions 
               that could be needed in the main KF source code
============================================================================
*/

#include <operations.h>

/*
 * B = A
 * len = length of both arrays/'matricies'
 */
void copy(float A[], float B[], uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
        B[i] = A[i];
    }
}

/*
 * C = A + B
 * len = length of both arrays/'matricies'
 */
void add(float A[], float B[], float C[], uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
        C[i] = A[i] + B[i];
    }
}

/*
 * C = A - B
 * len = length of both arrays/'matricies'
 */
void sub(float A[], float B[], float C[], uint16_t len) 
{
	for (uint16_t i = 0; i < len; i++) {
        C[i] = A[i] - B[i];
    }
}

/*
 * A = Identity Matrix
 * N = side length
 */
void eye(float A[], uint16_t N)
{
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            if(i == j)
            {
                A[i*N + j] = 1;
            }
            else
            {
                A[i*N + j] = 0;
            }
        }
    }
}

// Calculate the mean motion
float calculate_mean_motion(float mu, float rad_attractor, float orbit_alt)
{
    return sqrt(mu/(pow(orbit_alt + rad_attractor,3)));
}

// Set Clohessy-Wiltshire initial conditions
void set_cw_ics(float x_n_n[], float alpha, float beta, float deviation, float n)
{
    float alpha_rad = (alpha * 3.14) / 180;                 // Phase Angle Alpha of Circular Orbit [rad]
    // UNUSED:
    // float beta_rad = (beta * 3.14) / 180;                   // Phase Angle Beta of Circular Orbit [rad]
    // float B_0 = 2*deviation;                                // Out of plane sinusoidal amplitude [m]

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

// Set covariance initial conditions
void set_p_ic(uint8_t state_size, float P_n_n[], float sigma[])
{
    // Populate the diagonals based on the sigma vector given
    for(int i = 0; i < state_size; i++)
    {
        P_n_n[(state_size + 1) * i] = pow(sigma[i], 2);
    }
}

// Convert accelerometer differential pin inputs to g's (no corrections)
float accel(int AP, int AN){
    double bins = 4096;         // Bins in analog output
    double range = 2.0;         // +/- 2 g output
    double diff = AP - AN;  // Differential bin output

    return (diff * range) / bins;
}



// Convert temperature pin inputs to degrees C
float temp(float VT){
    return ((VT*90.0)/4096.0) + 35.0;  // Currently doesn't work
}