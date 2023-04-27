/*
============================================================================
Name        : main.c
Author      : Bennett Grow, Kaylie Rick, Jason Popich
Version     : 0.1
Copyright   : 
Description : The main kalman filter file
============================================================================
*/

// Library includes
extern "C" {
    #include "CControl/Headers/Functions.h"         // Matrix Math Library
}

// GAINS includes
#include "main.hpp"

int main()
{
    /*****  VARIABLE DECLARATION & PREALLOCATION  *****/

    /*    Set System Variables    */

    float t1 = 0;                           // Sample Time begin
    float t2 = 0.01;                        // Sample Time end

    uint16_t n_u = 3;                       // Number of deterministic inputs (3 - accelerations x,y,z)
    uint16_t n_x = 6;                       // Number of States (x, y, z, x_dot, y_dot, z_dot)
    uint16_t n_z = 6;                       // Number of measured states (x_g, y_g, z_g, x_dot_g, y_dot_g, z_dot_g)

    /*    Preallocate Matricies and Vectors    */

    float *P_n_n = (float*)malloc((n_x * n_x) * sizeof(float));                     // Estimate Uncertainty Matrix
    float *x_n_n = (float*)malloc((n_x * 1) * sizeof(float));                       // Estimate State vector

    // Input Variables
    float* z_n = (float*)malloc((n_z * 1) * sizeof(float));                                // Measurement vector
    float* u_n = (float*)malloc((n_u * 1) * sizeof(float));                                // Deterministic input vector

    /********************************************************/
    /*       FILL IN THE INITIAL VECTORS AND MATRICIES      */
    /********************************************************/

    /*    Set gravitational parameters    */

    float mu_moon = 4.9048695e12;           // Gravitational parameter of the Moon [m^3 s^-2]
    float rad_moon = 1737447.78;            // Radius of the Moon [m]
    float orbit_alt = 50000;                // Orbit radius [m]

    // Set the mean motion for CW equations
    float n = calculate_mean_motion(mu_moon, rad_moon, orbit_alt);

    /*    Set the initial orbit initial conditions    */

    float alpha = 0;                    // The Phase Angle Alpha [deg]
    float beta = 0;                     // The Phase Angle Beta [deg]
    float deviation = 50000;            // The deviation in position from the chief satellite [m]

    set_cw_ics(x_n_n, alpha, beta, deviation, n);

    /*    Set the initial uncertainty matrix    */

    float sigma_position = 1000;        // The error in the position x,y,z [m]
    float sigma_velocity = 0.1;         // The error in the velocity x,y,z [m/s]

    // Make the P_n_n matrix an identity matrix
    eye(P_n_n, n_x);

    // Populate sigmas
    float sigma_p_n_n[6] = {sigma_position, sigma_position, sigma_position, sigma_velocity, sigma_velocity, sigma_velocity};
    set_p_ic(n_x, P_n_n, sigma_p_n_n);

    // Create the KF object
    KalmanFilter KF(n_x, n_u, n_z, x_n_n, P_n_n, 1);

    /*  Set the measurement covariance matrix   */
    float sigma_position_ground = 1000;
    float sigma_velocity_ground = 0.1;

    // Populate the sigmas
    float sigma_r_n[6] = {sigma_position_ground, sigma_position_ground, sigma_position_ground, sigma_velocity_ground, sigma_velocity_ground, sigma_velocity_ground};
    KF.set_r_n(sigma_r_n);

    /*  Set the initial input covariance */
    float sigma_accel_x = 1;
    float sigma_accel_y = 1;
    float sigma_accel_z = 1;

    // Populate sigmas
    float sigma_q_a[3] = {sigma_accel_x, sigma_accel_y, sigma_accel_z};
    KF.set_q_a(sigma_q_a);

    /*********************************************/
	clock_t start, end;
	float cpu_time_used;
	start = clock();
    /*********************************************/ 

    // LOOP
    while(true)
    {
            // Set the measurement vector, if ground contact is non-zero
            z_n[0] = 0;
            z_n[1] = 0;
            z_n[2] = 0;
            z_n[3] = 0;
            z_n[4] = 0;
            z_n[5] = 0;
            KF.set_mea_input_vector(z_n);

            // Set the H matrix as if we don't have a ground contact, but if contact set true
            KF.set_h(false);

            // Set the deterministic input vector, if thrusting is non-zero
            u_n[0] = 0;
            u_n[1] = 0;
            u_n[2] = 0;
            u_n[3] = 0;
            u_n[4] = 0;
            u_n[5] = 0;
            KF.set_det_input_vector(u_n);

            // Run the KF
            KF.KF_run(t1, t2, n);
            //print(x_n_n, 6, 1);
            std::cout << x_n_n[0] << " " << x_n_n[1] << " " << x_n_n[2] << " " << x_n_n[3] << " " << x_n_n[4] << " " << x_n_n[5] << " " << std::endl;

            // Delay depending on requirements
            
            // Update t1 and t2
    }

    /*********************************************/
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %0.18f\n", cpu_time_used);
    /*********************************************/
}