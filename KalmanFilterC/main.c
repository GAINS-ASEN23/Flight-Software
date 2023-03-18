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
#include "CControl/Headers/Functions.h"

// GAINS includes
#include "kf.h"
#include "operations.h"

int main()
{
    /*****  VARIABLE DECLARATION & PREALLOCATION  *****/

    uint16_t n_u = 3;                       // Number of deterministic inputs (3 - accelerations x,y,z)
    uint16_t n_x = 6;                       // Number of States (x, y, z, x_dot, y_dot, z_dot)
    uint16_t n_z = 6;                       // Number of measured states (x_g, y_g, z_g, x_dot_g, y_dot_g, z_dot_g)

    // Calculate the mean motion
    float mu_moon = 4.9048695e12;                   // Gravitational parameter of the Moon [m^3 s^-2]
    float rad_moon = 1737447.78;                    // Radius of the Moon [m]
    float orbit_alt = 50000;                        // Orbit radius [m]
    float orbit_rad = orbit_alt + rad_moon;
    float n = sqrt(mu_moon/(pow(orbit_rad,3)));     // Mean motion of the Moon around the Earth [rad/s] 

    // State and Uncertainty Matricies
    float X_n_p_1_n[n_x];                   // Current Predicted State vector
    float P_n_p_1_n[n_x*n_x];               // Current Predicted Uncertainty Matrix

    float P_n_n[n_x*n_x];                   // Estimate Uncertainty Matrix
    float x_n_n[n_x];                       // Estimate State vector

    // Input Variables
    float z_n[n_x];                         // Measurement vector
    float u_n[n_u];                         // Deterministic input vector

    // Kalman Filter Matricies
    float F[n_x*n_x];                       // State Transition Matrix
    float FT[n_x*n_x];                      // Transposed State Transition Matrix

    float G[n_x*n_u];                       // Control Matrix
    float B[n_x*n_u];                       // Control Observation Matrix
    float Gamma[n_x*n_x];                   // Gamma Matrix (No other name??) 

    float H[n_z*n_x];                       // Observation Matrix
    float HT[n_z*n_x];                      // Transposed Observation Matrix

    float Q[n_x*n_x];                       // Process Noise Matrix
    float R_n[n_z*n_z];                     // Measurement Noise Covariance Matrix
    float K_n[n_x*n_z];                     // Kalman gain
    float I_SS[n_x*n_z];                    // Idenitity matrix (for K_n)

    /*****  FILL VECTORS AND MATRICIES WITH DUMMY VALUES  *****/

    // Define identity matricies
    eye(I_SS, n_x);
    eye(Q, n_x);
    eye(H, n_x);
    eye(P_n_n, n_x);
    eye(x_n_n, n_x);
    eye(R_n, n_z);

    // 6x1
    for(int j = 0; j < n_z; j++)
    {
        z_n[j] = 0; 
    }
        
    // 3x1
    for(int i = 0; i < n_u; i++)
    {
        u_n[i] = 0; 
    }

    // Zero out H
    H[0] = 0;
    H[7] = 0;
    H[14] = 0;
    H[21] = 0;
    H[28] = 0;
    H[35] = 0;

    print(H, n_x, n_x);
    printf("%x\n\n", H);

    /*********************************************/
	clock_t start, end;
	float cpu_time_used;
	start = clock();
    /*********************************************/ 

    // Update the matrices for this time step
    update_constant_matrices(n_x, n_u, F, FT, G, Gamma, B, n, 0, 0.01);

    /*********************************************/
    /*****  X_n_p_1_n = F * X_n_n + G * U_n  *****/
    predict_state(n_x, X_n_p_1_n, F, x_n_n, G, u_n);

    // print(x_n_n, n_x, n_x);
    // print(F, n_x, n_x);
    // print(G, n_x, n_z);
    // print(u_n, n_u, 1);

    /*********************************************/
    /*****  P_n_p_1_n = F * P_n_n * F^T + Q  *****/

    predict_uncertainty(n_x, P_n_p_1_n, F, FT, P_n_n, Q);

    print(P_n_n, n_x, n_x);
    print(F, n_x, n_x);
    print(P_n_p_1_n, n_x, n_x);

    /********************************************************************/
    /*****  K_n = P_n_n_m_1 * H^T * (H * P_n_n_m_1 * H^T + R_n)^-1  *****/

    printf("\n\n%x\n\n", H);
    print(H, n_x, n_x);

    // compute_kalman_gain(n_x, n_z, K_n, P_n_p_1_n, H, R_n);
    // print(K_n, n_x, n_z);
    // print(P_n_n, n_x, n_x);
    // print(P_n_n, n_x, n_x);
    
    /*************************************************************************************/
    /*****  P_n_n = (I - K_n * H) * P_n_n_m_1 * (I - K_n * H)^T + K_n * R_n * K_n^T  *****/

    // estimate_uncertainty(n_x, n_u, P_n_n, K_n, P_n_p_1_n, H, R_n, I_SS);

    // print(P_n_n, n_x, n_x);

    /*********************************************/
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %0.18f\n", cpu_time_used);
    /*********************************************/

}