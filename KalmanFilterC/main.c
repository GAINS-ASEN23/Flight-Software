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
#include "main.h"

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

    // State and Uncertainty Matricies
    float *x_n_p_1_n = (float*)malloc((n_x * 1) * sizeof(float));                   // Current Predicted State vector
    float *P_n_p_1_n = (float*)malloc((n_x * n_x) * sizeof(float));                 // Current Predicted Uncertainty Matrix

    float *P_n_n = (float*)malloc((n_x * n_x) * sizeof(float));                     // Estimate Uncertainty Matrix
    float *x_n_n = (float*)malloc((n_x * 1) * sizeof(float));                       // Estimate State vector

    // Input Variables
    float *z_n = (float*)malloc((n_z * 1) * sizeof(float));                         // Measurement vector
    float *u_n = (float*)malloc((n_u * 1) * sizeof(float));                         // Deterministic input vector

    // Kalman Filter Matricies
    float *F = (float*)malloc((n_x * n_x) * sizeof(float));                         // State Transition Matrix
    float *FT = (float*)malloc((n_x * n_x) * sizeof(float));                        // Transposed State Transition Matrix

    float *G = (float*)malloc((n_x * n_u) * sizeof(float));                         // Control Matrix
    float *B = (float*)malloc((n_x * n_u) * sizeof(float));                         // Control Observation Matrix
    float *BT = (float*)malloc((n_u * n_x) * sizeof(float));                        // Transposed Control Observation Matrix
    float *Gamma = (float*)malloc((n_x * n_x) * sizeof(float));                     // Gamma Matrix (No other name??) 

    float *H = (float*)malloc((n_z * n_x) * sizeof(float));                         // Observation Matrix
    float *HT = (float*)malloc((n_z * n_x) * sizeof(float));                        // Transposed Observation Matrix
    eye(H, n_z);                            // Define the H matrix as the identity for this application

    float *Q = (float*)malloc((n_x * n_x) * sizeof(float));                         // Process Noise Matrix
    float *Q_a = (float*)malloc((n_u * n_u) * sizeof(float));                       // Input Noise Matrix
    float *R_n = (float*)malloc((n_z * n_z) * sizeof(float));                       // Measurement Noise Covariance Matrix
    float *K_n = (float*)malloc((n_x * n_z) * sizeof(float));                       // Kalman gain

    float *I_SS = (float*)malloc((n_x * n_z) * sizeof(float));                      // Idenitity matrix (for K_n)
    eye(I_SS, n_x);                         // Define the Identity Matrix

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

    /*  Set the measurement covariance matrix   */
    float sigma_position_ground = 1000;
    float sigma_velocity_ground = 0.1;

    // Make the R_n matrix an identity matrix
    eye(R_n, n_z);

    // Populate the sigmas
    float sigma_r_n[6] = {sigma_position_ground, sigma_position_ground, sigma_position_ground, sigma_velocity_ground, sigma_velocity_ground, sigma_velocity_ground};
    set_r_n(n_z, R_n, sigma_r_n);

    /*  Set the initial measurement vector as zero */
    for(int j = 0; j < n_z; j++)
    {
        z_n[j] = 0; 
    }
        
    /*  Set the initial input vector as zero */
    for(int i = 0; i < n_u; i++)
    {
        u_n[i] = 0; 
    }

    /*  Set the H matrix as if we don't have a ground contact */
    set_h(H, false);

    /*  Set the B matricies */
    set_b(n_x, n_u, B, BT);

    /*  Set the initial input covariance */
    float sigma_accel_x = 1;
    float sigma_accel_y = 1;
    float sigma_accel_z = 1;

    // Make the input covariance matrix an identity matrix
    eye(Q_a, n_u);

    // Populate sigmas
    float sigma_q_a[3] = {sigma_accel_x, sigma_accel_y, sigma_accel_z};
    set_q_a(n_u, Q_a, sigma_q_a);

    /*********************************************/
	clock_t start, end;
	float cpu_time_used;
	start = clock();
    /*********************************************/ 

    // Update the State Transition Matrix
    compute_state_transition_matrix(F, n, t1, t2);

    // Calculate the Tranpose of F
    copy(F, FT, n_x*n_x);
    tran(FT, n_x, n_x);

    // Update the Gamma Matrix
    compute_gamma_matrix(Gamma, n, t1, t2);

    // Update the G Matrix
    mul(Gamma, B, G, n_x, n_x, n_z);

    // Update the process noise covariance matrix
    compute_process_noise_covariance_matrix(n_x, n_u, Q, F, FT, B, BT, Q_a);

    // printf("\n\n----------------------------------------\n");
    // printf("CONSTANT MATRICIES\n");
    // printf("----------------------------------------\n");

    // printf("G: \n");
    // print(F, n_x, n_x);

    // printf("FT: \n");
    // print(FT, n_x, n_x);

    // printf("Gamma: \n");
    // print(Gamma, n_x, n_x);

    // printf("B: \n");
    // print(B, n_x, n_u);

    // printf("BT: \n");
    // print(BT, n_u, n_x);

    // printf("G: \n");
    // print(G, n_x, n_u);

    // printf("Q_a: \n");
    // print(Q_a, n_u, n_u);

    // printf("Q: \n");
    // print(Q, n_x, n_x);

    /*********************************************/
    /*****  X_n_p_1_n = F * X_n_n + G * U_n  *****/
    predict_state(n_x, x_n_p_1_n, F, x_n_n, G, u_n);

    // printf("\n\n----------------------------------------\n");
    // printf("PREDICT STATE\n");
    // printf("----------------------------------------\n");

    // printf("x_n_n: \n");
    // print(x_n_n, n_x, 1);

    // printf("x_n_p_1_n: \n");
    // print(x_n_p_1_n, n_x, 1);

    // printf("F: \n");
    // print(F, n_x, n_x);

    // printf("G: \n");
    // print(G, n_x, n_u);

    // printf("u_n: \n");
    // print(u_n, n_u, 1);

    /*********************************************/
    /*****  P_n_p_1_n = F * P_n_n * F^T + Q  *****/

    predict_uncertainty(n_x, P_n_p_1_n, F, FT, P_n_n, Q);

    // printf("\n\n----------------------------------------\n");
    // printf("PREDICT UNCERTAINTY\n");
    // printf("----------------------------------------\n");

    // printf("P_n_n: \n");
    // print(P_n_n, n_x, n_x);

    // printf("P_n_p_1_n: \n");
    // print(P_n_p_1_n, n_x, n_x);

    // printf("F: \n");
    // print(F, n_x, n_x);

    /********************************************************************/
    /*****  K_n = P_n_n_m_1 * H^T * (H * P_n_n_m_1 * H^T + R_n)^-1  *****/

    compute_kalman_gain(n_x, n_z, K_n, P_n_p_1_n, H, R_n);

    // printf("\n\n----------------------------------------\n");
    // printf("COMPUTE KALMAN GAIN\n");
    // printf("----------------------------------------\n");

    // printf("K_n: \n");
    // print(K_n, n_x, n_z);

    // printf("P_n_p_1_n: \n");
    // print(P_n_n, n_x, n_x);

    // printf("H: \n");
    // print(H, n_z, n_x);

    // printf("R_n: \n");
    // print(R_n, n_x, n_x);

    /*************************************************************/
    /*****  X_n_n = x_n_p_1_n + K_n * (Z_n - H * x_n_p_1_n)  *****/

    estimate_state(n_x, n_z, x_n_n, x_n_p_1_n, K_n, z_n, H);

    printf("\n\n----------------------------------------\n");
    printf("ESTIMATE STATE\n");
    printf("----------------------------------------\n");

    printf("x_n_n: \n");
    print(x_n_n, n_x, 1);

    // printf("x_n_p_1_n: \n");
    // print(x_n_p_1_n, n_x, 1);

    // printf("K_n: \n");
    // print(K_n, n_x, n_z);

    // printf("z_n: \n");
    // print(z_n, n_z, 1);

    // printf("H: \n");
    // print(H, n_z, n_x);
    
    /*************************************************************************************/
    /*****  P_n_n = (I - K_n * H) * P_n_n_m_1 * (I - K_n * H)^T + K_n * R_n * K_n^T  *****/

    estimate_uncertainty(n_x, n_u, P_n_n, K_n, P_n_p_1_n, H, R_n, I_SS);

    printf("\n\n----------------------------------------\n");
    printf("ESTIMATE UNCERTAINTY\n");
    printf("----------------------------------------\n");

    printf("P_n_n: \n");
    print(P_n_n, n_x, n_x);

    // printf("K_n: \n");
    // print(K_n, n_x, n_z);

    // printf("P_n_p_1_n: \n");
    // print(P_n_n, n_x, n_x);

    // printf("H: \n");
    // print(H, n_z, n_x);

    // printf("R_n: \n");
    // print(R_n, n_x, n_x);

    // printf("I_SS: \n");
    // print(I_SS, n_x, n_x);

    /*********************************************/
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %0.18f\n", cpu_time_used);
    /*********************************************/


    /*********************************************/
    /*            CLEAN THE MEMORY UP            */
    /*********************************************/

    free(x_n_p_1_n);
    free(P_n_p_1_n);
    free(P_n_n);
    free(x_n_n);
    free(z_n);
    free(u_n);
    free(F);
    free(FT);
    free(G);
    free(B);
    free(BT);
    free(Gamma);
    free(H);
    free(HT);
    free(Q);
    free(Q_a);
    free(R_n);
    free(K_n);
}