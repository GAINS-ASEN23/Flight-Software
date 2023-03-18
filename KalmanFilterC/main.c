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

    float t1 = 0;                           // Sample Time begin
    float t2 = 0.01;                        // Sample Time end

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
    float x_n_p_1_n[n_x];                   // Current Predicted State vector
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
    float BT[n_u*n_x];                      // Transposed Control Observation Matrix
    float Gamma[n_x*n_x];                   // Gamma Matrix (No other name??) 

    float H[n_z*n_x];                       // Observation Matrix
    float HT[n_z*n_x];                      // Transposed Observation Matrix

    float Q[n_x*n_x];                       // Process Noise Matrix
    float Q_a[n_u*n_u];                     // Input Noise Matrix
    float R_n[n_z*n_z];                     // Measurement Noise Covariance Matrix
    float K_n[n_x*n_z];                     // Kalman gain
    float I_SS[n_x*n_z];                    // Idenitity matrix (for K_n)

    /*****  FILL VECTORS AND MATRICIES WITH DUMMY VALUES  *****/

    // Define identity matricies
    eye(I_SS, n_x);
    eye(H, n_x);
    eye(P_n_n, n_x);
    eye(R_n, n_z);

    // 6x1
    for(int j = 0; j < n_z; j++)
    {
        z_n[j] = 0; 
        x_n_n[j] = 1;
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

    // Fill Out B
    B[0] = 0;
    B[1] = 0;
    B[2] = 0;
    B[3] = 0;
    B[4] = 0;
    B[5] = 0;
    B[6] = 0;
    B[7] = 0;
    B[8] = 0;
    B[9] = 1;
    B[10] = 0;
    B[11] = 0;
    B[12] = 0;
    B[13] = 1;
    B[14] = 0;
    B[15] = 0;
    B[16] = 0;
    B[17] = 1;

    // Fill Out Q_a
    Q_a[0] = pow(1,2);
    Q_a[1] = 0;
    Q_a[2] = 0;
    Q_a[3] = 0;
    Q_a[4] = pow(1,2);
    Q_a[5] = 0;
    Q_a[6] = 0;
    Q_a[7] = 0;
    Q_a[8] = pow(1,2);


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

    // Calculate the Tranpose of B
    copy(B, BT, n_x*n_u);
    tran(BT, n_u, n_x);

    // Update the Gamma Matrix
    compute_gamma_matrix(Gamma, n, t1, t2);

    // Update the G Matrix
    mul(Gamma, B, G, n_x, n_x, n_z);

    // Update the process noise covariance matrix
    compute_process_noise_covariance_matrix(n_x, n_u, Q, F, FT, B, BT, Q_a);

    printf("\n\n----------------------------------------\n");
    printf("CONSTANT MATRICIES\n");
    printf("----------------------------------------\n");

    printf("G: \n");
    print(F, n_x, n_x);

    printf("FT: \n");
    print(FT, n_x, n_x);

    printf("Gamma: \n");
    print(Gamma, n_x, n_x);

    printf("B: \n");
    print(B, n_x, n_u);

    printf("G: \n");
    print(G, n_x, n_u);

    printf("Q_a: \n");
    print(Q_a, n_u, n_u);

    printf("Q: \n");
    print(Q, n_x, n_x);

    /*********************************************/
    /*****  X_n_p_1_n = F * X_n_n + G * U_n  *****/
    predict_state(n_x, x_n_p_1_n, F, x_n_n, G, u_n);

    printf("\n\n----------------------------------------\n");
    printf("PREDICT STATE\n");
    printf("----------------------------------------\n");

    printf("x_n_n: \n");
    print(x_n_n, n_x, 1);

    printf("x_n_p_1_n: \n");
    print(x_n_p_1_n, n_x, 1);

    printf("F: \n");
    print(F, n_x, n_x);

    printf("G: \n");
    print(G, n_x, n_u);

    printf("u_n: \n");
    print(u_n, n_u, 1);

    /*********************************************/
    /*****  P_n_p_1_n = F * P_n_n * F^T + Q  *****/

    predict_uncertainty(n_x, P_n_p_1_n, F, FT, P_n_n, Q);

    printf("\n\n----------------------------------------\n");
    printf("PREDICT UNCERTAINTY\n");
    printf("----------------------------------------\n");

    printf("P_n_n: \n");
    print(P_n_n, n_x, n_x);

    printf("P_n_p_1_n: \n");
    print(P_n_p_1_n, n_x, n_x);

    printf("F: \n");
    print(F, n_x, n_x);

    printf("Q: \n");
    print(Q, n_x, n_x);

    /********************************************************************/
    /*****  K_n = P_n_n_m_1 * H^T * (H * P_n_n_m_1 * H^T + R_n)^-1  *****/

    compute_kalman_gain(n_x, n_z, K_n, P_n_p_1_n, H, R_n);

    printf("\n\n----------------------------------------\n");
    printf("COMPUTE KALMAN GAIN\n");
    printf("----------------------------------------\n");

    printf("K_n: \n");
    print(K_n, n_x, n_z);

    printf("P_n_p_1_n: \n");
    print(P_n_n, n_x, n_x);

    printf("H: \n");
    print(H, n_z, n_x);

    printf("R_n: \n");
    print(R_n, n_x, n_x);

    /*************************************************************/
    /*****  X_n_n = x_n_p_1_n + K_n * (Z_n - H * x_n_p_1_n)  *****/

    estimate_state(n_x, n_z, x_n_n, x_n_p_1_n, K_n, z_n, H);

    printf("\n\n----------------------------------------\n");
    printf("ESTIMATE STATE\n");
    printf("----------------------------------------\n");

    printf("x_n_n: \n");
    print(x_n_n, n_x, 1);

    printf("x_n_p_1_n: \n");
    print(x_n_p_1_n, n_x, 1);

    printf("K_n: \n");
    print(K_n, n_x, n_z);

    printf("z_n: \n");
    print(z_n, n_z, 1);

    printf("H: \n");
    print(H, n_z, n_x);
    
    /*************************************************************************************/
    /*****  P_n_n = (I - K_n * H) * P_n_n_m_1 * (I - K_n * H)^T + K_n * R_n * K_n^T  *****/

    estimate_uncertainty(n_x, n_u, P_n_n, K_n, P_n_p_1_n, H, R_n, I_SS);

    printf("\n\n----------------------------------------\n");
    printf("ESTIMATE UNCERTAINTY\n");
    printf("----------------------------------------\n");

    printf("P_n_n: \n");
    print(P_n_n, n_x, n_x);

    printf("K_n: \n");
    print(K_n, n_x, n_z);

    printf("P_n_p_1_n: \n");
    print(P_n_n, n_x, n_x);

    printf("H: \n");
    print(H, n_z, n_x);

    printf("R_n: \n");
    print(R_n, n_x, n_x);

    printf("I_SS: \n");
    print(I_SS, n_x, n_x);

    /*********************************************/
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %0.18f\n", cpu_time_used);
    /*********************************************/

}