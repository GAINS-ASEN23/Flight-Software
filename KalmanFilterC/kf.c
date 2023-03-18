/*
============================================================================
Name        : kf.c
Author      : Jason Popich, Bennett Grow, Kaylie Rick
Version     : 0.1
Copyright   : 
Description : This file contains the Kalman Filter (KF) function definitions
============================================================================
*/

#include "kf.h"

// Pre-Declare static voids
static void compute_state_transition_matrix(float F[], float n, float t1, float t2);
static void compute_gamma_matrix(float Gamma[], float n, float t1, float t2);

/*********************************************/
/*      MAIN KALMAN FILTER FUNCTIONS         */
/*********************************************/

void predict_state(uint8_t state_size, float x_n_p_1_n[], float F[], float x_n_n[], float G[], float u_n[])
{
    /*          Equation               */
    /* x_n_p_1_n = F * x_n_n + G * u_n */

    // Declare temp vars
    float FX_n_n[state_size];
    float GU_n[state_size];

    // F*x_n_n
    mul(F, x_n_n, FX_n_n, state_size, state_size, 1);

    // G*u_n
    mul(G, u_n, GU_n, state_size, state_size, 1);

    //  x_n_p_1_n
    add(FX_n_n, GU_n, x_n_p_1_n, state_size*state_size);
}

void predict_uncertainty(uint8_t state_size, float P_n_p_1_n[], float F[], float FT[], float P_n_n[], float Q[])
{
    /*          Equation               */
    /* P_n_p_1_n = F * P_n_n * F^T + Q */

    // Declare temp vars
    float FP_n_n[state_size*state_size];
    float FP_n_nFT[state_size*state_size];

    // FP_n_n = F * P_n_n
    mul(F, P_n_n, FP_n_n, state_size, state_size, state_size);

    // FP_n_nFT = FP_n_n * FT
    mul(FP_n_n, FT, FP_n_nFT, state_size, state_size, state_size);

    // P_n_p_1_n = F*P_n_n*FT + Q
    add(FP_n_nFT, Q, P_n_p_1_n, state_size*state_size);
}

void compute_kalman_gain(uint8_t state_size, uint8_t measurement_num, float K_n[], float P_n_n_m_1[], float H[], float R_n[])
{
    /*                       Equation                           */
    /*  K_n = P_n_n_m_1 * H^T * (H * P_n_n_m_1 * H^T + R_n)^-1  */

    // Declare temp vars
    float P_n_n_m_1HT[state_size*state_size];
    float HP_n_n_m_1[measurement_num*state_size];
    float HP_n_n_m_1HT[measurement_num*measurement_num];
    float HP_n_n_m_1HTR_n[measurement_num*measurement_num];
    float HT[measurement_num*state_size];

    // HT = H^T
    // copy(H, HT, state_size*measurement_num);
    // tran(HT, state_size, measurement_num);

    // // P_n_n_m_1HT = P_n_n_m_1 * HT
    // mul(P_n_n_m_1, HT, P_n_n_m_1HT, state_size, state_size, measurement_num);

    // // HP_n_n_m_1 = H * P_n_n_m_1
    // mul(H, P_n_n_m_1, HP_n_n_m_1, measurement_num, state_size, state_size);

    // // HP_n_n_m_1HT = HP_n_n_m_1 * HT
    // mul(HP_n_n_m_1, HT, HP_n_n_m_1HT, measurement_num, state_size, measurement_num);

    // // HP_n_n_m_1HTR_n = HP_n_n_m_1HT + R_n
    // add(HP_n_n_m_1HT, R_n, HP_n_n_m_1HTR_n, measurement_num*measurement_num);

    // // Invert HP_n_n_m_1HTR_n
    // if(inv(HP_n_n_m_1HTR_n, measurement_num) == 0)
    // {
    //     printf("Error Inverting HP_n_n_m_1HTR_n in KF Eqn 3 \n");
    // }

    // // K_n = P_n_n_m_1HT * HP_n_n_m_1HTR_n
    // mul(P_n_n_m_1HT, HP_n_n_m_1HTR_n, K_n, state_size, measurement_num, measurement_num);
}

void estimate_state(uint8_t state_size, uint8_t measurement_num, float x_n_n[], float x_n_p_1_n[], float K_n[], float z_n[], float H[])
{
    /*                    Equation                       */
    /*  X_n_n = x_n_p_1_n + K_n * (Z_n - H * x_n_p_1_n)  */

    float HX_n_n_m_1[measurement_num];
    float Z_nHX_n_n_m_1[measurement_num];
    float K_nZ_nHX_n_n_m_1[state_size];

    // HX_n_n_m_1 = H * x_n_p_1_n
    mul(H, x_n_p_1_n, HX_n_n_m_1, measurement_num, state_size, 1);

    // Z_nHX_n_n_m_1 = Z_n - HX_n_n_m_1
    sub(z_n, HX_n_n_m_1, Z_nHX_n_n_m_1, measurement_num);

    // K_nZ_nHX_n_n_m_1 = K_n * Z_nHX_n_n_m_1
    mul(K_n, Z_nHX_n_n_m_1, K_nZ_nHX_n_n_m_1, state_size, measurement_num, 1);

    // X_n_n = x_n_p_1_n + K_nZ_nHX_n_n_m_1
    add(x_n_p_1_n, K_nZ_nHX_n_n_m_1, x_n_n, state_size);
}

void estimate_uncertainty(uint8_t state_size, uint8_t measurement_num, float P_n_n[], float K_n[], float P_n_n_m_1[], float H[], float R_n[], float I_SS[])
{
    /*                                Equation                                   */
    /*  P_n_n = (I - K_n * H) * P_n_n_m_1 * (I - K_n * H)^T + K_n * R_n * K_n^T  */

    float K_nH[state_size*state_size];
    float IK_nH[state_size*state_size];
    float IK_nHT[state_size*state_size];
    float IK_nHP_n_n_m_1[state_size*state_size];
    float IK_nHP_n_n_m_1IK_nHT[state_size*state_size];
    float K_nT[measurement_num*state_size];
    float K_nR_n[state_size*measurement_num];
    float K_nR_nK_nT[state_size*measurement_num];

    // K_nH = K_n * H
    mul(K_n, H, K_nH, state_size, measurement_num, state_size);

    // IK_nH = I - K_nH
    sub(I_SS, K_nH, IK_nH, state_size*state_size);

    // IK_nHT = (I - K_nH)^T
    copy(IK_nH, IK_nHT, state_size*state_size);
    tran(IK_nHT, state_size, state_size);

    // IK_nHP_n_n_m_1 = IK_nH * P_n_n_m_1
    mul(IK_nH, P_n_n_m_1, IK_nHP_n_n_m_1, state_size, state_size, state_size);

    // IK_nHP_n_n_m_1IK_nHT = IK_nH * P_n_n_m_1
    mul(IK_nHP_n_n_m_1, IK_nHT, IK_nHP_n_n_m_1IK_nHT, state_size, state_size, state_size);

    // K_nT = (K_n)^T
    copy(K_n, K_nT, state_size*measurement_num);
    tran(K_nT, state_size, measurement_num);

    // K_nR_n = K_n * R_n
    mul(K_n, R_n, K_nR_n, state_size, measurement_num, measurement_num);

    // K_nR_nK_nT = K_nR_n * K_nT
    mul(K_nR_n, K_nT, K_nR_nK_nT, state_size, measurement_num, state_size);

    // P_n_n = IK_nHP_n_n_m_1IK_nHT + K_nR_nK_nT
    add(IK_nHP_n_n_m_1IK_nHT, K_nR_nK_nT, P_n_n, state_size*state_size);
}


/*********************************************/
/*      KALMAN FILTER SUPPORT FUNCTIONS      */
/*********************************************/

void update_constant_matrices(uint8_t state_size, uint8_t measurement_num, float F[], float FT[], float G[], float Gamma[], float B[], float n, float t1, float t2)
{
    // Update the State Transition Matrix
    compute_state_transition_matrix(F, n, t1, t2);

    // Calculate the Tranpose of F
    copy(F, FT, state_size*state_size);
    tran(FT, state_size, state_size);

    // Update the Gamma Matrix
    compute_gamma_matrix(Gamma, n, t1, t2);

    // Update the G Matrix
    mul(Gamma, B, G, state_size, state_size, measurement_num);
}

static void compute_state_transition_matrix(float F[], float n, float t1, float t2)
{
    // Calculate dt
    float dt = t2 - t1;

    /*  Populate the F matrix    */

    // First Row
    F[0] = (4 - (3*cos(dt*n)));
    F[1] = 0;
    F[2] = 0;
    F[3] = (1/n)*sin(dt*n);
    F[4] = (-2/n)*(cos(dt*n) - 1);
    F[5] = 0;

    // Second Row
    F[6] = 6*sin(dt*n) - (6*dt*n);
    F[7] = 1;
    F[8] = 0;
    F[9] = (2/n)*(cos(dt*n) - 1);
    F[10] = (1/n)*(4*sin(dt*n) - (3*dt*n));
    F[11] = 0;

    // Third Row
    F[12] = 0;
    F[13] = 0;
    F[14] = cos(dt*n);
    F[15] = 0;
    F[16] = 0;
    F[17] = (1/n)*sin(dt*n);
    
    // Fourth Row
    F[18] = 3*n*sin(dt*n);
    F[19] = 0;
    F[20] = 0;
    F[21] = cos(dt*n);
    F[22] = 2*sin(dt*n);
    F[23] = 0;

    // Fifth Row
    F[24] = 6*n*(cos(dt*n) - 1);
    F[25] = 0;
    F[26] = 0;
    F[27] = -2*sin(dt*n);
    F[28] = 4*cos(dt*n) - 3;
    F[29] = 0;

    // Sixth Row
    F[30] = 0;
    F[31] = 0;
    F[32] = -n*sin(dt*n);
    F[33] = 0;
    F[34] = 0;
    F[35] = cos(dt*n);
}

static void compute_gamma_matrix(float Gamma[], float n, float t1, float t2)
{
    // Compute dt
    float dt = t2 - t1;

    /*  Populate the Gamma Matrix   */

    // First Row
    Gamma[0] = (4*dt) - (3/n)*(sin(dt*n));
    Gamma[1] = 0;
    Gamma[2] = 0;
    Gamma[3] = (2/(pow(n,2)))*pow(sin((dt*n)/2),2);
    Gamma[4] = (-1/pow(n,2))*((2*sin(dt*n)) - (2*dt*n));
    Gamma[5] = 0;

    // Second Row
    Gamma[6] = ((12/n)*pow(sin((dt*n)/2),2)) - (3*n*pow(t1,2)) - (3*n*pow(t2,2)) + (6*n*t1*t2);
    Gamma[7] = dt;
    Gamma[8] = 0;
    Gamma[9] = (1/pow(n,2))*(2*sin(dt*n) - 2*dt*n);
    Gamma[10] = ((8/pow(n,2))*pow(sin((dt*n)/2),2)) + (3*t1*t2) - ((3/2)*pow(t1,2)) - ((3/2)*pow(t2,2));
    Gamma[11] = 0;

    // Third Row
    Gamma[12] = 0;
    Gamma[13] = 0;
    Gamma[14] = (1/n)*sin(dt*n);
    Gamma[15] = 0;
    Gamma[16] = 0;
    Gamma[17] = (2/pow(n,2))*pow(sin((dt*n)/2),2);

    // Fourth Row
    Gamma[18] = 6*pow(sin((dt*n)/2),2);
    Gamma[19] = 0;
    Gamma[20] = 0;
    Gamma[21] = (1/n)*sin(dt*n);
    Gamma[22] = (4/n)*pow(sin((dt*n)/2),2);
    Gamma[23] = 0;

    // Fifth Row
    Gamma[24] = (6*sin(dt*n)) - (6*dt*n);
    Gamma[25] = 0;
    Gamma[26] = 0;
    Gamma[27] = (-4/n)*pow((dt*n)/2, 2);
    Gamma[28] = (4/n)*sin(dt*n) - (3*dt);
    Gamma[29] = 0;

    // Sixth Row
    Gamma[30] = 0;
    Gamma[31] = 0;
    Gamma[32] = cos(dt*n) - 1;
    Gamma[33] = 0;
    Gamma[34] = 0;
    Gamma[35] = (1/n)*sin(dt*n);
}