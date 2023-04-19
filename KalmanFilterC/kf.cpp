/*
============================================================================
Name        : kf.c
Author      : Jason Popich, Bennett Grow, Kaylie Rick
Version     : 0.1
Copyright   : 
Description : This file contains the Kalman Filter (KF) function definitions
============================================================================
*/

#include "kf.hpp"

/*********************************************/
/*     KALMAN FILTER CLASS FUNCTIONS         */
/*********************************************/

// Initialization Constructor
KalmanFilter::KalmanFilter(int n_x, int n_u, int n_z, float* x_n_n, float* P_n_n, bool CW_or_k)
{
    // Set the KF environment vars
    KalmanFilter::CW_or_K = CW_or_k; 
    KalmanFilter::n_x = n_x;
    KalmanFilter::n_u = n_u;
    KalmanFilter::n_z = n_z;

    /*    Preallocate Matricies and Vectors    */

    // State and Uncertainty Matricies
    KalmanFilter::x_n_p_1_n = (float*)malloc((n_x * 1) * sizeof(float));                   // Current Predicted State vector
    KalmanFilter::P_n_p_1_n = (float*)malloc((n_x * n_x) * sizeof(float));                 // Current Predicted Uncertainty Matrix

    KalmanFilter::P_n_n = P_n_n;                                                            // Estimate Uncertainty Matrix (must be defined outside of this class)
    KalmanFilter::x_n_n = x_n_n;                                                            // Estimate State vector(must be defined outside of this class)

    // Kalman Filter Matricies
    KalmanFilter::F = (float*)malloc((n_x * n_x) * sizeof(float));                          // State Transition Matrix
    KalmanFilter::FT = (float*)malloc((n_x * n_x) * sizeof(float));                         // Transposed State Transition Matrix

    KalmanFilter::G = (float*)malloc((n_x * n_u) * sizeof(float));                          // Control Matrix
    KalmanFilter::B = (float*)malloc((n_x * n_u) * sizeof(float));                          // Control Observation Matrix
    KalmanFilter::BT = (float*)malloc((n_u * n_x) * sizeof(float));                         // Transposed Control Observation Matrix
    KalmanFilter::Gamma = (float*)malloc((n_x * n_x) * sizeof(float));                      // Gamma Matrix (No other name??)
    KalmanFilter::GammaT = (float*)malloc((n_x * n_x) * sizeof(float));                     // Gamma Matrix Transposed (No other name??)

    KalmanFilter::H = (float*)malloc((n_z * n_x) * sizeof(float));                          // Observation Matrix
    KalmanFilter::HT = (float*)malloc((n_x * n_z) * sizeof(float));                         // Transposed Observation Matrix
    eye(KalmanFilter::H, n_z);                                                              // Define the H matrix as the identity for this application

    KalmanFilter::Q = (float*)malloc((n_x * n_x) * sizeof(float));                          // Process Noise Matrix
    KalmanFilter::Q_a = (float*)malloc((n_u * n_u) * sizeof(float));                        // Input Noise Matrix
    eye(KalmanFilter::Q_a, KalmanFilter::n_u);                                              // Make the input covariance matrix an identity matrix

    KalmanFilter::R_n = (float*)malloc((n_z * n_z) * sizeof(float));                        // Measurement Noise Covariance Matrix
    eye(KalmanFilter::R_n, n_z);                                                            // Make the R_n matrix an identity matrix

    KalmanFilter::K_n = (float*)malloc((n_x * n_z) * sizeof(float));                        // Kalman gain

    KalmanFilter::I_SS = (float*)malloc((n_x * n_z) * sizeof(float));                       // Idenitity matrix (for K_n)
    eye(KalmanFilter::I_SS, n_x);                                                           // Define the Identity Matrix

    /*  Set the B matricies */
    KalmanFilter::set_b();
}

void KalmanFilter::KF_run(float t1, float t2, float n)
{
    // Update the time
    KalmanFilter::t1 = t1;
    KalmanFilter::t2 = t2;

    // Update all the constant matricies
    KalmanFilter::update_constant_matrices(n);

    // Predict the state
    KalmanFilter::predict_state();

    // Predict the uncertainty
    // KalmanFilter::predict_uncertainty();

    // Calculate the Kalman Gain
    // KalmanFilter::compute_kalman_gain();

    // Estimate the current state based off of the previous state prediction and the current measurement
    // KalmanFilter::estimate_state();

    // Estimate the Uncertainty of the current state estimate
    // KalmanFilter::estimate_uncertainty();
}

void KalmanFilter::set_r_n(float sigma[])
{
    // Populate the diagonals based on the sigma vector given
    for(int i = 0; i < KalmanFilter::n_z; i++)
    {
        KalmanFilter::R_n[(KalmanFilter::n_z + 1) * i] = pow(sigma[i], 2);
    }
}

void KalmanFilter::set_q_a(float sigma[])
{
    // Populate the diagonals based on the sigma vector given
    for(int i = 0; i < KalmanFilter::n_u; i++)
    {
        KalmanFilter::Q_a[(KalmanFilter::n_u + 1) * i] = pow(sigma[i], 2);
    }
}

void KalmanFilter::set_h(bool ground_contact)
{
    if (ground_contact == false)
    {
        // Zero out H matrix
        KalmanFilter::H[0] = 0;
        KalmanFilter::H[7] = 0;
        KalmanFilter::H[14] = 0;
        KalmanFilter::H[21] = 0;
        KalmanFilter::H[28] = 0;
        KalmanFilter::H[35] = 0;
    }
    else
    {
        // Make H the identity matrix
        KalmanFilter::H[0] = 1;
        KalmanFilter::H[7] = 1;
        KalmanFilter::H[14] = 1;
        KalmanFilter::H[21] = 1;
        KalmanFilter::H[28] = 1;
        KalmanFilter::H[35] = 1;
    }
}

void KalmanFilter::set_b()
{
    // Fill Out B
    KalmanFilter::B[0] = 0;
    KalmanFilter::B[1] = 0;
    KalmanFilter::B[2] = 0;
    KalmanFilter::B[3] = 0;
    KalmanFilter::B[4] = 0;
    KalmanFilter::B[5] = 0;
    KalmanFilter::B[6] = 0;
    KalmanFilter::B[7] = 0;
    KalmanFilter::B[8] = 0;
    KalmanFilter::B[9] = 1;
    KalmanFilter::B[10] = 0;
    KalmanFilter::B[11] = 0;
    KalmanFilter::B[12] = 0;
    KalmanFilter::B[13] = 1;
    KalmanFilter::B[14] = 0;
    KalmanFilter::B[15] = 0;
    KalmanFilter::B[16] = 0;
    KalmanFilter::B[17] = 1;

    // Calculate the Tranpose of B
    copy(KalmanFilter::B, KalmanFilter::BT, KalmanFilter::n_x*KalmanFilter::n_u);
    tran(KalmanFilter::BT, KalmanFilter::n_x, KalmanFilter::n_u);
}

void KalmanFilter::set_det_input_vector(float* u_n)
{
    KalmanFilter::u_n = u_n;
}

void KalmanFilter::set_mea_input_vector(float* z_n)
{
    KalmanFilter::z_n = z_n;
}

/*********************************************/
/*      MAIN KALMAN FILTER FUNCTIONS         */
/*********************************************/

void KalmanFilter::predict_state()
{
    /*          Equation               */
    /* x_n_p_1_n = F * x_n_n + G * u_n */

    // Declare temp vars
    float FX_n_n[KalmanFilter::n_x];
    float GU_n[KalmanFilter::n_x];

    // Set the memory for the temp vars
    memset(FX_n_n, 0, KalmanFilter::n_x*sizeof(float));
    memset(GU_n, 0, KalmanFilter::n_x*sizeof(float));

    // F*x_n_n
    mul(F, x_n_n, FX_n_n, KalmanFilter::n_x, KalmanFilter::n_x, 1);
    // G*u_n
    mul(KalmanFilter::G, KalmanFilter::u_n, GU_n, KalmanFilter::n_x, KalmanFilter::n_u, 1);

    //  x_n_p_1_n
    add(FX_n_n, GU_n, KalmanFilter::x_n_p_1_n, KalmanFilter::n_x);
    set_var(x_n_p_1_n[1]);

    copy(KalmanFilter::x_n_p_1_n, KalmanFilter::x_n_n, KalmanFilter::n_x);
}

void KalmanFilter::predict_uncertainty()
{
    /*          Equation               */
    /* P_n_p_1_n = F * P_n_n * F^T + Q s*/

    // Declare temp vars
    float FP_n_n[KalmanFilter::n_x*KalmanFilter::n_x];
    float FP_n_nFT[KalmanFilter::n_x*KalmanFilter::n_x];

    // Set the memory for the temp vars
    memset(FP_n_n, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(FP_n_nFT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));

    // FP_n_n = F * P_n_n
    mul(KalmanFilter::F, KalmanFilter::P_n_n, FP_n_n, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);

    // FP_n_nFT = FP_n_n * FT
    mul(FP_n_n, KalmanFilter::FT, FP_n_nFT, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);

    // P_n_p_1_n = F*P_n_n*FT + Q
    add(FP_n_nFT, KalmanFilter::Q, KalmanFilter::P_n_p_1_n, KalmanFilter::n_x*KalmanFilter::n_x);
}

void KalmanFilter::compute_kalman_gain()
{
    /*                  Equation                        */
    /*  K_n = P_n_p_1_n * H^T * (H * P_n_p_1_n * H^T + R_n)^-1  */

    // Declare temp vars
    float P_n_n_m_1HT[KalmanFilter::n_x*KalmanFilter::n_x];
    float HP_n_n_m_1[KalmanFilter::n_z*KalmanFilter::n_x];
    float HP_n_n_m_1HT[KalmanFilter::n_z*KalmanFilter::n_z];
    float HP_n_n_m_1HTR_n[KalmanFilter::n_z*KalmanFilter::n_z];
    float HT[KalmanFilter::n_z*KalmanFilter::n_x];

    // Set the memory for the temp vars
    memset(P_n_n_m_1HT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(HP_n_n_m_1, 0, KalmanFilter::n_z*KalmanFilter::n_x*sizeof(float));
    memset(HP_n_n_m_1HT, 0, KalmanFilter::n_z*KalmanFilter::n_z*sizeof(float));
    memset(HP_n_n_m_1HTR_n, 0, KalmanFilter::n_z*KalmanFilter::n_z*sizeof(float));
    memset(HT, 0, KalmanFilter::n_z*KalmanFilter::n_x*sizeof(float));

    // HT = H^T
    copy(KalmanFilter::H, HT, KalmanFilter::n_x*KalmanFilter::n_z);
    tran(HT, KalmanFilter::n_x, KalmanFilter::n_z);

    // P_n_n_m_1HT = P_n_p_1_n * HT
    mul(KalmanFilter::P_n_p_1_n, HT, P_n_n_m_1HT, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_z);

    // HP_n_n_m_1 = H * P_n_p_1_n
    mul(H, KalmanFilter::P_n_p_1_n, HP_n_n_m_1, KalmanFilter::n_z, KalmanFilter::n_x, KalmanFilter::n_x);

    // HP_n_n_m_1HT = HP_n_n_m_1 * HT
    mul(HP_n_n_m_1, HT, HP_n_n_m_1HT, KalmanFilter::n_z, KalmanFilter::n_x, KalmanFilter::n_z);

    // HP_n_n_m_1HTR_n = HP_n_n_m_1HT + R_n
    add(HP_n_n_m_1HT, KalmanFilter::R_n, HP_n_n_m_1HTR_n, KalmanFilter::n_z*KalmanFilter::n_z);

    // Invert HP_n_n_m_1HTR_n
    if(inv(HP_n_n_m_1HTR_n, KalmanFilter::n_z) == 0)
    {
        // printf("Error Inverting HP_n_n_m_1HTR_n in KF Eqn 3 \n");
    }

    // K_n = P_n_n_m_1HT * HP_n_n_m_1HTR_n
    mul(P_n_n_m_1HT, HP_n_n_m_1HTR_n, KalmanFilter::K_n, KalmanFilter::n_x, KalmanFilter::n_z, KalmanFilter::n_z);
}

void KalmanFilter::estimate_state()
{
    /*                    Equation                       */
    /*  X_n_n = x_n_p_1_n + K_n * (Z_n - H * x_n_p_1_n)  */

    // Declare the temp vars
    float HX_n_n_m_1[KalmanFilter::n_z];
    float Z_nHX_n_n_m_1[KalmanFilter::n_z];
    float K_nZ_nHX_n_n_m_1[KalmanFilter::n_x];

    // Set the memory for the temp vars
    memset(HX_n_n_m_1, 0, KalmanFilter::n_z*sizeof(float));
    memset(Z_nHX_n_n_m_1, 0, KalmanFilter::n_z*sizeof(float));
    memset(K_nZ_nHX_n_n_m_1, 0, KalmanFilter::n_x*sizeof(float));

    // HX_n_n_m_1 = H * x_n_p_1_n
    mul(KalmanFilter::H, x_n_p_1_n, HX_n_n_m_1, KalmanFilter::n_z, KalmanFilter::n_x, 1);

    // Z_nHX_n_n_m_1 = Z_n - HX_n_n_m_1
    sub(KalmanFilter::z_n, HX_n_n_m_1, Z_nHX_n_n_m_1, KalmanFilter::n_z);

    // K_nZ_nHX_n_n_m_1 = K_n * Z_nHX_n_n_m_1
    mul(KalmanFilter::K_n, Z_nHX_n_n_m_1, K_nZ_nHX_n_n_m_1, KalmanFilter::n_x, KalmanFilter::n_z, 1);

    // X_n_n = x_n_p_1_n + K_nZ_nHX_n_n_m_1
    add(KalmanFilter::x_n_p_1_n, K_nZ_nHX_n_n_m_1, KalmanFilter::x_n_n, KalmanFilter::n_x);
}

void KalmanFilter::estimate_uncertainty()
{
    /*                                Equation                                   */
    /*  P_n_n = (I - K_n * H) * P_n_p_1_n * (I - K_n * H)^T + K_n * R_n * K_n^T  */

    // Declare Temp Vars
    float K_nH[KalmanFilter::n_x*KalmanFilter::n_x];
    float IK_nH[KalmanFilter::n_x*KalmanFilter::n_x];
    float IK_nHT[KalmanFilter::n_x*KalmanFilter::n_x];
    float IK_nHP_n_n_m_1[KalmanFilter::n_x*KalmanFilter::n_x];
    float IK_nHP_n_n_m_1IK_nHT[KalmanFilter::n_x*KalmanFilter::n_x];
    float K_nT[KalmanFilter::n_z*KalmanFilter::n_x];
    float K_nR_n[KalmanFilter::n_x*KalmanFilter::n_z];
    float K_nR_nK_nT[KalmanFilter::n_x*KalmanFilter::n_z];

    // Set the memory for the temp vars
    memset(K_nH, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(IK_nH, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(IK_nHT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(IK_nHP_n_n_m_1, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(IK_nHP_n_n_m_1IK_nHT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(K_nT, 0, KalmanFilter::n_z*KalmanFilter::n_x*sizeof(float));
    memset(K_nR_n, 0, KalmanFilter::n_x*KalmanFilter::n_z*sizeof(float));
    memset(K_nR_nK_nT, 0, KalmanFilter::n_x*KalmanFilter::n_z*sizeof(float));

    // K_nH = K_n * H
    mul(KalmanFilter::K_n, KalmanFilter::H, K_nH, KalmanFilter::n_x, KalmanFilter::n_z, KalmanFilter::n_x);

    // IK_nH = I - K_nH
    sub(KalmanFilter::I_SS, K_nH, IK_nH, KalmanFilter::n_x*KalmanFilter::n_x);

    // IK_nHT = (I - K_nH)^T
    copy(IK_nH, IK_nHT, KalmanFilter::n_x*KalmanFilter::n_x);
    tran(IK_nHT, KalmanFilter::n_x, KalmanFilter::n_x);

    // IK_nHP_n_n_m_1 = IK_nH * P_n_n_m_1
    mul(IK_nH, KalmanFilter::P_n_p_1_n, IK_nHP_n_n_m_1, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);

    // IK_nHP_n_n_m_1IK_nHT = IK_nH * P_n_n_m_1
    mul(IK_nHP_n_n_m_1, IK_nHT, IK_nHP_n_n_m_1IK_nHT, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);

    // K_nT = (K_n)^T
    copy(KalmanFilter::K_n, K_nT, KalmanFilter::n_x*KalmanFilter::n_z);
    tran(K_nT, KalmanFilter::n_x, KalmanFilter::n_z);

    // K_nR_n = K_n * R_n
    mul(KalmanFilter::K_n, KalmanFilter::R_n, K_nR_n, KalmanFilter::n_x, KalmanFilter::n_z, KalmanFilter::n_z);

    // K_nR_nK_nT = K_nR_n * K_nT
    mul(K_nR_n, K_nT, K_nR_nK_nT, KalmanFilter::n_x, KalmanFilter::n_z, KalmanFilter::n_x);

    // P_n_n = IK_nHP_n_n_m_1IK_nHT + K_nR_nK_nT
    add(IK_nHP_n_n_m_1IK_nHT, K_nR_nK_nT, KalmanFilter::P_n_n, KalmanFilter::n_x*KalmanFilter::n_x);
}


/*********************************************/
/*      KALMAN FILTER SUPPORT FUNCTIONS      */
/*********************************************/

void KalmanFilter::update_constant_matrices(float n)
{
    // Update the State Transition Matrix
    compute_state_transition_matrix(n);

    // Calculate the Tranpose of F
    copy(KalmanFilter::F, KalmanFilter::FT, KalmanFilter::n_x*KalmanFilter::n_x);
    tran(KalmanFilter::FT, KalmanFilter::n_x, KalmanFilter::n_x);

    // Update the Gamma Matrix
    compute_gamma_matrix(n);
    copy(KalmanFilter::Gamma, KalmanFilter::GammaT, KalmanFilter::n_x*KalmanFilter::n_x);
    tran(KalmanFilter::GammaT, KalmanFilter::n_x, KalmanFilter::n_x);

    // Update the G Matrix
    mul(KalmanFilter::Gamma, KalmanFilter::B, KalmanFilter::G, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_u);

    // Update the process noise covariance matrix
    compute_process_noise_covariance_matrix();
}

void KalmanFilter::compute_state_transition_matrix(float n)
{
    // Calculate dt
    float dt = KalmanFilter::t2 - KalmanFilter::t1;

    /*  Populate the F matrix    */
    if (KalmanFilter::CW_or_K == 0)
    {
        // First Row
        KalmanFilter::F[0] = (4 - (3*cos(dt*n)));
        KalmanFilter::F[1] = 0;
        KalmanFilter::F[2] = 0;
        KalmanFilter::F[3] = (1/n)*sin(dt*n);
        KalmanFilter::F[4] = (-2/n)*(cos(dt*n) - 1);
        KalmanFilter::F[5] = 0;

        // Second Row
        KalmanFilter::F[6] = 6*sin(dt*n) - (6*dt*n);
        KalmanFilter::F[7] = 1;
        KalmanFilter::F[8] = 0;
        KalmanFilter::F[9] = (2/n)*(cos(dt*n) - 1);
        KalmanFilter::F[10] = (1/n)*(4*sin(dt*n) - (3*dt*n));
        KalmanFilter::F[11] = 0;

        // Third Row
        KalmanFilter::F[12] = 0;
        KalmanFilter::F[13] = 0;
        KalmanFilter::F[14] = cos(dt*n);
        KalmanFilter::F[15] = 0;
        KalmanFilter::F[16] = 0;
        KalmanFilter::F[17] = (1/n)*sin(dt*n);

        // Fourth Row
        KalmanFilter::F[18] = 3*n*sin(dt*n);
        KalmanFilter::F[19] = 0;
        KalmanFilter::F[20] = 0;
        KalmanFilter::F[21] = cos(dt*n);
        KalmanFilter::F[22] = 2*sin(dt*n);
        KalmanFilter::F[23] = 0;

        // Fifth Row
        KalmanFilter::F[24] = 6*n*(cos(dt*n) - 1);
        KalmanFilter::F[25] = 0;
        KalmanFilter::F[26] = 0;
        KalmanFilter::F[27] = -2*sin(dt*n);
        KalmanFilter::F[28] = 4*cos(dt*n) - 3;
        KalmanFilter::F[29] = 0;

        // Sixth Row
        KalmanFilter::F[30] = 0;
        KalmanFilter::F[31] = 0;
        KalmanFilter::F[32] = -n*sin(dt*n);
        KalmanFilter::F[33] = 0;
        KalmanFilter::F[34] = 0;
        KalmanFilter::F[35] = cos(dt*n);
    }
    else 
    { // Use kinematic equations

        // First Row
        KalmanFilter::F[0] = 1;
        KalmanFilter::F[1] = 0;
        KalmanFilter::F[2] = 0;
        KalmanFilter::F[3] = dt;
        KalmanFilter::F[4] = 0;
        KalmanFilter::F[5] = 0;

        // Second Row
        KalmanFilter::F[6] = 0;
        KalmanFilter::F[7] = 1;
        KalmanFilter::F[8] = 0;
        KalmanFilter::F[9] = 0;
        KalmanFilter::F[10] = dt;
        KalmanFilter::F[11] = 0;

        // Third Row
        KalmanFilter::F[12] = 0;
        KalmanFilter::F[13] = 0;
        KalmanFilter::F[14] = 1;
        KalmanFilter::F[15] = 0;
        KalmanFilter::F[16] = 0;
        KalmanFilter::F[17] = dt;

        // Fourth Row
        KalmanFilter::F[18] = 0;
        KalmanFilter::F[19] = 0;
        KalmanFilter::F[20] = 0;
        KalmanFilter::F[21] = 1;
        KalmanFilter::F[22] = 0;
        KalmanFilter::F[23] = 0;

        // Fifth Row
        KalmanFilter::F[24] = 0;
        KalmanFilter::F[25] = 0;
        KalmanFilter::F[26] = 0;
        KalmanFilter::F[27] = 0;
        KalmanFilter::F[28] = 1;
        KalmanFilter::F[29] = 0;

        // Sixth Row
        KalmanFilter::F[30] = 0;
        KalmanFilter::F[31] = 0;
        KalmanFilter::F[32] = 0;
        KalmanFilter::F[33] = 0;
        KalmanFilter::F[34] = 0;
        KalmanFilter::F[35] = 1;

    }
}

void KalmanFilter::compute_gamma_matrix(float n)
{
    // Compute dt
    float dt = KalmanFilter::t2 - KalmanFilter::t1;

    /*  Populate the Gamma Matrix   */

    if (KalmanFilter::CW_or_K == 0)
    {
        // First Row
        KalmanFilter::Gamma[0] = (4*dt) - (3/n)*(sin(dt*n));
        KalmanFilter::Gamma[1] = 0;
        KalmanFilter::Gamma[2] = 0;
        KalmanFilter::Gamma[3] = (2/(pow(n,2)))*pow(sin((dt*n)/2),2);
        KalmanFilter::Gamma[4] = (-1/pow(n,2))*((2*sin(dt*n)) - (2*dt*n));
        KalmanFilter::Gamma[5] = 0;

        // Second Row
        KalmanFilter::Gamma[6] = ((12/n)*pow(sin((dt*n)/2),2)) - (3*n*pow(t1,2)) - (3*n*pow(t2,2)) + (6*n*t1*t2);
        KalmanFilter::Gamma[7] = dt;
        KalmanFilter::Gamma[8] = 0;
        KalmanFilter::Gamma[9] = (1/pow(n,2))*(2*sin(dt*n) - 2*dt*n);
        KalmanFilter::Gamma[10] = ((8/pow(n,2))*pow(sin((dt*n)/2),2)) + (3*t1*t2) - ((3/2)*pow(t1,2)) - ((3/2)*pow(t2,2));
        KalmanFilter::Gamma[11] = 0;

        // Third Row
        KalmanFilter::Gamma[12] = 0;
        KalmanFilter::Gamma[13] = 0;
        KalmanFilter::Gamma[14] = (1/n)*sin(dt*n);
        KalmanFilter::Gamma[15] = 0;
        KalmanFilter::Gamma[16] = 0;
        KalmanFilter::Gamma[17] = (2/pow(n,2))*pow(sin((dt*n)/2),2);

        // Fourth Row
        KalmanFilter::Gamma[18] = 6*pow(sin((dt*n)/2),2);
        KalmanFilter::Gamma[19] = 0;
        KalmanFilter::Gamma[20] = 0;
        KalmanFilter::Gamma[21] = (1/n)*sin(dt*n);
        KalmanFilter::Gamma[22] = (4/n)*pow(sin((dt*n)/2),2);
        KalmanFilter::Gamma[23] = 0;

        // Fifth Row
        KalmanFilter::Gamma[24] = (6*sin(dt*n)) - (6*dt*n);
        KalmanFilter::Gamma[25] = 0;
        KalmanFilter::Gamma[26] = 0;
        KalmanFilter::Gamma[27] = (-4/n)*pow((dt*n)/2, 2);
        KalmanFilter::Gamma[28] = (4/n)*sin(dt*n) - (3*dt);
        KalmanFilter::Gamma[29] = 0;

        // Sixth Row
        KalmanFilter::Gamma[30] = 0;
        KalmanFilter::Gamma[31] = 0;
        KalmanFilter::Gamma[32] = cos(dt*n) - 1;
        KalmanFilter::Gamma[33] = 0;
        KalmanFilter::Gamma[34] = 0;
        KalmanFilter::Gamma[35] = (1/n)*sin(dt*n);
    }
    else // Kinematics Gamma
    {
        // First Row
        KalmanFilter::Gamma[0] = dt;
        KalmanFilter::Gamma[1] = 0;
        KalmanFilter::Gamma[2] = 0;
        KalmanFilter::Gamma[3] = pow(dt,2)/2.0;
        KalmanFilter::Gamma[4] = 0;
        KalmanFilter::Gamma[5] = 0;

        // Second Row
        KalmanFilter::Gamma[6] = 0;
        KalmanFilter::Gamma[7] = dt;
        KalmanFilter::Gamma[8] = 0;
        KalmanFilter::Gamma[9] = 0;
        KalmanFilter::Gamma[10] = pow(dt,2)/2.0;
        KalmanFilter::Gamma[11] = 0;

        // Third Row
        KalmanFilter::Gamma[12] = 0;
        KalmanFilter::Gamma[13] = 0;
        KalmanFilter::Gamma[14] = dt;
        KalmanFilter::Gamma[15] = 0;
        KalmanFilter::Gamma[16] = 0;
        KalmanFilter::Gamma[17] = pow(dt,2)/2.0;

        // Fourth Row
        KalmanFilter::Gamma[18] = 0;
        KalmanFilter::Gamma[19] = 0;
        KalmanFilter::Gamma[20] = 0;
        KalmanFilter::Gamma[21] = dt;
        KalmanFilter::Gamma[22] = 0;
        KalmanFilter::Gamma[23] = 0;

        // Fifth Row
        KalmanFilter::Gamma[24] = 0;
        KalmanFilter::Gamma[25] = 0;
        KalmanFilter::Gamma[26] = 0;
        KalmanFilter::Gamma[27] = 0;
        KalmanFilter::Gamma[28] = dt;
        KalmanFilter::Gamma[29] = 0;

        // Sixth Row
        KalmanFilter::Gamma[30] = 0;
        KalmanFilter::Gamma[31] = 0;
        KalmanFilter::Gamma[32] = 0;
        KalmanFilter::Gamma[33] = 0;
        KalmanFilter::Gamma[34] = 0;
        KalmanFilter::Gamma[35] = dt;
    }
}

void KalmanFilter::compute_process_noise_covariance_matrix()
{
    /*                  Equation                */
    /*  Q = Gamma * (B * Q_a * B^T) * Gamma^T   */
    /*           Q = G * Q_a * G                */

    // Declare Temp Vars
    float BQ_a[KalmanFilter::n_x*KalmanFilter::n_u];
    float BQ_aBT[KalmanFilter::n_x*KalmanFilter::n_x];
    float GammaBQ_aBT[KalmanFilter::n_x*KalmanFilter::n_x];

    // Set the memory for the temp vars
    memset(BQ_a, 0, KalmanFilter::n_x*KalmanFilter::n_u*sizeof(float));
    memset(BQ_aBT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));
    memset(GammaBQ_aBT, 0, KalmanFilter::n_x*KalmanFilter::n_x*sizeof(float));

    // BQ_a = B * Q_a
    mul(KalmanFilter::B, KalmanFilter::Q_a, BQ_a, KalmanFilter::n_x, KalmanFilter::n_u, KalmanFilter::n_u);

    // BQ_aBT = BQ_a * B^T
    mul(BQ_a, KalmanFilter::BT, BQ_aBT, KalmanFilter::n_x, KalmanFilter::n_u, KalmanFilter::n_x);

    // GammaBQ_aBT = F * BQ_aBT
    mul(KalmanFilter::Gamma, BQ_aBT, GammaBQ_aBT, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);

    // Q = GammaBQ_aBT * FT
    mul(GammaBQ_aBT, KalmanFilter::Gamma, KalmanFilter::Q, KalmanFilter::n_x, KalmanFilter::n_x, KalmanFilter::n_x);
}