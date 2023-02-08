/*
 ============================================================================
 Name        : KF_accel.c
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.1
 Copyright   : 
 Description : 
 ============================================================================
 */

#include <GAINSKF.h>

void KF()
{
    /*****  VARIABLE DECLARATION & PREALLOCATION  *****/

    //float dt = 1/100; //seconds

    uint16_t ASize = 6;                     // Number of raw data inputs (6 DOF accelerations)
    uint16_t StateSize = 6;                // State vector size

    float Z_n[ASize];                         // Input acceleration vector
    float sigma_std[6] = {1, 2, 3, 4, 5, 6};                 // Input std deviations vector
    float P_n_n[StateSize*StateSize];       // Estimate covariance matrix
    float X_n_n[StateSize];                 // State vector
    float U_n[StateSize];                   // Control/input variable vector
    float X_n_n_m_1[StateSize];             // State vector
    float P_n_n_m_1[StateSize*StateSize];   // Estimate covariance matrix
    float G[StateSize*StateSize];           // control matrix
    float F[StateSize*StateSize];           // State transition matrix
    float FT[StateSize*StateSize];          // Transposed state transition matrix
    float H[StateSize*ASize];               // Observation matrix
    float HT[ASize*StateSize];              // Transposed observation matrix
    float Q[StateSize*StateSize];           // Process noise matrix
    float R_n[ASize*ASize];                 // Measurement noise covariance matrix
    float X_n_p_1_n[StateSize];             // State vector
    float P_n_p_1_n[StateSize*StateSize];   // Estimate covariance matrix
    float K_n[StateSize*ASize];             // Kalman gain
    float I_SS[StateSize*StateSize];        // StateSize Idenitity matrix

    // Create identity matricies
    eye(I_SS, StateSize);
    eye(Q, StateSize);
    eye(P_n_n_m_1, StateSize);
    
    /*****  FILL VECTORS AND MATRICIES WITH DUMMY VALUES  *****/

    // 6x1
    for(int j = 0; j < ASize; j++)
    {
        Z_n[j] = 1; 
    }
        

    // 25x1
    for(int i = 0; i < StateSize; i++)
    {
        U_n[i] = 1; 
        X_n_n_m_1[i] = 1; 
    }

    // 25x6
    for(int i = 0; i < StateSize * ASize; i ++)
    {
        H[i] = 1;
    }

    // 25x25
    for(int i = 0; i < StateSize*StateSize; i++)
    {
        P_n_n[i] = 1;
        X_n_n[i] = 1;
        G[i] = 1; 
        F[i] = 1; //dt*dt - 1*2 + dt*dt;
    }

    /*********************************************/
    /*
	clock_t start, end;
	float cpu_time_used;
	start = clock();
    */
    /*********************************************/ 


    /*********************************************/
    /*****  X_n_p_1_n = F * X_n_n + G * U_n  *****/

    // Declare temp vars FX_n_n & GU_n
    float FX_n_n[StateSize];
    float GU_n[StateSize];

    // FX_n_n
    mul(F, X_n_n, FX_n_n, StateSize, StateSize, 1);

    // GU_n
    mul(G, U_n, GU_n, StateSize, StateSize, 1);

    //  X_n_p_1_n
    add(FX_n_n, GU_n, X_n_p_1_n, StateSize*StateSize);


    /*********************************************/
    /*****  P_n_p_1_n = F * P_n_n * F^T + Q  *****/

    // Declare temp vars
    float FP_n_n[StateSize*StateSize];
    float FP_n_nFT[StateSize*StateSize];

    // FT = F^T
    copy(F, FT, StateSize*StateSize);
    tran(FT, StateSize, StateSize);

    // FP_n_n = F * P_n_n
    mul(F, P_n_n, FP_n_n, StateSize, StateSize, StateSize);

    // FP_n_nFT = FP_n_n * FT
    mul(FP_n_n, FT, FP_n_nFT, StateSize, StateSize, StateSize);

    // P_n_p_1_n = FP_n_nFT + Q
    add(Q, FP_n_nFT, P_n_p_1_n, StateSize*StateSize);


    /********************************************************************/
    /*****  K_n = P_n_n_m_1 * H^T * (H * P_n_n_m_1 * H^T + R_n)^-1  *****/

    // Declare temp vars
    float P_n_n_m_1HT[StateSize*ASize];
    float HP_n_n_m_1[ASize*StateSize];
    float HP_n_n_m_1HT[ASize*ASize];
    float HP_n_n_m_1HTR_n[ASize*ASize];

    // HT = H^T
    copy(H, HT, StateSize*ASize);
    tran(HT, StateSize, ASize);

    // P_n_n_m_1HT = P_n_n_m_1 * HT
    mul(P_n_n_m_1, HT, P_n_n_m_1HT, StateSize, StateSize, ASize);

    // R_n = sigma * sigma
    // 6x1 * 1x6
    mul(sigma_std, sigma_std, R_n, ASize, 1, ASize); 

    // HP_n_n_m_1 = H * P_n_n_m_1
    mul(H, P_n_n_m_1, HP_n_n_m_1, ASize, StateSize, StateSize);

    // HP_n_n_m_1HT = HP_n_n_m_1 * HT
    mul(HP_n_n_m_1, HT, HP_n_n_m_1HT, ASize, StateSize, ASize);

    // HP_n_n_m_1HTR_n = HP_n_n_m_1HT + R_n
    add(HP_n_n_m_1HT, R_n, HP_n_n_m_1HTR_n, ASize*ASize);

    // Invert HP_n_n_m_1HTR_n
    if(inv(HP_n_n_m_1HTR_n, ASize) == 0)
    {
        printf("Error Inverting HP_n_n_m_1HTR_n in KF Eqn 3 \n");
    }

    // K_n = P_n_n_m_1HT * HP_n_n_m_1HTR_n
    mul(P_n_n_m_1HT, HP_n_n_m_1HTR_n, K_n, StateSize, ASize, ASize);


    /*************************************************************/
    /*****  X_n_n = X_n_n_m_1 + K_n * (Z_n - H * X_n_n_m_1)  *****/

    float HX_n_n_m_1[ASize];
    float Z_nHX_n_n_m_1[ASize];
    float K_nZ_nHX_n_n_m_1[StateSize];

    // HX_n_n_m_1 = H * X_n_n_m_1
    mul(H, X_n_n_m_1, HX_n_n_m_1, ASize, StateSize, 1);

    // Z_nHX_n_n_m_1 = Z_n - HX_n_n_m_1
    sub(Z_n, HX_n_n_m_1, Z_nHX_n_n_m_1, ASize);

    // K_nZ_nHX_n_n_m_1 = K_n * Z_nHX_n_n_m_1
    mul(K_n, Z_nHX_n_n_m_1, K_nZ_nHX_n_n_m_1, StateSize, ASize, 1);

    // X_n_n = X_n_n_m_1 + K_nZ_nHX_n_n_m_1
    add(X_n_n_m_1, K_nZ_nHX_n_n_m_1, X_n_n, StateSize);


    /*************************************************************************************/
    /*****  P_n_n = (I - K_n * H) * P_n_n_m_1 * (I - K_n * H)^T + K_n * R_n * K_n^T  *****/


    float K_nH[StateSize*StateSize];
    float IK_nH[StateSize*StateSize];
    float IK_nHT[StateSize*StateSize];
    float IK_nHP_n_n_m_1[StateSize*StateSize];
    float IK_nHP_n_n_m_1IK_nHT[StateSize*StateSize];
    float K_nT[ASize*StateSize];
    float K_nR_n[StateSize*ASize];
    float K_nR_nK_nT[StateSize*ASize];

    // K_nH = K_n * H
    mul(K_n, H, K_nH, StateSize, ASize, StateSize);

    // IK_nH = I - K_nH
    sub(I_SS, K_nH, IK_nH, StateSize*StateSize);

    // IK_nHT = (I - K_nH)^T
    copy(IK_nH, IK_nHT, StateSize*StateSize);
    tran(IK_nHT, StateSize, StateSize);

    // IK_nHP_n_n_m_1 = IK_nH * P_n_n_m_1
    mul(IK_nH, P_n_n_m_1, IK_nHP_n_n_m_1, StateSize, StateSize, StateSize);

    // IK_nHP_n_n_m_1IK_nHT = IK_nH * P_n_n_m_1
    mul(IK_nHP_n_n_m_1, IK_nHT, IK_nHP_n_n_m_1IK_nHT, StateSize, StateSize, StateSize);

    // K_nT = (K_n)^T
    copy(K_n, K_nT, StateSize*ASize);
    tran(K_nT, StateSize, ASize);

    // K_nR_n = K_n * R_n
    mul(K_n, R_n, K_nR_n, StateSize, ASize, ASize);

    // K_nR_nK_nT = K_nR_n * K_nT
    mul(K_nR_n, K_nT, K_nR_nK_nT, StateSize, ASize, StateSize);

    // P_n_n = IK_nHP_n_n_m_1IK_nHT + K_nR_nK_nT
    add(IK_nHP_n_n_m_1IK_nHT, K_nR_nK_nT, P_n_n, StateSize*StateSize);



    /*********************************************/
    /*
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %0.18f\n", cpu_time_used);
    */
    /*********************************************/

}