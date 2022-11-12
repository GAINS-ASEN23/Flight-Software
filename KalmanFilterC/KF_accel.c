/*
 ============================================================================
 Name        : KF_accel.c
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.1
 Copyright   : 
 Description : 
 ============================================================================
 */

#include "CControl/Headers/Functions.h"
#include "add.c"

int main()
{

    /*****  VARIABLE DECLARATION & PREALLOCATION  *****/
    float dt = 1/100; //seconds

    uint16_t ASize = 6;                     // Number of raw data inputs (6 DOF accelerations)
    uint16_t StateSize = 25;                // State vector size

    float A[ASize];                         // Input acceleration vector
    float sigma_std[ASize];                 // Input std deviations vector
    float P_n_n[StateSize*StateSize];       // Estimate covariance matrix
    float X_n_n[StateSize];                 // State vector
    float U_n[StateSize];                   // Control/input variable vector
    float X_n_n_m_1[StateSize];             // State vector
    float P_n_n_m_1[StateSize*StateSize];   // Estimate covariance matrix
    float G[StateSize*StateSize];           // control matrix
    float F[StateSize*StateSize];           // State transition matrix
    float FTran[StateSize*StateSize];       // Transposed state transition matrix
    float H[StateSize];                     // Observation matrix
    float HTran[StateSize];                 // Transposed observation matrix
    float Q_meas[StateSize*StateSize];      // Process noise matrix
    float R_n[ASize*ASize];                 // Measurement noise covariance matrix

    float X_n_p_1_n[StateSize];             // State vector

    
    /*****  FILL VECTORS AND MATRICIES WITH DUMMY VALUES  *****/

    // 6x1
    for(int j = 0; j < ASize; j++)
    {
        A[j] = 1;                           // pre-allocating sensor data from each accelerometer
        sigma_std[j] = 1;                   // pre-allocating error (std) from each acelerometer
    }
        

    // 25x1
    for(int i = 0; i < StateSize; i++)
    {
        U_n[i] = 1; 
        X_n_n_m_1[i] = 1;                   // predicted state
    }

    // 25x6
    for(int i = 0; i < StateSize * ASize; i ++)
    {
        H[i] = 1;                           //observation matrix (conversions happen)
        // HTran[i] = 1;                    //copy of H (for transpose
    }

    // 25x25
    for(int i = 0; i < StateSize*StateSize; i++)
    {
        P_n_n[i] = 1;                       // estimate uncertinity
        X_n_n[i] = 1;                       // state estimate
        P_n_n_m_1[i] = 1;                   // predicted error
        G[i] = 1;                           // control matrix
        F[i] = dt*dt - 1*2 + dt*dt;         // state transition matrix
        // FTran[i] = 1;                    // copy of state transion matrix copy to put in function
        Q_meas[i] = 1;                      //Process noise matrix
    }

    /*****  X_n_p_1_n = F * X_n_n + G * U_n  *****/

    // Declare temp vars FX_n_n & GU_n
    float FX_n_n[StateSize];
    float GU_n[StateSize];

    // FX_n_n
    mul(F, X_n_n, FX_n_n, StateSize, StateSize, 1);

    // GU_n
    mul(G, U_n, GU_n, StateSize, StateSize, 1);


    add(FX_n_n, GU_n, X_n_p_1_n, StateSize);

    print(X_n_p_1_n, StateSize, 1);


/*


    // R_n = sigma * sigma
    // 6x1 * 1x6
    mul(sigma_std, sigma_std, R_n, ASize, 1, ASize); 


// Define Process Noise Matrix 
float Q[StateSize][StateSize];
float QTemp[StateSize][StateSize];

    mul(F, Q_meas, QTemp, StateSize, StateSize, StateSize);
    mul(QTemp, FTran, Q, StateSize, StateSize, StateSize);





    // *****Updating state to future state********

    // Exteroplate state x_n_p_1_n and P_n_p_1_n
    float x_n_p_1_n_temp1[StateSize];
    float x_n_p_1_n_temp2[StateSize];
    float P_n_p_1_n_temp1[StateSize][StateSize];
    float P_n_p_1_n_temp2[StateSize][StateSize];

    mul(F, x_n_n, x_n_p_1_n_temp1, StateSize, StateSize, 1);
    mul(G, U_n, x_n_p_1_n_temp2, StateSize, StateSize, 1);


    float x_n_p_1_n[StateSize] = x_n_p_1_n_temp1 + x_n_p_1_n_temp2; // am I allowed to include a toolbox for adding

    mul(F, P_n_n, P_n_p_1_n_temp1, StateSize, StateSize, StateSize);
    mul(P_n_p_1_n_temp1, FTran, P_n_p_1_n_temp2, StateSize, StateSize, StateSize);

    float P_n_p_1_n[StateSize][StateSize] = P_n_p_1_n_temp2 + Q;





    // **** Measurement Updates *****

    // Kalman gain
    float K_n;
    float K_n_temp1[StateSize];
    float K_n_temp2[1];

    mul(H, P_n_p_1_n, K_n_temp1, 1, StateSize, StateSize);
    mul(K_n_temp1, HTran, K_n_temp2, 1, StateSize, StateSize);






      % Compute the Kalman Gain
        K_n = P_n_p_1_n*H'*((H*P_n_p_1_n*H' + R_n)^-1); // dementions don't add up here in this term (H*P_n_p_1_n*H' + R_n) ahhhh



    


*/





}


// notes (main changes from matlab version)
// 1) A is a 6 by 6 at one specific time step