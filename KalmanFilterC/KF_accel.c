// Include Libraries
#include "CControl/Headers/Functions.h"
//#include "KF_accel.h"

int main()
{

    // struct State KF;

     float dt = 1/100; //seconds

     uint16_t ASize = 6;
     uint16_t StateSize = 25;

     float A[ASize]; //
     float sigma_std[ASize];
     float P_n_n[StateSize*StateSize];
     float X_n_n[StateSize]; // also known as state
     float U_n[StateSize];
     float X_n_n_m_1[StateSize];
     float P_n_n_m_1[StateSize*StateSize];
     float G[StateSize*StateSize]; // control matrix
     float F[StateSize*StateSize];
     float FTran[StateSize*StateSize];
     float H[StateSize];
     float HTran[StateSize];
     float Q_meas[StateSize*StateSize]; 
     float R_n[ASize*ASize];


    
    for(int j = 0; j < ASize; j++)
    {
        A[j] = 1; //pre-allocating sensor data from each accelerometer
        sigma_std[j] = 1; // pre-allocating error (std) from each acelerometer
    }
        


     for(int i = 0; i < StateSize; i++)
     {
        U_n[i] = 1; 
        X_n_n_m_1[i] = 1; // predicted state

     }

     for(int i = 0; i < StateSize + ASize; i ++)
     {
        H[i] = 1; //observation matrix (conversions happen)
        HTran[i] = 1; //copy of H (for transpose
     }

    for(int i = 0; i < StateSize*2; i++)
    {
        P_n_n[i] = 1; // estimate uncertinity
        X_n_n[i] = 1; // state estimate
        P_n_n_m_1[i] = 1; // predicted error
        G[i] = 1; // control matrix
        F[i] = dt*dt - 1*2 + dt*dt; // state transition matrix
        FTran[i] = 1; // copy of state transion matrix copy to put in function
        Q_meas[i] = 1; //Process noise matrix
    }

     // another for loop for H

    
    //uint16_t RSize = ASize; 

    mul(sigma_std, sigma_std, R_n, 1, ASize, ASize); // okay issue why does it not like R_n, seems like it doesn't want a matrix



/*

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