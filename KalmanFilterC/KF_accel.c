// Include Libraries
#include "CControl/Headers/Functions.h"

int main()
{
    // ***** defining constants (incoming from accelerometers)*****
    // Acceleration data in [m/s^2]  QUESTION 1: This isn't going to be coming in directly in m/s^2, should we account for that for FLOPS sake and shouldn't populating the A matrix not be included??
    uint16_t ASize = 6;
    float A[ASize]; // does float take up more space then let's say float
    for(int i = 0; i < ASize; i++) // should I take populating it out for FLOPS sake (wouldn't be on flight software) wasn't sure if not populating it would be a problem
    {
        A[i] = 9.81;
    }

    // std (we determine for acceleroemter beforehand) QUESTION: this is for error associated in each accelerometer correct?
    float sigma_std[ASize];
    for(int i = 0; i < ASize; i++)
    {
        sigma_std[i] = 13;
    }


    // time step (100 hz) check value
    float dt = 1/100; //seconds

    // Pnn Would we populate this ourselves (I think yes so leave for flops sake)
    uint16_t StateSize = 25; // is this correct??
    float P_n_n[StateSize][StateSize];
    for(int i = 0; i < StateSize; i++) // should I take populating it out for FLOPS sake (wouldn't be on flight software) wasn't sure if not populating it would be a problem
    {
        for(int j = 0; j < StateSize; j++)
        {
            if(i == j)
            {
                P_n_n[i][j] = 2.23; // lol
            }
            else
            {
                P_n_n[i][j] = 0;
            }
        }
    }

    // state at time step n and definind U_n(check size) Is there more involved with U
    // QUESTION: Should I be coming for loops like the one above and below could be combined (would that help with FLOPS)
    float X_n_n[StateSize]; // also known as state
    float U_n[StateSize];
    for(int i = 0; i < StateSize; i++) // same question as ealier here but what calculations are actually involved in our code (I think this stays to some extent, co7uld even need more)
    {
        if(i < ASize)
        {
            X_n_n[i] = A[i];
        }
        else
        {
            X_n_n[i] = 0;
        }

        U_n[i] = 0;
    }

    //prealocating some vectors **again could combinme??
    float X_n_n_m_1[StateSize];
    float P_n_n_m_1[StateSize][StateSize];
    float G[StateSize][StateSize]; // control matrix
    for(int i = 0; i < StateSize; i++)
    {
        for(int j = 0; j < StateSize; j++)
        {
            P_n_n_m_1[i][j] = 0;
            G[i][j] = 0;
        }

        X_n_n_m_1[i] = 0;
        
    }

    // Defining State Transition matrix 
    // QUESTION: does the contents of this affect the flops (I think yes) as in having x*t^2 +2 is more flops then just defining it as 1
    float F[StateSize][StateSize];
    float FTran[StateSize][StateSize];
    for(int i = 0; i < StateSize; i++)
    {
        for(int j = 0; j < StateSize; j++)
        {
            F[i][j] = dt*dt; // just did this to get more flops (could fill in different estimate)
            FTran[i][j] = dt*dt;
        }
    }
    // get F transpose from F
    tran(FTran, StateSize, StateSize);


    // Observation matrix
    float H[StateSize];
    float HTran[StateSize];
    for(int i = 0; i < StateSize; i++) // same question as ealier here but what calculations are actually involved in our code (I think this stays to some extent, co7uld even need more)
    {
        if(i < ASize)
        {
            H[i] = 1; // assuming no scale factor
            HTran[i] = 1;
        }
        else
        {
            H[i] = 0;
            HTran[i] = 0;
        }
    }
    tran(HTran, 1, StateSize);

    // Measurement uncertiniy matrix
    //uint16_t RSize = ASize; 
    float R_n[ASize][ASize];
    
     

    mul(sigma_std, sigma_std, R_n, 1, ASize, ASize); // okay issue why does it not like R_n, seems like it doesn't want a matrix


    // Defnining process noise matrix
    float Q_meas[StateSize][StateSize]; // QUESTION/ISSUE: Size doesn't make sense here when mulitpied by sigma_std ** for now just made 0's of size but not mujltiplied by  
    for(int i = 0; i < StateSize; i++)
    {
        for(int j = 0; j < StateSize; j++)
        {
            Q_meas[i][j] = 0; // just did this to get more flops (could fill in different estimate)
        }
    }


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



    








}


// notes (main changes from matlab version)
// 1) A is a 6 by 6 at one specific time step