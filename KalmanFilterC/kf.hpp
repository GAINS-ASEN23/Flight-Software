/*
============================================================================
Name        : kf.h
Author      : Jason Popich, Bennett Grow, Kaylie Rick
Version     : 0.1
Copyright   : 
Description : This file contains the Kalman Filter (KF) function declarations
============================================================================
*/

#ifndef _KF_H_
#define _KF_H_

#include <stdlib.h>						        // Standard library
#include <stdint.h>						        // For uint8_t, uint16_t and uint16_t
extern "C" {
    #include "CControl/Headers/Functions.h"         // Matrix Math Library
}
#include "operations.hpp"                       // Some GAINS functions

class KalmanFilter
{
    public:
    /*  CLASS FUNCTIONS  */

    // Default Constructor (DO NOT USE)
    KalmanFilter(){};

    // Initialization Constructor
    KalmanFilter(int n_x, int n_u, int n_z, float* x_n_n, float* P_n_n);

    // Default Destructor
    ~KalmanFilter()
    {
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
    };

    /*  PUBLIC KF FUNCTIONS  */

    void KF_run(float t1, float t2, float n);
    void get_state();
    void get_uncertainty();

    void set_r_n(float sigma[]);
    void set_det_input_vector(float* u_n);
    void set_mea_input_vector(float* z_n);
    void set_q_a(float sigma[]);
    void set_h(bool ground_contact);
    void set_b();

    private:

    /*  PRIVATE VARIABLES   */

    float t1 = 0;                               // Sample Time begin
    float t2 = 0;                               // Sample Time end

    uint16_t n_u = 0;                           // Number of deterministic inputs (3 - accelerations x,y,z)
    uint16_t n_x = 0;                           // Number of States (x, y, z, x_dot, y_dot, z_dot)
    uint16_t n_z = 0;                           // Number of measured states (x_g, y_g, z_g, x_dot_g, y_dot_g, z_dot_g)

    // State and Uncertainty Matricies
    float *x_n_p_1_n = nullptr;                 // Current Predicted State vector
    float *P_n_p_1_n = nullptr;                 // Current Predicted Uncertainty Matrix

    float *P_n_n = nullptr;                     // Estimate Uncertainty Matrix
    float *x_n_n = nullptr;                     // Estimate State vector

    // Input Variables
    float *z_n = nullptr;                       // Measurement vector
    float *u_n = nullptr;                       // Deterministic input vector

    // Kalman Filter Matricies
    float *F = nullptr;                         // State Transition Matrix
    float *FT = nullptr;                        // Transposed State Transition Matrix

    float *G = nullptr;                         // Control Matrix
    float *B = nullptr;                         // Control Observation Matrix
    float *BT = nullptr;                        // Transposed Control Observation Matrix
    float *Gamma = nullptr;                     // Gamma Matrix (No other name??)
    float *GammaT = nullptr;                    // Gamma Matrix Transposed (No other name??)

    float *H = nullptr;                         // Observation Matrix
    float *HT = nullptr;                        // Transposed Observation Matrix

    float *Q = nullptr;                         // Process Noise Matrix
    float *Q_a = nullptr;                       // Input Noise Matrix
    float *R_n = nullptr;                       // Measurement Noise Covariance Matrix
    float *K_n = nullptr;                       // Kalman gain

    float *I_SS = nullptr;                      // Idenitity matrix (for K_n)

    /*  SUPPORT FUNCTIONS   */

    /****************************************************************************************/
    /*  update_constant_matrices                                                            */
    /*                                                                                      */
    /*  This function updates the F matrix, G matrix, and Gamma Matrix                      */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @Param   float n                  The mean motion constant                          */
    /*                                                                                      */
    /*  @Param   uint32_t t1              The start of the sample period                    */
    /*                                                                                      */
    /*  @Param   uint32_t t2              The end of the sample period                      */
    /*                                                                                      */
    /****************************************************************************************/
    void update_constant_matrices(float n);
    
    // COMMENT FUNCTION
    void compute_state_transition_matrix(float n);

    // COMMENT FUNCTION
    void compute_gamma_matrix(float n);

    // COMMENT FUNCTION
    void compute_process_noise_covariance_matrix();

    /* MAIN KALMAN FUNCTIONS */

    /****************************************************************************************/
    /*  predict_state                                                                       */
    /*                                                                                      */
    /*  This function predict the state at timestep n+1                                     */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @Param   float x_n_p_1_n[]              The x of n plus 1 state vector that gets    */
    /*                                              predicted                               */
    /*                                                                                      */
    /*  @Param   float F[]                      The state-transition matrix F               */
    /*                                                                                      */
    /*  @Param   float x_n_n[]                  The current estimate of the state           */
    /*                                                                                      */
    /*  @Param   float G[]                      The control matrix G                        */
    /*                                                                                      */
    /*  @Param   float u_n[]                    The deterministic input vector U_n          */
    /*                                                                                      */
    /****************************************************************************************/
    void predict_state();

    /****************************************************************************************/
    /*  predict_uncertainty                                                                 */
    /*                                                                                      */
    /*  This function predicts the uncertainty in the state prediction at the timestep n+1  */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @Param   float P_n_p_1_n[]              The P of n plus 1 prediction uncertainty    */
    /*                                              matrix that gets predicted              */
    /*                                                                                      */
    /*  @Param   float F[]                      The state-transition matrix F               */
    /*                                                                                      */
    /*  @Param   float FT[]                     The transposed state-transition matrix F    */
    /*                                                                                      */
    /*  @Param   float P_n_n[]                  The current estimate uncertainty            */
    /*                                                                                      */
    /*  @Param   float Q[]                      The process noise covariance matrix         */
    /*                                                                                      */
    /****************************************************************************************/
    void predict_uncertainty();

    /****************************************************************************************/
    /*  compute_kalman_gain                                                                 */
    /*                                                                                      */
    /*  This function calculates the kalman gain                                            */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @param   float K_n[]                    The Kalman Gain, K_n                        */
    /*                                                                                      */
    /*  @Param   float P_n_n_m_1[]              The P of n minus 1 prediction uncertainty   */
    /*                                              matrix                                  */
    /*                                                                                      */
    /*  @Param   float H[]                      The observation matrix, H                   */
    /*                                                                                      */
    /*  @Param   float R_n[]                    The measurement covariance matrix, R_n      */
    /*                                                                                      */
    /****************************************************************************************/
    void compute_kalman_gain();

    /****************************************************************************************/
    /*  estimate_state                                                                      */
    /*                                                                                      */
    /*  This function estimates the state at timestep n based off of measurements           */
    /*          (if measurements are included)                                              */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @param   float x_n_n[]                  The current state estimate, x_n             */
    /*                                                                                      */
    /*  @param   float x_n_p_1_n[]              The predicted state estimate, x_n_p_1_n     */
    /*                                                                                      */
    /*  @param   float K_n[]                    The Kalman Gain, K_n                        */
    /*                                                                                      */
    /*  @param   float z_n[]                    The Measurement Vecotr, z_n                 */
    /*                                                                                      */
    /*  @Param   float H[]                      The observation matrix, H                   */
    /*                                                                                      */
    /****************************************************************************************/
    void estimate_state();

    /****************************************************************************************/
    /*  estimate_uncertainty                                                                */
    /*                                                                                      */
    /*  This function estimates the uncertainty in the state estimate at timestep n         */
    /*                                                                                      */
    /*  Returns:    Nothing                                                                 */
    /*                                                                                      */
    /*  @Param   float P_n_n[]                  The current estimate uncertainty            */
    /*                                                                                      */
    /*  @param   float K_n[]                    The Kalman Gain, K_n                        */
    /*                                                                                      */
    /*  @Param   float p_n_n_m_1[]              The P of n minus 1 prediction uncertainty   */
    /*                                              matrix                                  */
    /*                                                                                      */
    /*  @Param   float H[]                      The observation matrix, H                   */
    /*                                                                                      */
    /*  @Param   float R_n[]                    The measurement covariance matrix, R_n      */
    /*                                                                                      */
    /****************************************************************************************/
    void estimate_uncertainty();

    protected:
};

#endif