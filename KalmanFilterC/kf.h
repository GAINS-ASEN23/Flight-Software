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

#include <stdlib.h>						    // Standard library
#include <stdint.h>						    // For uint8_t, uint16_t and uint16_t
#include "CControl/Headers/Functions.h"     // Matrix Math Library
#include "operations.h"                     // Some GAINS functions

/* SUPPORT FUNCTIONS */

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
void update_constant_matrices(uint8_t state_size, uint8_t input_num, float F[], float FT[], float G[], float Gamma[], float B[], float n, float t1, float t2);


void compute_state_transition_matrix(float F[], float n, float t1, float t2);
void compute_gamma_matrix(float Gamma[], float n, float t1, float t2);
void compute_process_noise_covariance_matrix(uint8_t state_size, uint8_t input_num, float Q[], float B[], float BT[], float Gamma[], float GammaT[], float Q_a[]);


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
void predict_state(uint8_t state_size, float x_n_p_1_n[], float F[], float x_n_n[], float G[], float u_n[]);

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
void predict_uncertainty(uint8_t state_size, float P_n_p_1_n[], float F[], float FT[], float P_n_n[], float Q[]);

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
void compute_kalman_gain(uint8_t state_size, uint8_t input_num, float K_n[], float P_n_n_m_1[], float H[], float R_n[]);

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
void estimate_state(uint8_t state_size, uint8_t input_num, float x_n_n[], float x_n_p_1_n[], float K_n[], float z_n[], float H[]);

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
void estimate_uncertainty(uint8_t state_size, uint8_t input_num, float P_n_n[], float K_n[], float P_n_n_m_1[], float H[], float R_n[], float I_SS[]);


#endif