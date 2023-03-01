function [state, error] = KF_accel(M_n, U_n, x_n_n, P_n_n, R_n, dt)
%{
    Last Edited: Jason Popich 02/28/23

    DESCRIPTION

    Inputs:
        M_n - Measurement Vector, current measurement input
        U_n - Input Vector, measurable deterministic input to the system
        x_n_n - Current Estimated State Vector
        P_n_n - Current Estimated State Uncertainty Matrix
        R_n - Measurement Uncertainty Matrix
        dt - Length of time between steps [sec]

    Outputs:
        state - The next time step Estimated State Vector
        error - The next time step Estimated State Uncertainty Matrix
%}

    %% ---------------------------------------
    %        SET VARIABLES FOR ITERATION
    % ----------------------------------------
    
    % Define the State Transition Matrix
    F = [1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
    
    % Define the input matrix
    B = [zeros(3); eye(3)];
    
    % Define Control Matrix
    G = F*B;

    % Define the observation matrix
    H = [zeros(3) eye(3)*dt];
    
    % Set errors
    pos_sigma = 1;              % Position error in [m]
    vel_sigma = 1;               % Velocity error in [m/s]
    
    % Define the Measurement Process Noise Matrix, Q_meas
    Q_meas = [eye(3)*pos_sigma zeros(3); zeros(3) eye(3)*vel_sigma];

    % Define the Process Noise Matrix, Q
    Q = F*Q_meas*F';
    
    %% ---------------------------------------
    %           KALMAN FILTER
    % ----------------------------------------

    % Time Update

    % Extrapolate the state
    x_n_p_1_n = F*x_n_n + G*U_n;

    % Extrapolate uncertainty
    P_n_p_1_n = F*P_n_n*F' + Q;

    % Measurement Update

    % Compute the Kalman Gain
    K_n = P_n_p_1_n*H'*((H*P_n_p_1_n*H' + R_n)^-1);

    % Compute Measurement Equation
    z_n = H*M_n;

    % Update the estimate with measurement
    x_n_n = x_n_p_1_n + K_n*(z_n - H*x_n_p_1_n);

    % Get the size of H
    [H_rows, H_cols] = size(K_n*H);

    % Update the estimate uncertainty
    P_n_n = (eye(H_rows,H_cols) - K_n*H)*P_n_p_1_n*((eye(H_rows,H_cols) - K_n*H)')+K_n*R_n*K_n';

    % Output calculated values
    state = x_n_n;
    error = P_n_n;

end