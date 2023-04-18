function [state, error] = KF_accel(Qa, M_n, U_n, x_n_n, P_n_n, R_n, dt, tnm1, tn, ground_data)
%{
    Last Edited: Bennett Grow 3/17/23

    DESCRIPTION

    Inputs:
        M_n - Measurement Vector, current measurement input
        U_n - Input Vector, measurable deterministic input to the system
        x_n_n - Current Estimated State Vector
        P_n_n - Current Estimated State Uncertainty Matrix
        R_n - Measurement Uncertainty Matrix
        bool_accel - Acceleration Flag to change the H Matrix

    Outputs:
        state - The next time step Estimated State Vector
        error - The next time step Estimated State Uncertainty Matrix
%}

    %% ---------------------------------------
    %        SET VARIABLES FOR ITERATION
    % ----------------------------------------
    % Define the State Transition Matrix
    F = [1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
    
    % Define Control Matrix
    G = [(1/2)*(dt^2) 0 0; 0 (1/2)*(dt^2) 0; 0 0 (1/2)*(dt^2); dt 0 0; 0 dt 0; 0 0 dt];

    % Define the observation matrix
    if (ground_data == true)
        H = eye(6);
    else
        H = zeros(6);
    end
    
    % Process noise covariance matrix
    if(U_n == zeros(3,1))
        Q = zeros(6);

    else % Thrusting
        Q = G*Qa*G';
    end

    %% ---------------------------------------
    %           KALMAN FILTER
    % ----------------------------------------

    % Extrapolate uncertainty
    P_n_p_1_n = F*P_n_n*F' + Q;

    % Measurement Update

    % Compute the Kalman Gain
    K_n = P_n_p_1_n*H'*((H*P_n_p_1_n*H' + R_n)^-1);

    % Compute Measurement Equation
    z_n = H*M_n;

    % Update the estimate with measurement
    x_n_n = x_n_n + K_n*(z_n - H*x_n_n);

    % Extrapolate the state
    x_n_n = F*x_n_n + G*U_n;

    % Get the size of H
    [H_rows, H_cols] = size(K_n*H);

    % Update the estimate uncertainty
    P_n_n = (eye(H_rows,H_cols) - K_n*H)*P_n_p_1_n*((eye(H_rows,H_cols) - K_n*H)')+K_n*R_n*K_n';

    % Output calculated values
    state = x_n_n;
    error = P_n_n;

end