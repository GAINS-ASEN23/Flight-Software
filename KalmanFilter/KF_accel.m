function [state] = KF_accel(A, sigma_std, P_n_n, dt)
%{
    Last Edited: Jason Popich 10/11/22

    Given an acceleration measurement vector, the function returns the state vector at every
        point in time run through a Kalman Filter

    NOTE: This function is not a generic Kalman Filter. Some values are hardcoded for quantification of system. 
            Make a different function for generic functionality.

    Inputs:
        A - Acceleration data [m/s^2] - double vector 
        dt - Length of time between steps [sec] - double

    Outputs:
        X_kf - The State Vector at every point in time
%}

    %% Set Initial Conditions
    x_n_n = [0; 0; A(1)];                                           % The estimated system state vector at time step n
    state = x_n_n;
    U_n = [0; 0; 0];                                                % The input variable, measurable deterministic input to the system

    % Preallocate vectors
    X_n_n_m_1 = zeros(3,1);                                         % The predicated system state vector at time step n-1
    P_n_n_m_1 = zeros(3,3);                                         % The prior estimate uncertainty (covariance) matrix of the current state (predicted at previous state)

    % Define the State Transition Matrix
    F = [1 dt (1/2)*(dt^2); 0 1 dt; 0 0 1];

    % Define Control Matrix
    % NOTE: Zero because no inputs
    G = zeros(3,3);

    % Define the observation matrix
    % In the state vector only state that is measured is acceleration
    H = [0 0 1];

    % Define the Measurement Uncertainty Matrix, R_n
    R_n = sigma_std^2;

    % Define the Measurement Process Noise Matrix, Q_meas
    % NOTE: Only measurement process error is acceleration
    Q_meas = [0 0 0; 0 0 0; 0 0 1]*(sigma_std^2);

    % Define the Process Noise Matrix, Q
    Q = F*Q_meas*F';

    % Determine the initial uncertainty of a prediction - covariance matrix for the next state
    % NOTE: Since this is the initial condition, estimate is 1
    % P_n_n = eye(3,3);

    % Calculate velocities and positions using 1D kinematic equations
    for i = 2:length(A)
        %% Time Update

        % Extrapolate the state
        x_n_p_1_n = F*x_n_n + G*U_n;

        % Extrapolate uncertainty
        P_n_p_1_n = F*P_n_n*F' + Q;

        %% Measurement Update
        
        % Compute the Kalman Gain
        K_n = P_n_p_1_n*H'*((H*P_n_p_1_n*H' + R_n)^-1);

        % Compute Measurement Equation
        z_n = H*[0; 0; A(i)];

        % Update the estimate with measurement
        x_n_n = x_n_p_1_n + K_n*(z_n - H*x_n_p_1_n);

        % Get the size of H
        [H_rows, H_cols] = size(H);

        % Update the estimate uncertainty
        P_n_n = (eye(H_rows,H_cols) - K_n*H)*P_n_p_1_n*((eye(H_rows,H_cols) - K_n*H)')+K_n*R_n*K_n';

        % Output state
        state = [state x_n_n];
    end

end