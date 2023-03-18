function [state, error] = KF_cw(n, Q, M_n, U_n, x_n_n, P_n_n, R_n, dt, tnm1, tn, ground_data)
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
    
    % Define the Clohessy-Wiltshire variables
    mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
    a_moon = 1.74e6+5e4;                    % Semimajor axis of Moon's orbit around Earth [m]
    
    % Mean motion of the Moon around the Earth [rad/s] 
    n = sqrt(mu_moon/(a_moon^3));
    
    % Define A
    %A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*(n^2) 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0];

    % Define the State Transition Matrix
    %F = expm(A*t);
    F = [4-3*cos(n*dt) 0 0 sin(n*dt)/n -2*cos(n*dt)/n+2/n 0; 6*(sin(n*dt)-(n*dt)) 1 0 -2*(1-cos(n*dt))/n (4*sin(n*dt)-3*n*dt)/n 0; 0 0 cos(n*dt) 0 0 sin(n*dt)/n; 3*n*sin(n*dt) 0 0 cos(n*dt) 2*sin(n*dt) 0; -6*n*(1-cos(n*dt)) 0 0 -2*sin(n*dt) 4*cos(n*dt)-3 0; 0 0 -n*sin(n*dt) 0 0 cos(n*dt)];

    
    % Define the input matrix
    %B = [zeros(3); eye(3)];
    
    % Define Control Matrix
    %Gamma = @(n,t1,t2)reshape([t1.*-4.0+t2.*4.0+(sin(n.*(t1-t2)).*3.0)./n,(sin((n.*(t1-t2))./2.0).^2.*1.2e+1)./n-n.*t1.^2.*3.0-n.*t2.^2.*3.0+n.*t1.*t2.*6.0,0.0,sin((n.*(t1-t2))./2.0).^2.*6.0,sin(n.*(t1-t2)).*-6.0+n.*(t1-t2).*6.0,0.0,0.0,-t1+t2,0.0,0.0,0.0,0.0,0.0,0.0,-sin(n.*(t1-t2))./n,0.0,0.0,cos(n.*(t1-t2))-1.0,1.0./n.^2.*sin((n.*(t1-t2))./2.0).^2.*2.0,-1.0./n.^2.*(sin(n.*(t1-t2)).*2.0-n.*(t1.*2.0-t2.*2.0)),0.0,-sin(n.*(t1-t2))./n,(sin((n.*(t1-t2))./2.0).^2.*-4.0)./n,0.0,1.0./n.^2.*(sin(n.*(t1-t2)).*2.0-n.*(t1.*2.0-t2.*2.0)),t1.*t2.*3.0+1.0./n.^2.*sin((n.*(t1-t2))./2.0).^2.*8.0-t1.^2.*(3.0./2.0)-t2.^2.*(3.0./2.0),0.0,(sin((n.*(t1-t2))./2.0).^2.*4.0)./n,t1.*3.0-t2.*3.0-(sin(n.*(t1-t2)).*4.0)./n,0.0,0.0,0.0,1.0./n.^2.*sin((n.*(t1-t2))./2.0).^2.*2.0,0.0,0.0,-sin(n.*(t1-t2))./n],[6,6]);
    %G = Gamma(n, tnm1, tn)*B;
    G_func = @(dt,n,t1,t2)reshape([1.0./n.^2.*sin((dt.*n)./2.0).^2.*2.0,1.0./n.^2.*(sin(dt.*n).*2.0-dt.*n.*2.0),0.0,sin(dt.*n)./n,(sin((dt.*n)./2.0).^2.*-4.0)./n,0.0,-1.0./n.^2.*(sin(dt.*n).*2.0-dt.*n.*2.0),1.0./n.^2.*sin((dt.*n)./2.0).^2.*8.0+t1.*t2.*3.0-t1.^2.*(3.0./2.0)-t2.^2.*(3.0./2.0),0.0,(sin((dt.*n)./2.0).^2.*4.0)./n,dt.*-3.0+(sin(dt.*n).*4.0)./n,0.0,0.0,0.0,1.0./n.^2.*sin((dt.*n)./2.0).^2.*2.0,0.0,0.0,sin(dt.*n)./n],[6,3]);
    G = G_func(dt,n,tnm1,tn);

    % Define the observation matrix
    if (ground_data == true)
        H = eye(6);
    else
        H = zeros(6);
    end
    
    % Set errors
    pos_sigma = 1000;              % Position error in [km]
    vel_sigma = 1;               % Velocity error in [km/s]
    
    % Define the Measurement Process Noise Matrix, Q_meas
    % Q_meas = [eye(3)*pos_sigma zeros(3); zeros(3) eye(3)*vel_sigma];

    % Define the Process Noise Matrix, Q
    % Q = F*Q_meas*F';

    
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