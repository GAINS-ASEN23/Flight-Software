%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%            GAINS - CW Kalman Filter - ASEN 4018         %
%                                                         %
%                                                         %
%                     Jason Popich                        %
%                                                         %
%                      02/28/2023                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear
clc

%% Generate some dummy acceleration data

% Generate analytical truth data
t1 = 0;                     % Start time
t2 = 3600;                  % End time
n = 10000;                  % Number of data points
dt = (t2-t1)/n;             % Time step
t = linspace(t1, t2, n);    % n times from t1 to t1
[A,V,X] = IM_testdata(t);

%% Set the initial conditions

% The initial estimate state vector for the Accel KF
accel_x_n_n = [0; 0; 0; 0; 0; 0];

% The initial estimate uncertainty vector for the Accel KF
accel_p_n_n = [eye(3)*0.001 zeros(3); zeros(3) eye(3)*(1e-7)];

% The initial estimate state vector for the CW KF
x_n_n = [1000; 1000; 0; 0; 0; 0];

% The initial estimate uncertainty vector for the CW KF
p_n_n = [eye(3)*0.001 zeros(3); zeros(3) eye(3)*(1e-7)];

%% Run the Kalman Filter
state = [];
error = [];
for i = 1:length(A)
    % Get the acceleration data
    accel_M_n = [0; 0; 0; A(i); A(i); A(i)];
    
    % Set the acceleration measurement error
    accel_R_n = eye(3)*(1e-1);
    
    % KF the accel data
    [accel_x_n_n, accel_p_n_n] = KF_accel(accel_M_n, zeros(3,1), accel_x_n_n, accel_p_n_n, accel_R_n, dt);
    
    % Get the current Measurement
    % M_n = accel_x_n_n;
    M_n = zeros(6,1);
    
    % Get the current Input
    U_n = zeros(3,1);
    
    % Get the current Measurement Error
    % R_n = accel_p_n_n;
    R_n = eye(6)*1e10;
    
    % Run the KF equations for current step
    [x_n_n, p_n_n] = KF_cw(M_n, U_n, x_n_n, p_n_n, R_n);
    
    % Save State and Error
    state = [state; x_n_n'];
    error = [error; p_n_n];
end

%% Plot Results
figure;
title("Acceleration [m/s^2]")
plot(t, A)

figure;
title("Position (m)")
subplot(3,1,1)
plot(t, state(:,1));
ylabel("x (m)")

subplot(3,1,2)
plot(t, state(:,2));
ylabel("y (m)")

subplot(3,1,3)
plot(t, state(:,3));
ylabel("z (m)")

figure;
title("Velocity (m)")
subplot(3,1,1)
plot(t, state(:,4));
ylabel("${\dot{x}}$ (m)", 'interpreter', 'latex', 'FontWeight', 'bold')

subplot(3,1,2)
plot(t, state(:,5));
ylabel("${\dot{y}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')

subplot(3,1,3)
plot(t, state(:,6));
ylabel("${\dot{z}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')
