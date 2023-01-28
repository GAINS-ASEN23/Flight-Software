%{
University of Colorado Boulder Aerospace Engineering Sciences
Senior Design Project 2022/2023
General Atomics Inertial Navigation System [GAINS]

main.m
Main script to run and test the GAINS Kalman Filter in Matlab

AccelNoiseRed.m
Uses accelerometer characteristics derived in testing to reducuce error in
the 


Bennett Grow



%}

% Housekeeping
clc; close all; clear all;


%% Defining Parameters
% Setting up struct
S = struct;                     % Initialize struct S to hold all relevant information for KF
S.timeStep = 1/100;             % [sec] Elapsed time between KF cycles
S.endTime = 10;                 % [sec] End time of KF
S.t = 0:S.timeStep:S.endTime;   % [sec] Vector of times from 0 to S.endTime with S.timeStep steps
S.tLen = length(S.t);           % Number of times in S.t

% Mean motion of the spacecraft
mu_moon = 4.9048695e12;                 % [m^3 s^-2] Gravitational parameter of the Moon
a_moon = 1.74e6+5e4;                    % [m] Semimajor axis of Moon's orbit around Earth
S.n = sqrt(mu_moon/(a_moon^3));         % [rad/s] Mean motion of the Moon around the Sun (max values for a used)

% P = 29.530 * 24 * 3600; % [sec] Orbital period of the Moon around the Sun
% S.n = 2*pi/P;           % [rad/s] Mean motion of the Moon around the Sun


% Clohessy-Wiltshire equations form A matrix
S.A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*S.n^2 0 0 0 2*S.n 0; 0 0 0 -2*S.n 0 0; 0 0 -S.n^2 0 0 0];

% B matrix for control input filtering
S.B = [zeros(3); eye(3)];

% State transistion Matrix
S.F = expm(S.A);

% Accelerometer uncertanties
S.sigma1 = 0.1;
S.sigma2 = 0.1;
S.sigma3 = 0.1;

S.Q_a = zeros(6);
S.Q_a(4,4) = S.sigma1;
S.Q_a(5,5) = S.sigma2;
S.Q_a(6,6) = S.sigma3;

% Process noise covariance
S.Q = S.F*S.Q_a*S.F';

S.G = S.F*S.B;

% Observatation matrix (update with any needed unit converstion)
S.H = eye(3);

% Measurement noise covariance matrix (Assumed perfect at the moment, could
% use DSN, GPS to change)
S.R_n = zeros(6,6);

% Initial State and initial error 
S.x_n_n = zeros(6,6);
S.P_n_n = zeros(6,6);

%%%%%% Begining of cyclical equations %%%%%%%% run at 100 Hz or more
% need to simulate for loop

%% Taking inputs/ determing input acceleration
% Get z_n from ground station
S.z_n = [1 0 0 1 1 1]; % filler (random)

% Simulate raw data
accel_raw = [1 1 1]; % temporary filler values (m/s^2)


% gyroscope/ star tracker control loop

% Accelerometer coordinate frame transformation

% Noise mitigation
[ax_bar, ay_bar, az_bar] = AccelNoiseRed(1,1,1); % I should be able to do lines 78 and 79 in one step ya? it doesn't :(
accel_mit = [ax_bar, ay_bar, az_bar]; 
S.U_t = [0 0 0 accel_mit]; % double check that this is the correct implementation of U_t (dimentions don't make sense)

%{ 
comented out becuase there's some issues with dimentions (specifically
%U_t)

%% Kalman Filter equations
% Q: how does accel factor into kalman filter equations (don't think this
% part has been correctly updated on flowchart)

% Time Update
    % Extrapolate the state
    S.x_n_p_1_n = S.F*S.x_n_n + S.G*S.U_t; % double check that this is the correct implementation of U_t (dimentions don't make sense)
    
    % Extrapolate uncertainty
    S.P_n_p_1_n = S.F*S.P_n_n*F' + S.Q;
    
% Measurement Update
    
    % Compute the Kalman Gain
    S.K_n = S.P_n_p_1_n*S.H'*((S.H*S.P_n_p_1_n*S.H' + S.R_n)^-1);
    
    % Update the estimate with measurement
    S.x_n_n = S.x_n_p_1_n + S.K_n*(S.z_n - S.H*S.x_n_p_1_n);
    
    % Get the size of H
    [H_rows, H_cols] = size(S.H);
    
    % Update the estimate uncertainty
    S.P_n_n = (eye(H_rows,H_cols) - S.K_n*S.H)*S.P_n_p_1_n*((eye(H_rows,H_cols) - S.K_n*H)')+S.K_n*S.R_n*S.K_n';
    
    % Output state
    state = [state S.x_n_n];

%}




