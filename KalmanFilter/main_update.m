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

%% Read in data
%Accel/gyro
addpath('C:\Users\Addi Woodard\OneDrive - UCB-O365\00_Sr Spring\ASEN 4018 - GAINS\3 - Flight Software\AccelGyroData');
load('accelgyrodata.mat');

%Startracker
starTracker_raw = csvread('StarTrackerSample.csv');
ST_X = starTracker_raw(:,1);
ST_Y = starTracker_raw(:,2);
ST_Z = starTracker_raw(:,3);
ST_VX = starTracker_raw(:,4);
ST_VY = starTracker_raw(:,5);
ST_VZ = starTracker_raw(:,6);
ST_B0 = starTracker_raw(:,7);
ST_B1 = starTracker_raw(:,8);
ST_B2 = starTracker_raw(:,9);
ST_B3 = starTracker_raw(:,10);

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
S.H = eye(6);

% Measurement noise covariance matrix (Assumed perfect at the moment, could
% use DSN, GPS to change)
S.R_n = zeros(6,1);

% Initial State and initial error 
S.x_n_n = zeros(6,1);
S.P_n_n = zeros(6,6);

%% Load in gyroscope, accelerometer, and star tracker data
% Fake data for rn purposes (load in from stk later)
%sampleSize = 20;
accel_raw = xyz5(1:end-5,:);%ones(sampleSize,3);
%accel_raw(:,1) = 2;
gyro_raw = gxyz5;%.01.*ones(sampleSize,3);
sampleSize = length(starTracker_raw);

% for loop to populate sample every 10 datapoints (1,11,21,ect) like this
%   b/c the first sample cannot be NaN
% starTracker_raw = NaN(sampleSize,4);
% for i = 1:10:sampleSize
%     %disp(i)
%     starTracker_raw(i,:) = [1,1,1,1];
% end


% set inital quat_prev (inital quaterion going into the tranformation
% function, Accel_Inertial)
quat_prev = starTracker_raw(1,7:10);

% check for correctness 
%{
for i = 1:sampleSize
    if isnan(starTracker_raw(i,1)) || isnan(starTracker_raw(i,2)) || isnan(starTracker_raw(i,3))
        disp(i)
        disp('NaN')
    end
end
%}

%%%%%% Begining of cyclical equations %%%%%%%% run at 100 Hz or more
% need to simulate for loop

% Taking inputs/ determing input acceleration (KAYLIE RE-DOUBLE PARANTHESIS
% THIS WHEN DONE TESTING FUNC)***************************

% PLACE HOLDER FOR LOOP FOR TESTING THE ACCEL INERIAL FUNCTION (WILL NEED
% TO CHANGE LATER)
for j = 1:sampleSize

    % Get z_n from ground station
    S.z_n = [1 0 0 1 1 1]; % filler (random)

    % Getting accel and gyro at each time step to go into Accel_Inertial
    %   funtion
    accel_body(j,:) = accel_raw(j,:);
    gyro(j,:) = gyro_raw(j,:);
    st(j,:) = starTracker_raw(j,7:10);
    
    
    % gyroscope/ star tracker control loop and accelerometer coordinate frame transformation
    %   send in accel_body, gryo, at 3x1 and st, quat_prev
    %[accel_inertial(j), quat_next(j)] = Accel_Inertial(accel_body(j,:), gyro(j,:), st(j,:), quat_prev(j,:));
    [accel_inertial(j,:),quat_next(j,:)] = Accel_Inertial(accel_body(j,:), gyro(j,:), st(j,:), quat_prev(j,:));
    % updatding quaternion to go back into function
    quat_prev = quat_next;
    

    
    % Noise mitigation
    [ax_bar(j), ay_bar(j), az_bar(j)] = AccelNoiseRed(accel_inertial(j,1),accel_inertial(j,2),accel_inertial(j,3)); % I should be able to do lines 78 and 79 in one step ya? it doesn't :(
    S.U_t(j,:)  = [ax_bar(j), ay_bar(j), az_bar(j)]; 
    
    
    %% Kalman Filter equations
    % Q: how does accel factor into kalman filter equations (don't think this
    % part has been correctly updated on flowchart)
    
    % Time Update
        % Extrapolate the state
        S.x_n_p_1_n(:,j) = S.F*S.x_n_n + S.G*S.U_t(j,:)'; % double check that this is the correct implementation of U_t (dimentions don't make sense)
        
        % Extrapolate uncertainty
        S.P_n_p_1_n = S.F*S.P_n_n*S.F' + S.Q;
        
    % Measurement Update
        
        % Compute the Kalman Gain
        S.K_n = S.P_n_p_1_n*S.H'*((S.H*S.P_n_p_1_n*S.H' + S.R_n)^-1);
        
        % Update the estimate with measurement
        S.x_n_n = S.x_n_p_1_n(:,j) + S.K_n*(S.z_n - S.H*S.x_n_p_1_n(:,j));
        
        % Get the size of H
        [H_rows, H_cols] = size(S.H);
        
        % Update the estimate uncertainty
        S.P_n_n1 = (eye(H_rows,H_cols) - S.K_n*S.H)*S.P_n_p_1_n*((eye(H_rows,H_cols) - S.K_n*S.H)') + (S.K_n*S.R_n)'*S.K_n';
    
        
        % Output state
        S.FinalState = [S.x_n_p_1_n(:,j) S.x_n_n];
end    
    
    
    
    

