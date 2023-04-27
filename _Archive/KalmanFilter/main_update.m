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
%addpath('C:\Users\Addi Woodard\OneDrive - UCB-O365\00_Sr Spring\ASEN 4018 - GAINS\3 - Flight Software\AccelGyroData');
load('2.7.23-35min-100Hz-accelGyro.mat');

%Startracker - TRIMMED to first 500 points
starTracker_raw = csvread('StarTrackerSample.csv');
ST_X = starTracker_raw(1:500,1);
ST_Y = starTracker_raw(1:500,2);
ST_Z = starTracker_raw(1:500,3);
ST_VX = starTracker_raw(1:500,4);
ST_VY = starTracker_raw(1:500,5);
ST_VZ = starTracker_raw(1:500,6);
ST_B0 = starTracker_raw(1:500,7);
ST_B1 = starTracker_raw(1:500,8);
ST_B2 = starTracker_raw(1:500,9);
ST_B3 = starTracker_raw(1:500,10);

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

% Clohessy-Wiltshire equations form A matrix
S.A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*S.n^2 0 0 0 2*S.n 0; 0 0 0 -2*S.n 0 0; 0 0 -S.n^2 0 0 0];

% B matrix for control input filtering
S.B = [zeros(3); eye(3)];

% State transistion Matrix
S.F = expm(S.A);
% Need to add dt above?


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
accel_raw = timetable2table(Acceleration(1:500,:)); %5 points longer than gyro data if using all of it! %(Kaylie) coverted from timetable because causing issues (do we want/need it in time table?)
accel_raw = table2array(accel_raw(:,2:4));
gyro_raw = timetable2table(AngularVelocity(1:500,:));
gyro_raw = table2array(gyro_raw(:,2:4));
sampleSize = length(ST_X);

%%% STILL WORKING ON PADDING - AW

% for loop to populate sample every 10 datapoints (1,11,21,ect) like this
%   b/c the first sample cannot be NaN
% starTracker_raw = NaN(sampleSize,4);
% for i = 1:10:sampleSize
%     %disp(i)
%     starTracker_raw(i,:) = [1,1,1,1];
% end


% set inital quat_prev (inital quaterion going into the tranformation
% function, Accel_Inertial)
%Set initial quaternion
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
tic
for j = 1:sampleSize

    % Get z_n from ground station
    S.z_n = [0 0 0 0 0 0]; % 0's should mean getting nothing
    %If we don't get data, these are zeros??


    % Getting accel and gyro at each time step to go into Accel_Inertial
    %   funtion
    accel_body(j,:) = accel_raw(j,:);
    gyro(j,:) = gyro_raw(j,:);
    st(j,:) = starTracker_raw(j,7:10);
    
    
    % gyroscope/ star tracker control loop and accelerometer coordinate frame transformation
    %   send in accel_body, gryo, at 3x1 and st, quat_prev
    %[accel_inertial(j), quat_next(j)] = Accel_Inertial(accel_body(j,:), gyro(j,:), st(j,:), quat_prev(j,:));
    [accel_inertial(j,:),quat_next(j,:)] = Accel_Inertial(accel_body(j,:), gyro(j,:), st(j,:), quat_prev(j,:));
    % updating quaternion to go back into function
    quat_prev(j+1,:) = quat_next(j,:);
    

    
    % Noise mitigation
    %[ax_bar(j), ay_bar(j), az_bar(j)] = AccelNoiseRed(accel_inertial(j,1),accel_inertial(j,2),accel_inertial(j,3)); % I should be able to do lines 78 and 79 in one step ya? it doesn't :(
    ax_bar(j) = accel_inertial(j,1);
    ay_bar(j) = accel_inertial(j,2);
    az_bar(j) = accel_inertial(j,3);
    S.U_t(j,:)  = [ax_bar(j), ay_bar(j), az_bar(j)]; 
    
    
    %% Kalman Filter equations
    
    % Time Update
        % Extrapolate the state
        if j == 1 %ALWAYS start with update from the ground (z_n == x_n_n(1))
            S.x_n_p_1_n(:,j) = S.F*S.z_n' + S.G*S.U_t(j,:)'; % double check that this is the correct implementation of U_t (dimentions don't make sense)
        else
            S.x_n_p_1_n(:,j) = S.F*S.x_n_n(:,j-1) + S.G*S.U_t(j,:)';
        end
        % Extrapolate uncertainty
        S.P_n_p_1_n = S.F*S.P_n_n*S.F' + S.Q;
        
    % Measurement Update
        
        % Compute the Kalman Gain
        S.K_n = S.P_n_p_1_n*S.H'*((S.H*S.P_n_p_1_n*S.H' + S.R_n)^-1);
        
        % Update the estimate with measurement
        S.x_n_n(:,j) = S.x_n_p_1_n(:,j) + S.K_n*(S.z_n' - S.H*S.x_n_p_1_n(:,j));
        
        % Get the size of H
        [H_rows, H_cols] = size(S.H);
        
        % Update the estimate uncertainty
        S.P_n_n1 = (eye(H_rows,H_cols) - S.K_n*S.H)*S.P_n_p_1_n*((eye(H_rows,H_cols) - S.K_n*S.H)') + (S.K_n*S.R_n)'*S.K_n';
    
        
        % Output state [current state (6x1);next state(6x1)]
        S.FinalState(:,j) = [S.x_n_n(:,j);S.x_n_p_1_n(:,j)];
end    
toc
    
%Plotting   
figure
plot(1:length(S.FinalState),S.FinalState(1,:),'.b');
hold on;
plot(1:length(S.FinalState),S.FinalState(2,:),'.g')
plot(1:length(S.FinalState),S.FinalState(3,:),'.r')
title('Position vs Time');

figure
plot(1:length(S.FinalState),S.FinalState(4,:),'.b');
hold on;
plot(1:length(S.FinalState),S.FinalState(5,:),'.g')
plot(1:length(S.FinalState),S.FinalState(6,:),'.r')
title('Velocity vs Time');


%Provisions added for how long it takes to run 500 points (tic-toc)
%Next, cut data to 30 mins to check it
