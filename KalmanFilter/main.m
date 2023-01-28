%{
University of Colorado Boulder Aerospace Engineering Sciences
Senior Design Project 2022/2023
General Atomics Inertial Navigation System [GAINS]

main.m
Main script to run and test the GAINS Kalman Filter in Matlab


Bennett Grow



%}

clc; close all; clear all;

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




