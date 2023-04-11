%{
    bias.m
    Author -> Bennett Grow
    Last Modified -> 11/3/22

    Estimate accelerometer bias with 8-position single-axis static testing

    References:
        - Estimation of Deterministic and Stochastic IMU Error Parameters
            By: Unsal and Demirbas, DOI: 978-1-4673-0387-3/12/

        - A Standard Testing and Calibration Procedure for Low Cost MEMS 
          Inertial Sensors and Units
            By: Aggarwal et al., DOI: 10.1017/S0373463307004560

        - Static Calibration of the Tactical Grade Inertial Measurement
          Units By: Hayal

    IMU is turned about z axis when x axis multi position static
    test is performed. Similarly, IMU is turned around x axis when
    y axis multi position static test is performed and, IMU is turned
    about/around y axis when z axis multi position static test is
    performed.

    Processing steps:
    1. Decomposing position states and angular rates from test
       data
    2. Taking the average value of each position and rate.
    3. Creating the true and measurement matrices for least
       square fitting. (Each measurement has equal weight to
       calculate calibration parameters)
    4. Implementing least squares fitting and calculating error
       (calibration) parameters
%}

clear all; close all; clc;

%% Load and Average Data
accels = zeros(1,9);
for i = 1:9
    d = readmatrix(strcat('GAINS 8 Pos Test/GAINS000', string(i), '/accel.csv'));
    accels(i) = mean(d(:,2));
end

%% Local Gravity

g_sl = 9.80665;             % [m/s^2] Acceleration at sea level
r_e =  6.3781 * 10^6;       % [m] Earth's radius
h = 1606;                   % [m] Local altitude
% https://www.advancedconverter.com/map-tools/find-altitude-by-coordinates
g = g_sl*(r_e/(r_e+h))^2;   % [m/s^2] Local acceleration
g_conv = g/g_sl;            % [g's]

%% Reference Gravity
ang = linspace(-180,0,9);   % [deg] 
ref = g_conv.*cosd(ang);   %[g's]

%% Single Axis
B = mean(accels);       % (Hayal 2.5)
K = g_conv;

% S = (sum(abs(accels)) - sum(abs(gvec))) / (length(accels)*K);
%accels2 = [accels(1:4) accels(6:`d9)];
%gvec2 = [gvec(1:4) gvec(6:9)];

% SF = (sum(abs(accels)) - sum(abs(ref))) / sum(abs(ref));      
S = (accels(end) - accels(1) - 2*K)/(2*K);                      % (Hayal 2.6)

fprintf("Single Axis Bias: %.8f   Scale Factor: %.8f \n", B, S)

ref
accels

adj_ref = ref*(1+S)+B

error_percentage = abs(abs(adj_ref)-abs(accels))./abs(accels).*100

%% Three Axis
%{
% Reference Accelerations
% (Hayal 2.12) (4x6)

ax = [1;0;0];
ay = [0;1;0];
az = [0;0;1];

a1 = gvec(1).*ax;
a2 = gvec(end).*ax;
a3 = gvec(1).*ay;
a4 = gvec(end).*ay;
a5 = gvec(1).*az;
a6 = gvec(end).*az;
a = [a1 a2 a3 a4 a5 a6];
A = [a;ones(1,6)]; 

% Measured Accelerations
%   (Hayal 2.10) (3x6)
m1 = accels(1).*ax;
m2 = accels(end).*ax;
m3 = accels(1).*ay;
m4 = accels(end).*ay;
m5 = accels(1).*az;
m6 = accels(end).*az;
M = [m1 m2 m3 m4 m5 m6];

% Matrix of estimated accelerometer error parameters
X = M * A' * (A * A')^(-1) % (Hayal 2.11) (3x4)


% 
% % Reference Accelerations
% % (Hayal 2.12) (4x6)
% a =  gvec.*[1;0;0];
% A = [a;ones(1,9)]; 
% 
% % Measured Accelerations
% %   (Hayal 2.10) (3x6)
% Y = accels.*[1;0;0];
% 
% % Matrix of estimated accelerometer error parameters
% X = Y * A' * (A * A')^(-1) % (Hayal 2.11) (3x4)



%}








