%{
    bias.m
    Author -> Bennett Grow
    Last Modified -> 11/3/22

    Estimate accelerometer bias with multi-position static testing

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

%% Local gravitational constant

g_sl = 9.80665;             % [m/s^2] Acceleration at sea level
r_e =  6.3781 * 10^6;       % [m] Earth's radius
h = 1606;                   % [m] Local altitude
% https://www.advancedconverter.com/map-tools/find-altitude-by-coordinates
g = g_sl*(r_e/(r_e+h))^2;   % [m/s^2] Local acceleration

%% Aggarwal 2 Position Test

X = [1;0;0];
Y = [0;1;0];
Z = [0;0;1];

% Reference Accelerations
a1 =  g.*X;
a2 = -g.*X;
a3 =  g.*Y;
a4 = -g.*Y;
a5 =  g.*Z;
a6 = -g.*Z;
a = [a1,a2,a3,a4,a5,a6]; % (3x6)
A = [a;ones(1,6)]; % (Hayal 2.12) (4x6)

% Measured Accelerations
%   (Hayal 2.10) (3x6)
%   Random 3x6
%a(a==0) = 1
r = -0.1+(0.2)*rand(3,6);
U = a + r;



% Matrix of estimated accelerometer error parameters
M = U*A'*inv(A*A') % (Hayal 2.11) (3x4)



%% Unsal 8 Position Test

ang = 22.5;                         % [deg] Angle between test positions
gvec = g.*linspace(-1,1,180/ang);   % [m/s^2] 








