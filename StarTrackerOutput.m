%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%                    Star-Tracker Data Emulation
%                          Derek J. Popich
%                               2023
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
clc
clear
close all

%% Variables
% Constants
G = 6.67*(10^-11);                                                                                      % Gravitational Constant - m^3/kg-s^2
M_m = 7.34*(10^22);                                                                                     % Mass of Moon - Kg
MU = (G*M_m);
Rm = 1737*1000;                                                                                         % Moon Radius - m
t_0 = 0;                                                                                                % Initial Time - Seconds

% Orbital Elements
a = 50000+Rm;                                                                                           % Semi-Major Axis - m
e = 1*10^(-5);                                                                                          % Eccentricity
p = a*(1-e^2);                                                                                          % Semi-Latus Rectum - m
i = 0;                                                                                                  % Inclination - Radians
w = 0;                                                                                                  % Argument of Periapsis - Radians
W = 0;                                                                                                  % Ascending Node - Radians
f0 = 0;                                                                                                 % True Anomaly - Radians

T = 2*pi*sqrt(a^3/MU);                                                                                  % Period of Spacecraft - Seconds
Vcirc = sqrt(MU/a);                                                                                     % Circular Velocity of S/C - m/s

%% Orbit Simulation
% Simulation Constants
dt = 0.01;                                                                                              % Time Step - Seconds
t = 1:dt:60*60*3;                                                                                       % Create time Vector of Spacecraft Orbit

newOE = [a e i W w f0];                                                                                 % Vector for ODE45 Setup Variables
for n = 1:length(t)                                                                                     % Run ODE45 Simulations
    % Create Position Vectors and Velocity Vectors in r_sc
    r_sc(n,:) = (propagateState(newOE,t(n),t_0, MU))';
end

%% Hill Frame Calculation
% NOTE: Circular Orbit Velocity is CONSTANT-ish
% Hill Frame: {Or, Otheta, Oh}
for j = 1:length(r_sc)
    % Create Hill Frame Vectors
    Or(j,:) = [r_sc(j,1)/norm(r_sc(j,1:3)) r_sc(j,2)/norm(r_sc(j,1:3)) r_sc(j,3)/norm(r_sc(j,1:3))];
    Oh(j,:) = (cross(r_sc(j,1:3),r_sc(j,4:6)))./norm(cross(r_sc(j,1:3),r_sc(j,4:6)));
    Otheta(j,:) = cross(Oh(j,:), Or(j,:));
end

%% Quaternion Calculation
for k = 1:length(r_sc)
    % Create Rotation Matrix
    R = [Or(k,:); Otheta(k,:); Oh(k,:)];                                                                % Rotation Matrix - Hill -> ECI
    quat(k,:) = rotm2quat(R);                                                                           % Quaternions from Rotation Matrix
end

%% CSV Output
Output = [r_sc(:,:), quat(:,:)];
writematrix(Output,'StarTrackerSample.csv')
toc
function x = propagateState(oe0,t,t_0,MU)
% DESCRIPTION: Computes the propagated position and velocity in m, m/s
%              
%
% INPUTS:
% oe0       Orbit elements [a,e,i,Om,om,f] at time t0 (m,s,rad)
% t_0       Time at the initial epoch (s)
% MU        Central body's gravitational constant (m^3/s^2)

% OUTPUTS:
% x         Position and velocity vectors of the form [r; rdot] at
%           time t

% Orbital Elements
a = oe0(:,1);                                                                    % Semi-Major Axis - m
e = oe0(:,2);                                                                    % Eccentricity
p = a*(1-e^2);                                                                   % Semi-Latus Rectum - m
i = oe0(:,3);                                                                    % Inclination - Radians
w = oe0(:,5);                                                                    % Argument of Periapsis - Radians
W = oe0(:,4);                                                                    % Ascending Node - Radians
f0 = oe0(:,6);                                                                   % True Anomaly - Radians

%% Run Newtons Method
tol = 1*(10^-12);

% Compute needed values
n = sqrt(MU/(a^3));
M = n*t;

% Initial Guess
E = M;

% First Computation
Eg = E - (E-e*sin(E)-M)/(1-e*cos(E));
f = f0;

% Newtons Method Loop
while ( abs(E-Eg) > tol )
    E = Eg;
    Eg = E - (E-e*sin(E)-M)/(1-e*cos(E));
    
    dE = abs(E-Eg);
    f = 2*atan(sqrt((1+e)/(1-e))*tan(Eg/2))+f0;
end

%% Create Vectors for Plotting
% Vector Computations
r = p/(1+e*cos(f));
Pr = [r*cos(f);r*sin(f);0];
PrDot = sqrt(MU/p).*[-sin(f);e+cos(f);0];

% DCM Conversions
PN = angle2dcm(W,i,w,'ZXZ');
NP = PN';

Nr = (NP*Pr)';
NrDot = (NP*PrDot)';

%% Output Vectors
Nr = [Nr(:,1) Nr(:,2) Nr(:,3)];
NrDot = [NrDot(:,1) NrDot(:,2) NrDot(:,3)];
x = [Nr NrDot]';
end
