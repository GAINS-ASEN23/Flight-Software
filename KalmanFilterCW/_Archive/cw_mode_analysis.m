%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%            GAINS - CW Modes Analysis - ASEN 4018        %
%                                                         %
%                                                         %
%                     Jason Popich                        %
%                                                         %
%                      03/07/2023                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear
clc


%% Set variables

% Define the Clohessy-Wiltshire variables
mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
a_moon = 1.74e6+5e4;                    % Semimajor axis of Moon's orbit around Earth [m]

% Mean motion of the Moon around the Sun [rad/s] 
n = sqrt(mu_moon/(a_moon^3));

% Set the State Space Model variables for the Clohessy-Wiltshire equations
A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*(n^2) 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0];
B = zeros(6);
C = eye(6);
D = zeros(6);

%% Start the analysis

% Get the State Space Model
cw_ss = ss(A, B, C, D);

% Get the eigenvalues of the Clohessy-Wiltshire Equations
[eig_basis, eig_diag] = eigs(A);

% Get the rank of the eigenbasis
eig_rank = rank(eig_basis);

% Set a non-zero initial condition to excite the different modes
%      N1 N2 N3 N4 N5 N6
x_0 = [1000 0 0 0 0 0; ... % x
       0 1000 0 0 0 0; ... % y
       0 0 1000 0 0 0; ... % z
       0 0 0 1000 0 0; ... % x_dot
       0 0 0 0 1000 0; ... % y_dot
       0 0 0 0 0 1000];    % z_dot
   
% Set the time frame [10s long simulation]
t = 0:0.1:10;

% Set an initial control input (zero for now)
% u = [delta_e delta_t]
u = zeros(length(t), 6);

% Simulate each modal space
y_1 = lsim(cw_ss, u, t, x_0(:,1));
y_2 = lsim(cw_ss, u, t, x_0(:,2));
y_3 = lsim(cw_ss, u, t, x_0(:,3));
y_4 = lsim(cw_ss, u, t, x_0(:,4));
y_5 = lsim(cw_ss, u, t, x_0(:,5));
y_6 = lsim(cw_ss, u, t, x_0(:,6));