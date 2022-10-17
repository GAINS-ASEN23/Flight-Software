%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%         GAINS - Simple Kalman Filter - ASEN 4018        %
%                                                         %
%                                                         %
%                     Jason Popich                        %
%                                                         %
%                      10/11/2022                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Last edited: Bennett Grow 10/14/22
%   Added IM to session path
%   Changed legend location

%% Set Environment
clear
clc
parent = fileparts(cd);
addpath(string(parent) + '\IntegrationMethods');

%% Analytical Data
t1 = 0;                     % Start time
t2 = 0.2;                   % End time
n = 400;                    % Number of data points
dt = (t2-t1)/n;             % Time step
t = linspace(t1, t2, n);    % n times from t1 to t1

% Generate analytical truth data
[A,V,X] = IM_testdata(t);

%% Simple Kinematics
[V_k, X_k] = IM_kinematic(A, V, X, dt);

%% Omega Arithmetic

% Sampling Frequency
fs = n/(t2-t1);

[V_o, X_o] = IM_omega(A,t,fs);

V_o = real(V_o(1:n));
X_o = real(X_o(1:n));

%% Kalman Filter
sigma_std = 13.892443989449804;
% sigma_std = 5;
P_n_n = [0.1 0 0; 0 1 0; 0 0 0];
state = KF_accel(A, sigma_std, P_n_n, dt);

%% Plot Data
figure;
tiled = tiledlayout(3,1);
xlab = xlabel(tiled,{'Time','$\mathbf{s}$'},'interpreter', 'latex');

nexttile
hold on
ylabel({'Acceleration','$\mathbf{m/s^{2}}$'},'interpreter', 'latex');
plot(t,A)
%plot(t,state(3,:))
%legend('Truth', 'KF')
hold off

nexttile
hold on
ylabel({'Velocity','$\mathbf{m/s}$'},'interpreter', 'latex');
plot(t,V)
plot(t,state(2,:))
plot(t,V_k)
plot(t, V_o);
legend('Truth', 'KF', 'Kinematics', 'Omega','Location','eastoutside')
hold off

nexttile
hold on
ylabel({'Position','$\mathbf{m}$'},'interpreter', 'latex');
plot(t,X)
plot(t,state(1,:))
plot(t,X_k)
% plot(t, X_o);
legend('Truth', 'KF', 'Kinematics', 'Omega','Location','eastoutside')
hold off