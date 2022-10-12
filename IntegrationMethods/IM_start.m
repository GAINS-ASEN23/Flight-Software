%{
Last Edited: Bennett Grow 10/5/22

Test various numerical integration techniques against analytical truth data
%}

clear all; close all; clc

%% Analytical Data
t1 = 0;                     % Start time
t2 = 0.2;                   % End time
n = 40000;                    % Number of data points
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

%% Adams-Bashforth


%% Unit Conversion
V = V .* 1E3;       % m/s -> mm/s
X = X .* 1E6;       % m -> um

V_k = V_k .* 1E3;
X_k = X_k .* 1E6;

V_o = V_o .* 1E3;
X_o = X_o .* 1E6;

V_o = real(V_o(1:n));
X_o = real(X_o(1:n));

%% Plotting
figure;
tiled = tiledlayout(3,1);
xlab = xlabel(tiled,{'Time','$\mathbf{s}$'},'interpreter', 'latex');

nexttile
hold on
ylabel({'Acceleration','$\mathbf{m/s^{2}}$'},'interpreter', 'latex');
plot(t,A)
yline(0)
hold off

nexttile
hold on
ylabel({'Velocity','$\mathbf{mm/s}$'},'interpreter', 'latex');
plot(t,V)
plot(t,V_k)
plot(t,V_o)
yline(0)
hold off

nexttile
hold on
ylabel({'Position','$\mathbf{\mu m}$'},'interpreter', 'latex');
plot(t,X)
plot(t,X_k)
plot(t,X_o)
yline(0)
hold off


%% Plotting 2

figure;
tiled = tiledlayout(3,1);
xlab = xlabel(tiled,{'Time','$\mathbf{s}$'},'interpreter', 'latex');

nexttile
hold on
ylabel({'Acceleration','$\mathbf{m/s^{2}}$'},'interpreter', 'latex');
plot(t,A)
%plot(t,ifft(ddXf))
yline(0)
hold off

nexttile
hold on
ylabel({'Velocity','$\mathbf{mm/s}$'},'interpreter', 'latex');
%plot(t,V)
%plot(t,V_k)
plot(t,V_o)
yline(0)
hold off

nexttile
hold on
ylabel({'Position','$\mathbf{\mu m}$'},'interpreter', 'latex');
%plot(t,X)
%plot(t,X_k)
plot(t,X_o)
yline(0)
hold off


