%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%            GAINS - CW Kalman Filter - ASEN 4018         %
%                                                         %
%                                                         %
%              Jason Popich & Bennett Grow                %
%                                                         %
%                      03/28/2023                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear;
clc;
close all;

%% Sample Data
sampledata = readmatrix("StarTrackerSample.csv");

n = 20000;                  % Number of data points
dt = 0.01;                  % Time step
t1 = 0;                     % Start time
t2 = n*dt + t1;             % End time
t = linspace(t1, t2, n);    % n times from t1 to t1

%% Set the initial conditions

% The initial estimate state vector for the CW KF
x_n_n = sampledata(1,1:6)';

% The initial estimate uncertainty vector for the CW KF
p_n_n = [eye(3)*0.1 zeros(3); zeros(3) eye(3)*(1e-5)];

%% Run the Kalman Filter
state = [];
error = [];

for i = 1:n

    % Get the current Measurement
    % M_n = accel_x_n_n;
    M_n = zeros(6,1);
    
    % Get the current Input
    U_n = zeros(3,1);
    
    % Get the current Measurement Error
    % R_n = accel_p_n_n;
    R_n = eye(6)*1e10;
    
    % Run the KF equations for current step
    [x_n_n, p_n_n] = KF_cw(M_n, U_n, x_n_n, p_n_n, R_n, dt);
    
    % Save State and Error
    state = [state; x_n_n'];
    error = [error; p_n_n];

end

%% Plot Results
plot_pos_vel = true;
plot3_pos = true;


if plot_pos_vel == true

    % figure;
    % title("Acceleration [m/s^2]")
    % plot(t, A)
    
    figure;
    title("Position (m)")
    subplot(3,1,1)
    plot(t, state(:,1));
    hold on;
    plot(t, sampledata(1:n,1));
    ylabel("x (m)")
    
    subplot(3,1,2)
    plot(t, state(:,2));
    hold on;
    plot(t, sampledata(1:n,2));
    ylabel("y (m)")
    
    subplot(3,1,3)
    plot(t, state(:,3));
    hold on;
    plot(t, sampledata(1:n,3));
    ylabel("z (m)")
    
    figure;
    title("Velocity (m)")
    subplot(3,1,1)
    plot(t, state(:,4));
    hold on;
    plot(t, sampledata(1:n,4));
    ylabel("${\dot{x}}$ (m)", 'interpreter', 'latex', 'FontWeight', 'bold')
    
    subplot(3,1,2)
    plot(t, state(:,5));
    hold on;
    plot(t, sampledata(1:n,5));
    ylabel("${\dot{y}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')
    
    subplot(3,1,3)
    plot(t, state(:,6));
    hold on;
    plot(t, sampledata(1:n,6));
    ylabel("${\dot{z}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')

end

if plot3_pos == true

    figure;
    hold on;
    title('3D Position');
    plot3(state(:,1), state(:,2), state(:,3), 'b')
    xlabel("x (m)")
    ylabel("y (m)")
    zlabel("z (m)")

    plot3(sampledata(1:n,1), sampledata(1:n,2), sampledata(1:n,3), 'r')


end