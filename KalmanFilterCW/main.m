%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                            %
%              GAINS - CW Kalman Filter - ASEN 4018          %
%                                                            %
%                                                            %
%  Jason Popich, Bennett Grow, Addison Woodard, Kaylie Rick  %
%                                                            %
%                        02/28/2023                          %
%                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear;
clc;
close all;
tic;

%% Sample Data
% sampledata = readmatrix("StarTrackerSample.csv");

N = 35000;                  % Number of data points
dt = 2;                     % Time step
t1 = 0;                     % Start time
t2 = N*dt + t1;             % End time
t = linspace(t1, t2, N);    % n times from t1 to t1

%% Set the initial conditions

% The initial estimate state vector for the CW KF
%x_n_n = sampledata(1,1:6)';

mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
rad_moon = 1738100;                     % Radius of the Moon [m]
a_moon = 1.74e6+5e4;                    % Semimajor axis of Moon's orbit around Earth [m]
orbit_alt = 1000;                       % Orbit altitude above the moon [m]
orbit_rad = rad_moon + orbit_alt;       % Orbit radius [m]
n_mm = sqrt(mu_moon/(a_moon^3));        % Mean motion of the Moon around the Earth [rad/s] 

x_0 = orbit_rad;                        % Initial Position [m] Hill Frame
% y_dot_0 = sqrt(mu_moon / orbit_rad);          % Initial Orbital velocity [m/s] Hill Frame
y_dot_0 = -2*n_mm*x_0;                  % Initial Orbital velocity [m/s] Hill Frame

x_n_n = [x_0; 0; 0; 0; y_dot_0; 0];

% The initial estimate uncertainty vector for the CW KF
p_n_n = [eye(3)*0.1 zeros(3); zeros(3) eye(3)*(1)];


%% Run the Kalman Filter
state = [];
error = [];

for i = 1:N

    % Get the current Measurement
    % M_n = accel_x_n_n;
    M_n = x_n_n;
    
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
    
    figure(1);
    title("Position (m)")
    subplot(3,1,1)
    plot(t, state(:,1));
    hold on;
    %plot(t, sampledata(1:n,1));
    ylabel("x (m)")
    
    subplot(3,1,2)
    plot(t, state(:,2));
    hold on;
    %plot(t, sampledata(1:n,2));
    ylabel("y (m)")
    
    subplot(3,1,3)
    plot(t, state(:,3));
    hold on;
    %plot(t, sampledata(1:n,3));
    ylabel("z (m)")
    
    figure(2);
    title("Velocity (m)")
    subplot(3,1,1)
    plot(t, state(:,4));
    hold on;
    %plot(t, sampledata(1:n,4));
    ylabel("${\dot{x}}$ (m)", 'interpreter', 'latex', 'FontWeight', 'bold')
    
    subplot(3,1,2)
    plot(t, state(:,5));
    hold on;
    %plot(t, sampledata(1:n,5));
    ylabel("${\dot{y}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')
    
    subplot(3,1,3)
    plot(t, state(:,6));
    hold on;
    %plot(t, sampledata(1:n,6));
    ylabel("${\dot{z}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')

end

if plot3_pos == true

    figure(3);
        hold on;
        title('3D Position');
        plot3(state(:,1), state(:,2), state(:,3), 'b')
        xlabel("x (m)")
        ylabel("y (m)")
        zlabel("z (m)")
        axis equal;

        %[sx, sy, sz] = sphere(1000);
        %surf(sx.*rad_moon,sy.*rad_moon,sz.*rad_moon);


    %plot3(sampledata(1:n,1), sampledata(1:n,2), sampledata(1:n,3), 'r')


end

toc