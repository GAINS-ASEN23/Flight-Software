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

N = 30000;                  % Number of data points
dt = 2;                     % Time step
t1 = 0;                     % Start time
t2 = N*dt + t1;             % End time
t = linspace(t1, t2, N);    % n times from t1 to t1

%% Set the initial conditions

% The initial estimate state vector for the CW KF
%x_n_n = sampledata(1,1:6)';
mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
rad_moon = 1737447.78;                  % Radius of the Moon [m]
a_moon = 1.74e6+5e4;                    % Semimajor axis of Moon's orbit around Earth [m]
orbit_alt = 50000;                      % Orbit altitude above the moon [m]
orbit_rad = rad_moon + orbit_alt;       % Orbit radius [m]
n_mm = sqrt(mu_moon/(a_moon^3));        % Mean motion of the Moon around the Earth [rad/s] 
alpha = (0*pi) / 180;                   % Phase Angle Alpha of Circular Orbit

x_0 = orbit_rad*cos(alpha);             % Initial Position [m] Hill Frame
y_0 = -2*orbit_rad*sin(alpha);          % Initial Position [m] Hill Frame
x_dot_0 = -x_0*n_mm*sin(alpha);         % Initial Orbital velocity [m/s] Hill Frame
y_dot_0 =  -2*n_mm*x_0*cos(alpha);      % Initial Orbital velocity [m/s] Hill Frame

x_n_n = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];

% The initial estimate uncertainty vector for the CW KF
p_n_n = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];

%% Generate the Acceleration Measurement

% Generate the Thrust
thrust_accel = sample_thrust(dt);
sigma_thrust = 1;


%% Run the Kalman Filter
state = [];
error = [];

% Thrust Variables
j = 1;
accel_flag = false;

for i = 1:N
    % Start thrusting about halfway through the orbit (N/2)
    if (i > N/2) && (j < length(thrust_accel))
        % Get the acceleration
        a_x = thrust_accel(j);
        a_y = 0;
        a_z = 0;

        % Set the current measurement vector
        M_n = [0; 0; 0; a_x*dt; a_y*dt; a_z*dt] + x_n_n;
        
        % Get the current Measurement Error
        R_n = p_n_n + [0; 0; 0; sigma_thrust; sigma_thrust; sigma_thrust];
        
        % Set the accel flag
        accel_flag = true;
        
        % Increment the j index to get the next acceleration
        j = j + 1;
    else
        % Set the current measurement vector
        M_n = zeros(6,1);
        
        % Get the current Measurement Error
        R_n = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];
        
        % Set the accel flag
        accel_flag = false;
    end
    
    % Get the current Input
    U_n = zeros(3,1);
    
    % Run the KF equations for current step
    [x_n_n, p_n_n] = KF_cw(M_n, U_n, x_n_n, p_n_n, R_n, dt, accel_flag);
    
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
    plot(t(1:length(state(:,1))), state(:,1));
    hold on;
    %plot(t, sampledata(1:n,1));
    ylabel("x (m)")
    
    subplot(3,1,2)
    plot(t(1:length(state(:,1))), state(:,2));
    hold on;
    %plot(t, sampledata(1:n,2));
    ylabel("y (m)")
    
    subplot(3,1,3)
    plot(t(1:length(state(:,1))), state(:,3));
    hold on;
    %plot(t, sampledata(1:n,3));
    ylabel("z (m)")
    
    figure;
    title("Velocity (m)")
    subplot(3,1,1)
    plot(t(1:length(state(:,1))), state(:,4));
    hold on;
    %plot(t, sampledata(1:n,4));
    ylabel("${\dot{x}}$ (m)", 'interpreter', 'latex', 'FontWeight', 'bold')
    
    subplot(3,1,2)
    plot(t(1:length(state(:,1))), state(:,5));
    hold on;
    %plot(t, sampledata(1:n,5));
    ylabel("${\dot{y}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')
    
    subplot(3,1,3)
    plot(t(1:length(state(:,1))), state(:,6));
    hold on;
    %plot(t, sampledata(1:n,6));
    ylabel("${\dot{z}}$ (m)", 'interpreter', 'latex', 'FontWeight','bold')

end

if plot3_pos == true
    figure;
    [sx, sy, sz] = sphere(1000);
    surf(sx.*rad_moon,sy.*rad_moon,sz.*rad_moon, 'EdgeColor',[192/256 192/256 192/256]);
    hold on

    title('3D Position');
    plot3(state(:,1), state(:,2), state(:,3), 'b', 'LineWidth', 3)
    xlabel("x (m)")
    ylabel("y (m)")
    zlabel("z (m)")
    axis equal;
        
    %plot3(sampledata(1:n,1), sampledata(1:n,2), sampledata(1:n,3), 'r')
end

toc
