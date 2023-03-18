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
tic;

%% Timestep

N = 30000;                  % Number of data points
dt = 2;                     % Time step
t1 = 0;                     % Start time
t2 = N*dt + t1;             % End time
t = linspace(t1, t2, N);    % n times from t1 to t1

%% Set the initial conditions

% The initial estimate state vector for the CW KF
%x_n_n = sampledata(1,1:6)';
mu_moon = 4.9048695e12;%3.986004418e14;%;                 % Gravitational parameter of the Moon [m^3 s^-2]
rad_moon = 1737447.78;                  % Radius of the Moon [m]
% a_moon = 1.74e6+5e4;                   % Semimajor axis of Moon's orbit around Earth [m]
orbit_alt = 50000;                      % Orbit radius [m]
orbit_rad = orbit_alt + rad_moon;
n_mm = sqrt(mu_moon/(orbit_rad^3));      % Mean motion of the Moon around the Earth [rad/s] 
A0 = 0;

% n_mm = sqrt(mu_moon/(A0^3));      % Mean motion of the Moon around the Earth [rad/s] 

alpha = (0*pi) / 180;                   % Phase Angle Alpha of Circular Orbit

x_0 = A0*cos(alpha);             % Initial Position [m] Hill Frame
y_0 = -2*A0*sin(alpha);          % Initial Position [m] Hill Frame
x_dot_0 = -x_0*n_mm*sin(alpha);         % Initial Orbital velocity [m/s] Hill Frame
y_dot_0 =  -2*n_mm*x_0*cos(alpha);      % Initial Orbital velocity [m/s] Hill Frame

x_n_n = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];
%x_n_n = zeros(6,1);


% The initial estimate uncertainty vector for the CW KF
p_n_n = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];

%% Generate the Acceleration Measurement

[~,thrust_accel] = sample_thrust(dt);
sigma_thrust = 1;
fprintf("\nVelocity impluse: %0.3f \n", dt*sum(thrust_accel))

%% Run the Kalman Filter to Generate 'Truth Data'

if isfile("GS_data.mat") == false
    fprintf("Generating GS data...")
    GS_data()
    fprintf(" Done! \n")
end

load GS_data.mat

%% Run the Kalman Filter
state = [];
error = [];

% Thrust Variables
j = 1;
accel_flag = false;

% Accelerator Process Noise Std. Dev.
sigma_a = ones(3,1) .* 0.1;

% Process Noise Covariance Matrix
Q_func = @(dt,n,sigma_ax,sigma_ay,sigma_az)reshape([1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).^2+1.0./n.^2.*sigma_ay.^2.*(cos(dt.*n)-1.0).^2.*4.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,0.0,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,0.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,1.0./n.^2.*sigma_ax.^2.*(cos(dt.*n)-1.0).^2.*4.0+1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).^2,0.0,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,0.0,0.0,1.0./n.^2.*sigma_az.^2.*sin(dt.*n).^2,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,0.0,sigma_ax.^2.*cos(dt.*n).^2+sigma_ay.^2.*sin(dt.*n).^2.*4.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,0.0,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,sigma_ax.^2.*sin(dt.*n).^2.*4.0+sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).^2,0.0,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,0.0,0.0,sigma_az.^2.*cos(dt.*n).^2],[6,6]);
Q = Q_func(dt,n_mm,sigma_a(1),sigma_a(2),sigma_a(3));

for i = 1:N
    % Start thrusting about halfway through the orbit (N/2)
    if (i > N/2) && (j < length(thrust_accel))

        % Get the acceleration
        accel = thrust_accel(j).*x_n_n(1:3)./norm(x_n_n(1:3));
        a_x = accel(1);
        a_y = accel(2);
        a_z = accel(3);

        a_x = thrust_accel(j);
        a_y = 0;
        a_z = 0;

        % Set the current measurement vector
        %M_n = [0; 0; 0; a_x*dt; a_y*dt; a_z*dt] + x_n_n;
        
        % Get the current Measurement Error
        %R_n = p_n_n + [zeros(3) zeros(3); zeros(3) eye(3)*sigma_thrust];
        
        % Control Input
        U_n = [a_x; a_y; a_z];

        % Set the accel flag
        accel_flag = false;
        
        % Increment the j index to get the next acceleration
        j = j + 1;

    else
        % Control Input
        U_n = zeros(3,1);
        
        % Set the accel flag
        accel_flag = false;

        % Control Input
        U_n = zeros(3,1);
    end
    
    % Set the current measurement vector
    M_n = zeros(6,1);
    
    % Get the current Measurement Error
    R_n = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];
    
    if i == 1
        t1 = 0;
    else
        t1 = t(i-1);
    end
    t2 = t(i);

    % Run the KF equations for current step
    [x_n_n, p_n_n] = KF_cw(Q, M_n, U_n, x_n_n, p_n_n, R_n, dt, t1, t2, accel_flag);
    
    % Save State and Error
    state = [state; x_n_n'];
    error = [error; p_n_n];

end

%% Plot Results
close all;

plot_pos_vel = true;
plot3_pos = true;

plot_KF(plot_pos_vel, plot3_pos, state, t, state_GS, t_GS, rad_moon)

toc
