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
tic;

%% Sample Data
sampledata = readmatrix("StarTrackerSample.csv");

n = 20000;                  % Number of data points
dt = 0.01;                  % Time step
t1 = 0;                     % Start time
t2 = n*dt + t1;             % End time
t = linspace(t1, t2, n);    % n times from t1 to t1

%% Propagator
mu = 4.9048695e12;     %Earth Graitational Constant
earthRad = 1737.4;       %km
% Initial Circular Radius of Space Shuttle
r_int = 500;
% Create initial coordinates representing the target (Moon)
xt = earthRad + r_int;
yt = 0;
zt = 0;
xdott = 0;
ydott = sqrt(mu/(earthRad+r_int));
zdott = 0;
rT = [xt, yt, zt]';
vT = [xdott,ydott,zdott]';
% Add 51.65 degrees of inclination to the orbit
inc = -51.65;
RYM = [cosd(inc) 0 sind(inc); 0 1 0; -sind(inc) 0 cosd(inc)];
rT = RYM*rT;
vT = RYM*vT;

% Create initial coordinates representing the chaser satellite (Hubble)
% Initial Positions and Velocities of the hubble telescope from the space 
% shuttle bay All units are in km/s
% Assume that both telescope and shuttle are docked initially
   xi = rT(1);
   yi = rT(2);
   zi = rT(3);
xdoti = vT(1);
ydoti = vT(2);
zdoti = vT(3);
rI = [xi, yi, zi]';
vI = [xdoti,ydoti,zdoti]';

[rHill,vHill] = ECI2Hill_Vectorized(rT,vT,rI,vI);

% Add initial displacement on the x vector
rHill(1) = rHill(1)+1;

% Now ... recompute the chaser satellite vector ...
[rI,vI] = Hill2ECI_Vectorized(rT,vT,rHill,vHill);

% Find the angular rate of the target orbit (shuttle)
omega = sqrt(mu/sqrt(sum(rT.^2))^3);

% Use the Linear Propagator (LOP) to propagate Hill's coordinates forward in time
% This is to find the relative orbital motion of the "Chaser" satellite
[rHill,vHill] = CWHPropagator(rHill,vHill,omega,t);
[rHill,vHill] = CWHPropagator(rHillInit,vHillInit,omega,t);

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
    
    figure(1);
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
    
    figure(2);
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

    figure(3);
    hold on;
    title('3D Position');
    plot3(state(:,1), state(:,2), state(:,3), 'b')
    xlabel("x (m)")
    ylabel("y (m)")
    zlabel("z (m)")

    plot3(sampledata(1:n,1), sampledata(1:n,2), sampledata(1:n,3), 'r')


end

toc