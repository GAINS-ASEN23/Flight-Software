%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                            %
%          GAINS - Kinematics Kalman Filter - ASEN 4018      %
%                                                            %
%                                                            %
%  Jason Popich, Bennett Grow, Addison Woodard, Kaylie Rick  %
%                                                            %
%                        03/18/2023                          %
%                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear;
clc;
close all;
tic;

%% Read in the accelerometer data from GAINS
data = readmatrix('state.csv');
testStart = 10.5;      %%%%[MANUALLY TYPE THIS IN]%%%%
testEnd = 79.78;       %%%%[MANUALLY TYPE THIS IN]%%%%
testTime = testEnd - testStart;

steadyStart = 5.97;    %%%%[MANUALLY TYPE THIS IN]%%%%
steadyEnd = 10.63;     %%%%[MANUALLY TYPE THIS IN]%%%%

logVecTest = data(:,1) >= testStart & data(:,1) <= testEnd;
logVecOffset = data(:,1) >= steadyStart & data(:,1) <= steadyEnd;

time = data(logVecTest,1) - testStart;

%%%%%%% COMPUTE OFFSETS %%%%%%%%
accOffset = mean(data(logVecOffset,2));
velOffset = accOffset * time + 0.17053; %  Extra offset due to static acc?
posOffset = velOffset .* time;

% Creat Plot Vectors
acc = data(logVecTest, 2) - accOffset;
pos = data(logVecTest, 5) + posOffset;
vel = data(logVecTest, 8) + velOffset;

%% Timestep

N = length(acc)*2;          % Number of data points
dt = mean(diff(time));      % Time step [s]
t1 = 0;                     % Start time [s]
t2 = N*dt + t1;             % End time [s]
t = linspace(t1, t2, N);    % N times from t1 to t1

%% Set the initial conditions

% Initial estimate
x_0 = 0;                    % Initial X Position [m]
y_0 = 0;                    % Initial Y Position [m]
x_dot_0 = 0;                % Initial X velocity [m/s]
y_dot_0 = 0;                % Initial Y velocity [m/s]
x_n_n_G = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];
x_n_n_F = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];


% Initial estimate uncertainty
p_n_n_F = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];

%% Run the Kalman Filters
stateF = [];
errorF = [];

% Thrust Variables
j = 1;

% Accelerator Process Noise Std. Dev.
Qa_F = eye(3,3) .* 0.1;

% Process Noise Covariance Matrix
% Q_func = @(dt,n,sigma_ax,sigma_ay,sigma_az)reshape([1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).^2+1.0./n.^2.*sigma_ay.^2.*(cos(dt.*n)-1.0).^2.*4.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,0.0,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,0.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,1.0./n.^2.*sigma_ax.^2.*(cos(dt.*n)-1.0).^2.*4.0+1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).^2,0.0,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,0.0,0.0,1.0./n.^2.*sigma_az.^2.*sin(dt.*n).^2,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,0.0,sigma_ax.^2.*cos(dt.*n).^2+sigma_ay.^2.*sin(dt.*n).^2.*4.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,0.0,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,sigma_ax.^2.*sin(dt.*n).^2.*4.0+sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).^2,0.0,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,0.0,0.0,sigma_az.^2.*cos(dt.*n).^2],[6,6]);
% Q = Q_func(dt,n_mm,sigma_a(1),sigma_a(2),sigma_a(3));

for i = 1:N
    % Start thrusting one quarter of the way through the orbit (N/4)
    if (i > N/4) && (j < length(acc))

        % Accelerations when thrusting
        a_x_F = acc(j);
        a_y_F = 0;
        a_z_F = 0;
        
        % Increment the j index to get the next acceleration
        j = j + 1;
    else
        % Accelerations when not thrusting
        a_x_F = 0;
        a_y_F = 0;
        a_z_F = 0;
    end
    
    % Deterministic Control Input
    U_n_F = [a_x_F; a_y_F; a_z_F];
    
    % Handle time edge case
    if i == 1
        t1 = 0;
    else
        t1 = t(i-1);
    end
    t2 = t(i);

    M_n_F = zeros(6,1); % No Ground Measurements
    R_n_F = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];
    [x_n_n_F, p_n_n_F] = KF_accel(Qa_F, M_n_F, U_n_F, x_n_n_F, p_n_n_F, R_n_F, dt, t1, t2, false);
    
    % Save State and Error
    stateF = [stateF; x_n_n_F'];
    errorF = [errorF; p_n_n_F];
end

%% Plot Results

figure;
hold on
plot(t, stateF(:,1))
hold off
title("KF - X Position [m]");
xlabel("Time[s]");
ylabel("X [m]");

figure;
hold on
plot(t, stateF(:,2))
hold off
title("KF - Y Position [m]");
xlabel("Time[s]");
ylabel("Y [m]");

figure;
hold on
plot(t, stateF(:,3))
hold off
title("KF - Z Position [m]");
xlabel("Time[s]");
ylabel("Z [m]");

figure;
hold on
plot(t, stateF(:,4))
hold off
title("KF - X Velocity [m/s]");
xlabel("Time[s]");
ylabel("X Velocity [m/s]");

figure;
hold on
plot(t, stateF(:,5))
hold off
title("KF - Y Velocity [m/s]");
xlabel("Time[s]");
ylabel("Y Velocity [m/s]");

figure;
hold on
plot(t, stateF(:,6))
hold off
title("KF - Z Velocity [m/s]");
xlabel("Time[s]");
ylabel("Z Velocity [m/s]");

toc
