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

%% Timestep

N = 6794*2;                 % Number of data points
dt = 1;                     % Time step [s]
t1 = 0;                     % Start time [s]
t2 = N*dt + t1;             % End time [s]
t = linspace(t1, t2, N);    % N times from t1 to t1

%% Set the initial conditions

mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
rad_moon = 1737447.78;                  % Radius of the Moon [m]
orbit_alt = 50000;                      % Chief altitude above the moon [m]
orbit_rad = orbit_alt + rad_moon;       % Orbital radius of the chief [m]
n_mm = sqrt(mu_moon/(orbit_rad^3));     % Mean motion of the Chief around the Moon [rad/s] 
A0 = 0;                                % Initial X offset of deputy from chief [m]

T = 2*pi/n_mm;                          % Chief Orbital Period [s]
df = 360/T/dt;                          % True anomaly step [deg/step]
f = mod(df.*t, 360);                    % N true anomaly angles [deg]
chief_V = sqrt(mu_moon/orbit_rad);      % Total chief velocity [m/s]       
chief_state = [orbit_rad.*cosd(f); orbit_rad.*sind(f); zeros(1,N); -sind(f).*chief_V; cosd(f).*chief_V; zeros(1,N)]';


% Initial estimate
alpha = (0*pi) / 180;                   % Phase Angle Alpha of Circular Orbit
x_0 = A0*cos(alpha);                    % Initial X Position [m] Hill Frame
y_0 = -2*A0*sin(alpha);                 % Initial Y Position [m] Hill Frame
x_dot_0 = -x_0*n_mm*sin(alpha);         % Initial Orbital X velocity [m/s] Hill Frame
y_dot_0 =  -2*n_mm*x_0*cos(alpha);      % Initial Orbital Y velocity [m/s] Hill Frame
x_n_n_G = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];
x_n_n_F = [x_0; y_0; 0; x_dot_0; y_dot_0; 0];


% Initial estimate uncertainty
p_n_n_F = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];

%% Generate the Acceleration Measurement

[thrust_accel_noisy,thrust_accel_ideal] = sample_thrust(dt);
sigma_thrust = 1;
fprintf("\nVelocity impluse: %0.4f Ideal, %0.4f Noisy\n", dt*sum(thrust_accel_ideal),dt*sum(thrust_accel_noisy))

%% Run the Kalman Filter to Generate 'Truth Data'
%{
if isfile("GS_data.mat") == false
    fprintf("Generating GS data...")
    GS_data()
    fprintf(" Done! \n")
end

load GS_data.mat
%}
%% Run the Kalman Filters
stateG = [];
stateF = [];
errorF = [];

% Thrust Variables
j = 1;
accel_flag = false;

% Accelerator Process Noise Std. Dev.
Qa_F = eye(3,3) .* 0.1;

% Process Noise Covariance Matrix
% Q_func = @(dt,n,sigma_ax,sigma_ay,sigma_az)reshape([1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).^2+1.0./n.^2.*sigma_ay.^2.*(cos(dt.*n)-1.0).^2.*4.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,0.0,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,0.0,1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).*(cos(dt.*n)-1.0).*-2.0+1.0./n.^2.*sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*2.0,1.0./n.^2.*sigma_ax.^2.*(cos(dt.*n)-1.0).^2.*4.0+1.0./n.^2.*sigma_ay.^2.*(sin(dt.*n).*4.0-dt.*n.*3.0).^2,0.0,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,0.0,0.0,1.0./n.^2.*sigma_az.^2.*sin(dt.*n).^2,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,(sigma_ax.^2.*cos(dt.*n).*sin(dt.*n))./n-(sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*4.0)./n,(sigma_ay.^2.*sin(dt.*n).*(sin(dt.*n).*4.0-dt.*n.*3.0).*2.0)./n+(sigma_ax.^2.*cos(dt.*n).*(cos(dt.*n)-1.0).*2.0)./n,0.0,sigma_ax.^2.*cos(dt.*n).^2+sigma_ay.^2.*sin(dt.*n).^2.*4.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,0.0,(sigma_ax.^2.*sin(dt.*n).^2.*-2.0)./n-(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(cos(dt.*n)-1.0).*2.0)./n,(sigma_ax.^2.*sin(dt.*n).*(cos(dt.*n)-1.0).*-4.0)./n+(sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).*(sin(dt.*n).*4.0-dt.*n.*3.0))./n,0.0,sigma_ay.^2.*sin(dt.*n).*(cos(dt.*n).*4.0-3.0).*2.0-sigma_ax.^2.*cos(dt.*n).*sin(dt.*n).*2.0,sigma_ax.^2.*sin(dt.*n).^2.*4.0+sigma_ay.^2.*(cos(dt.*n).*4.0-3.0).^2,0.0,0.0,0.0,(sigma_az.^2.*cos(dt.*n).*sin(dt.*n))./n,0.0,0.0,sigma_az.^2.*cos(dt.*n).^2],[6,6]);
% Q = Q_func(dt,n_mm,sigma_a(1),sigma_a(2),sigma_a(3));

for i = 1:N

    % Control Input
    U_n_G = zeros(3,1);
    U_n_F = zeros(3,1);
    
%     % Unit vector pointing to deputy
%     current_R = norm(chief_state(i,1:3));
%     unit_pos = chief_state(i,1:3)./current_R;

    % Start thrusting one quater of the way through the orbit (N/4)
    if (i > N/4) && (j < length(thrust_accel_ideal))

        % Accelerations
        a_x_G = thrust_accel_ideal(j);
        a_y_G = 0;
        a_z_G = 0;

        a_x_F = thrust_accel_noisy(j);
        a_y_F = 0;
        a_z_F = 0;

        % Set the current measurement vector
        %M_n = [0; 0; 0; a_x*dt; a_y*dt; a_z*dt] + x_n_n;
        
        % Get the current Measurement Error
        %R_n = p_n_n + [zeros(3) zeros(3); zeros(3) eye(3)*sigma_thrust];
        
        % Control Input
        U_n_G(1) = U_n_G(1) + a_x_G;
        U_n_G(2) = U_n_G(2) + a_y_G;
        U_n_G(3) = U_n_G(3) + a_z_G;

        U_n_F(1) = U_n_F(1) + a_x_F;
        U_n_F(2) = U_n_F(2) + a_y_F;
        U_n_F(3) = U_n_F(3) + a_z_F;
        
        % Increment the j index to get the next acceleration
        j = j + 1;

    end
    
    % Handle time edge case
    if i == 1
        t1 = 0;
    else
        t1 = t(i-1);
    end
    t2 = t(i);


    % Regain contact three quaters of the way through the orbit (N*3/4)
    if (i > N*3/4) 
        M_n_F = x_n_n_G;
        R_n_F = [eye(3)*0.0001 zeros(3); zeros(3) eye(3)*0.0001];
        [x_n_n_F, p_n_n_F] = KF_cw(n_mm, Qa_F, M_n_F, U_n_F, x_n_n_F, p_n_n_F, R_n_F, dt, t1, t2, true);

    else
        M_n_F = zeros(6,1);
        R_n_F = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];
        [x_n_n_F, p_n_n_F] = KF_cw(n_mm, Qa_F, M_n_F, U_n_F, x_n_n_F, p_n_n_F, R_n_F, dt, t1, t2, false);

    end


    x_n_n_G = ground_cw(n_mm, U_n_G, x_n_n_G, dt, t1, t2);
    
    % Save State and Error
    stateG = [stateG; x_n_n_G'];
    stateF = [stateF; x_n_n_F'];
    errorF = [errorF; p_n_n_F];

end

% chief_state = circular orbit of chief --> "nominal" [Moon centered frame]
% state = KF output --> "deviations" [Hill frame]
% deputy_state = c_s+s KF output in same frame as chief state

deputy_state_F = chief_state + stateF;
deputy_state_G = chief_state + stateG;

%% Plot Results
%{
plot_1(stateF, t, "Deputy Deviations From Chief Orbit [Hill Frame]")
plot_1_3D(stateF, "Deputy Deviations From Chief Orbit [Hill Frame]")
% plot_1(chief_state, t, "Chief")
% plot_1(deputy_state, t, "Deputy")

labels1 = ["Chief Vs Deputy Orbits", "Chief", "Deputy"];
plot_2(chief_state, deputy_state_F, t, labels1)
plot_2_3D(chief_state, deputy_state_F, labels1)
%}
plot_1(stateG-stateF, t, "Error in Flight Deviations from Ground Deviations [Hill]");
labels2 = ["Ground vs. Flight Deviations from Chief Orbit [Hill]", "Ground", "Flight"];
plot_2(stateG, stateF, t, labels2)


toc
