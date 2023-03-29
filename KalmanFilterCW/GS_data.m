function GS_data
    N_GS = 30000;                  % Number of data points
    dt = 2;                     % Time step
    t1 = 0;                     % Start time
    t2 = N_GS*dt + t1;             % End time
    t_GS = linspace(t1, t2, N_GS);    % n times from t1 to t1

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



    state_GS = [];
    error = [];
   
    
    for i = 1:N_GS

        % Set the current measurement vector
        M_n = zeros(6,1);
        
        % Get the current Measurement Error
        R_n = [eye(3)*1000 zeros(3); zeros(3) eye(3)*(1)];
        
        % Set the accel flag
        accel_flag = false;
        
        % Get the current Input
        U_n = zeros(3,1);
        
        % Run the KF equations for current step
        [x_n_n, p_n_n] = KF_cw(M_n, U_n, x_n_n, p_n_n, R_n, dt, accel_flag);
        
        % Save State and Error
        state_GS = [state_GS; x_n_n'];
        error = [error; p_n_n];
    
    end

    save GS_data.mat state_GS N_GS t_GS

end