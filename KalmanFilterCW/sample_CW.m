function [X] = sample_CW(N, t, X0)
    mu_moon = 4.9048695e12;                 % Gravitational parameter of the Moon [m^3 s^-2]
    a_moon = 1.74e6+5e4;                    % Semimajor axis of Moon's orbit around Earth [m]
    n = sqrt(mu_moon/(a_moon^3));           % Mean motion of the Moon around the Earth [rad/s] 

    F = @(n,dt) [4-3*cos(n*dt) 0 0 sin(n*dt)/n -2*cos(n*dt)/n+2/n 0; 6*(sin(n*dt)-(n*dt)) 1 0 -2*(1-cos(n*dt))/n (4*sin(n*dt)-3*n*dt)/n 0; 0 0 cos(n*dt) 0 0 sin(n*dt)/n; 3*n*sin(n*dt) 0 0 cos(n*dt) 2*sin(n*dt) 0; -6*n*(1-cos(n*dt)) 0 0 -2*sin(n*dt) 4*cos(n*dt)-3 0; 0 0 -n*sin(n*dt) 0 0 cos(n*dt)];

    X = zeros(N,6);
    X(1,:) = X0;

    for i = 2:N
        X(i,:) = F(n,t(i)) * X(i-1,:)';
    end
end
