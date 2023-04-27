function [V_k, X_k] = IM_kinematic(A, V, X, dt)
%{
Last Edited: Bennett Grow 10/6/22

Given acceleration vector returns velocities and positions based on simple
kinematics.

Inputs:
    A - Acceleration data [m/s^2] - double vector 
    V - Initial velocity [m/s] - double
    X - Initial position [m] - double
    dt - Length of time between steps [sec] - double

Outputs:
    V_k - calculated velocities [m/s] - double vector
    X_k - calculated positions [m] - double vector
%}

% Preallocate vectors and set initial positions
n = length(A);
V_k = zeros(1,n);
V_k(1) = V(1);
X_k = zeros(1,n);
X_k(1) = X(1);

% Calculate velocities and positions using 1D kinematic equations
for i = 2:n
    V_k(i) = V_k(i-1) + A(i)*dt;
    X_k(i) = X_k(i-1) + V_k(i)*dt + 0.5*A(i)*dt^2;
end

end