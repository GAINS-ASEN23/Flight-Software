function [y] = IM_AB(f,dt,t,ya)
%{
Last Edited: Bennett Grow 10/11/22

Integration using the Adams-Bashforth-Moulton method.
https://en.wikipedia.org/wiki/Linear_multistep_method#Two-step_Adams%E2%80%93Bashforth

https://www.math.mcgill.ca/gantumur/math170c/matlab/abm4.m

%}


h = dt;
h24 = h / 24;

y(1,:) = ya;

m = min(3,n);

for i = 1 : m % start-up phase, using Runge-Kutta of order 4
    s(i,:) = f(i);
    s2 = f(t(i) + h / 2, y(i,:) + s(i,:) * h /2);
    s3 = f(t(i) + h / 2, y(i,:) + s2 * h /2);
    s4 = f(t(i+1), y(i,:) + s3 * h);
    y(i+1,:) = y(i,:) + (s(i,:) + s2+s2 + s3+s3 + s4) * h / 6;
end

for i = m + 1 : n % main phase
    s(i,:) = f(t(i), y(i,:));
    y(i+1,:) = y(i,:) + (55 * s(i,:) - 59 * s(i-1,:) + 37 * s(i-2,:) - 9 * s(i-3,:)) * h24; % predictor
    t(i+1) = t(i) + h;
    y(i+1,:) = y(i,:) + (9 * f(t(i+1), y(i+1,:)) + 19 * s(i,:) - 5 * s(i-1,:) + s(i-2,:)) * h24; % corrector
end
