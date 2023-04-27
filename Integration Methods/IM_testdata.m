function [A,V,X] = IM_testdata(t)
%{
Last Edited: Bennett Grow 10/5/22

Generates exact acceleration, velocity, and position data using a
composition of sine functions as a base at given times in seconds. Exact
analytical solutions.

Inputs:
    t - time values to evauluate at [sec]

Outputs:
    A, V, X - Acceleration, velocity, position at given times [m/s^2] [m/s]
        [m] - double vectors

%}


% Relationship between amplitude and rms = sqrt(2)
% RMS levels of individual sine waves are 10, 5, 8, and 2
A = 10.*sqrt(2).*sin(100.*pi.*t) + 5.*sqrt(2).*sin(240.*pi.*t) + 8.*sqrt(2).*sin(630.*pi.*t) + 2*sqrt(2).*sin(1000.*pi.*t);
% m/s^2

V = -sqrt(2).*cos(100.*pi.*t)./(10.*pi) - sqrt(2).*cos(240.*pi.*t)./(48.*pi) - 4.*sqrt(2).*cos(630.*pi.*t)./(315.*pi) - sqrt(2).*cos(1000.*pi.*t)./(500.*pi);
% m/s - scale y to mm/s to view

X = -1./(7875.*2.^(7./2).*pi) .* ( 63.*sin(1000.*pi.*t)./(250.*pi) + 160.*sin(630.*pi.*t)./(63.*pi) + 175.*sin(240.*pi.*t)./(16.*pi) + 126.*sin(100.*pi.*t)./(pi) );
% m - scale y to um to view


end