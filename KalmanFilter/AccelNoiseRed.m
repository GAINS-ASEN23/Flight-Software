function [ax_bar, ay_bar, az_bar] = AccelNoiseRed(ax, ay, az)

% Defining accelerometer paramenters from testing (not yet done, filler
% values)

% Bias values (filler -> from datasheet) [m/s^2]
Bx = 0.1962;
By = 0.1962;
Bz = 0.1962;

% Misalighnment values (filler -> all 0s)
Mxy = 0;
Mxz = 0;
Myx = 0;
Myz = 0;
Mzx = 0;
Mzy = 0;

% Scale Factor (filler -> from datasheet)
Sx = 0.75;
Sy = 0.75;
Sz = 0.75;


% matrix math

%A = [ax_bar, ay_bar, az_bar]';
B = [1+Sx Mxy Mxz Bx; Myx 1+Sy Myz By; Mzx Mzy 1+Sz Bz];
C = [ax,ay,az,1]';
A = B*C;
ax_bar = A(1);
ay_bar = A(2);
az_bar = A(3);

% do I not remeber how to matlab, why can't I just do this?
% [ax_bar, ay_bar, az_bar]' = [1+Sx Mxy Mxz Bx; Myx 1+Sy Myz By; Mzx Mzy 1+Sz Bz]*[ax,ay,az,1]';



end