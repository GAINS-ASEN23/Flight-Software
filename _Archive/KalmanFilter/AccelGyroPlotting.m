clear; close all; clc;

load 2.7.23-35min-100Hz-accelGyro.mat;
%36:17 of data
%Mult by 5 = 181.4167 min = 3 hr 25 min

%% Accelerometer
x = Acceleration.X;
%x2 = repmat(Acceleration.X,3,1);
y = Acceleration.Y;
%y2 = repmat(y,3,1);
z = Acceleration.Z;

xyz5 = repmat([x,y,z],5,1);

%timestamp = Acceleration.Timestamp;

combinedAccel = sqrt(xyz5(:,1).^2 + xyz5(:,2).^2 + xyz5(:,3).^2);

%t = zeros(size(timestamp));

%for n = 1 : length(timestamp)
%  t(n) = seconds(timestamp(n) - timestamp(1));
%end

figure
plot(1:length(xyz5), combinedAccel)
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accel Data repeated 5x');

%% Gyro
gx = AngularVelocity.X;
gy = AngularVelocity.Y;
gz = AngularVelocity.Z;

gxyz5 = repmat([gx,gy,gz],5,1);

combinedRot = sqrt(gxyz5(:,1).^2 + gxyz5(:,2).^2 + gxyz5(:,3).^2);

figure
plot(1:length(gxyz5), combinedRot)
xlabel('Time (s)')
ylabel('Angular Velocity (m/s^2)')
title('Gyro Data repeated 5x')
ylim([-2 50]);

%% Save off data

