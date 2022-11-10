
clear all; close all; clc;

% Load data to matrix
raw = readmatrix("ADXL357_DATA_1667950841.csv");

% Average temperature of test
temp = mean(raw(:,1)); % [deg C] 

% Number of data points n
n = length(raw);
n_vec = 1:n;

% Split data
X = raw(:,2);
Y = raw(:,3);
Z = raw(:,4);

figure;
subplot(3,1,1)
plot(n_vec, X);
ylabel("X Acceleration [g]");

subplot(3,1,2)
plot(n_vec, Y);
ylabel("Y Acceleration [g]");

subplot(3,1,3)
plot(n_vec, Z);
ylabel("Z Acceleration [g]");







