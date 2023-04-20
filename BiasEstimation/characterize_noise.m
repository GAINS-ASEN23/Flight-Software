
clc; close all; clear all;

%% Load and clean data
% From freezer test - raw accelerometer data averaged over 5ms
state = readmatrix('GAINS 8 Pos Test/temp/state.csv'); 
startpt = 11000;
endpt = 145000;
raw = state(startpt:endpt,2);

L = length(raw);
t1 = state(startpt,1);
t2 = state(endpt,1);
t = state(startpt:endpt, 1) - t1;
Fs = L / (t2-t1);
T = 1/Fs;
clear state startpt endpt t1 t2;

%% Fit a line to find slope
x = 1:L;
p = polyfit(x,raw,1);

%% Center noise about x-axis
raw_centered = raw' - p(1).*(x-mean(x));
raw_centered = raw_centered - mean(raw_centered);

%% Noise characterization
[R,POW] = snr(raw_centered);
R = db2mag(R);
y1 = zeros(1,L);
y2 = awgn(y1,R,POW);

%% Plotting
figure;
title("Reference Data")
plot(raw_centered);
lim = 0.06;
ylim([-lim,lim]);

figure;
title("Generated Data")
plot(y2);
ylim([-lim,lim]);



