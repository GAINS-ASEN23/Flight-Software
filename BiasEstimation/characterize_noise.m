
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

y1 = zeros(1,L);
y2 = awgn(y1,R,POW);

%% Plotting
figure;
plot(raw_centered);

figure;
plot(y2);


figure; 
plot(f,P1) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")

