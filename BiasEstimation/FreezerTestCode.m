clear; close all; clc;

data = csvread('state.csv');
temps = load('TempData.txt');
times = data(:,1)/60;

figure
%yyaxis left
plot(data(:,3),'b');
grid on;
hold on;
title('Accel vs Time');
xlabel('Time [mins]');
ylabel('Accel [g]');

%yyaxis right
figure
plot([1:23],temps,'.k','MarkerSize',15);
grid on;
yline(0,'--b');
title('Temp vs Time');
xlabel('Time [mins]');
ylabel('Temp [C]')