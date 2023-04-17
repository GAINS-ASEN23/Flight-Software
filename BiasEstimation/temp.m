
clc; close all; clear all;
plotgraphs = false;

ref1 = readmatrix(strcat('GAINS 8 Pos Test/temp/ref.csv'));
ref2 = readmatrix(strcat('GAINS 8 Pos Test/temp/ref2.csv'));
ref2(:,1) = -ref2(:,1);
ref3 = load('GAINS 8 Pos Test/temp/TempData.txt');
ref3(:,2) = ref3;
ref3(:,1) = -(0:length(ref3)-1).*60;

d1 = readmatrix(strcat('GAINS 8 Pos Test/temp/accel.csv'));
d1(:,1) = (d1(:,1)-d1(1,1))./1E6;
d2 = readmatrix(strcat('GAINS 8 Pos Test/temp/accel2.csv'));
d2(:,1) = (d2(:,1)-d2(1,1))./1E6;
d2(:,1) = -d2(:,1);

state = readmatrix('GAINS 8 Pos Test/temp/state.csv');
stateclean = [-state(2:10:end,1), state(2:10:end,3)];

%% 


ref = [flip(ref3);ref1];
raw = [flip(stateclean);d1(:,1:2)];
ref(:,1) = ref(:,1) + abs(ref(1,1));
raw(:,1) = raw(:,1) + abs(raw(1,1));



figure;
hold on;
grid on;
title("Effect of Temperature on Accelerometer Bias");
xlabel("Time [sec]");
xlim([0,2800]);
yyaxis left;
ylabel("Acceleration [g's]");
plot(raw(:,1), raw(:,2), 'b');
ylim([0.5,1.5]);


yyaxis right;
ylabel("Temperature [deg C]");
plot(ref(:,1), ref(:,2), 'r', LineWidth=2);
ylim([0,63]);

