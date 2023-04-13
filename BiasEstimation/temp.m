
clc; close all; clear all;
plotgraphs = false;

ref1 = readmatrix(strcat('GAINS 8 Pos Test/temp/ref.csv'));
ref2 = readmatrix(strcat('GAINS 8 Pos Test/temp/ref2.csv'));
ref2(:,1) = -ref2(:,1);
d1 = readmatrix(strcat('GAINS 8 Pos Test/temp/accel.csv'));
d1(:,1) = (d1(:,1)-d1(1,1))./1E6;
d2 = readmatrix(strcat('GAINS 8 Pos Test/temp/accel2.csv'));
d2(:,1) = (d2(:,1)-d2(1,1))./1E6;
d2(:,1) = -d2(:,1);


%% 
ref2(:,1) = ref2(:,1) - 421.84987;
d2(:,1) = d2(:,1) - 421.84987;

ref = [flip(ref2);ref1];
raw = [flip(d2);d1];

reffit = polyfit(ref(:,1),ref(:,2),3);
rawfit = polyfit(raw(:,1),raw(:,3),3);

X = -2500:100:1500;
refy = polyval(reffit, X);
rawy = polyval(rawfit, X);

figure;
hold on;
yyaxis right;
ylabel('Degrees C')
plot(ref(:,1), ref(:,2), 'r');
plot(X, refy, 'r');

title('Temp Over Time')
xlabel('Seconds')
yyaxis left;
ylabel('Bins')
plot(raw(:,1), raw(:,3), 'b');
plot(X, rawy, 'b');


correction = polyfit(refy, rawy, 1);
corrected = raw(:,3) ./ correction(2) - correction(1);
corrected2 = polyval(correction, X);

figure;
hold on;
yyaxis right;
ylabel('Degrees C')
plot(ref(:,1), ref(:,2), 'r');
plot(raw(:,1), corrected, 'k');

figure;
hold on;
yyaxis right;
ylabel('Degrees C')
plot(X, refy, 'g');
plot(X, corrected2, 'k');



% title('Temp Over Time')
% xlabel('Seconds')
% yyaxis left;
% ylabel('Bins')
% plot(raw(:,1), raw(:,3), 'b');
