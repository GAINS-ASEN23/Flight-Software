% Coorelate temperature with accelerometer bias change
% April 2023
% Bennett Grow

clc; close all; clear all;

%% Reference Temps
ref1 = readmatrix(strcat('testing_data/temp/ref.csv'));
ref2 = readmatrix(strcat('testing_data/temp/ref2.csv'));
ref2(:,1) = -ref2(:,1);
ref3 = load('testing_data/temp/TempData.txt');
ref3(:,2) = ref3;
ref3(:,1) = -(0:length(ref3)-1).*60;

ref = [flip(ref3);ref1];
ref(:,1) = ref(:,1) + abs(ref(1,1));

%% Oven Test
d1 = readmatrix(strcat('testing_data/temp/accel.csv'));
d1(:,1) = (d1(:,1)-d1(1,1))./1E6;
d2 = readmatrix(strcat('testing_data/temp/accel2.csv'));
d2(:,1) = (d2(:,1)-d2(1,1))./1E6;
d2(:,1) = -d2(:,1);

dt = mean(diff(d1(:,1)));
n = floor(0.01 / dt);
n = 20;
i = 1;
j = 1;
while (j <= length(d1)-n)
    oven(i,1) = d1(j+floor(n/2),1);
    oven(i,2) = mean(d1(j:j+n-1,2));
    j = j + n;
    i = i + 1;
end
    
%% Freezer Test
state = readmatrix('testing_data/temp/state.csv');
stateclean = [-state(2:10:end,1), state(2:10:end,3)];
freezer = flip(stateclean);

%% Organize Data
raw = [freezer; oven];
raw(:,1) = raw(:,1) + abs(raw(1,1));

transition = abs(freezer(1,1));

%% Fit Freezer and Oven Raw Data
pO = polyfit(oven(:,1), oven(:,2), 1);
pF = polyfit(freezer(:,1), freezer(:,2), 1);

xF = 0:transition;
xO = transition+1:2800;

%% Evaluate Fit centered around X=0 at transition
yFc = polyval([pF(1), transition * -pF(1)],xF);
yOc = polyval([pO(1), transition * -pO(1) ],xO);

%% Fit Ref Data
x = 0:2800-1;
pR = polyfit(ref(:,1),ref(:,2), 1);
yR = polyval(pR,x);

%% Combine Freezer and Oven Fits to One Line
C = [[xF;yFc]';[xO;yOc]'];

pC = polyfit(x,C(:,2), 1);
yC = polyval(pC,x);

t = 0:0.1:60;
p = polyfit(yR,yC,1);
y = polyval(p,t);

%% Plot Fits
figure(2);
title("Fitted Accelerations and Temperatures");
hold on;
grid on;
xlim([0,ref(end,1)])
yyaxis left;
ylabel("Acceleration [g]");
% plot(raw(:,1), raw(:,2), 'b');
plot(xF,yFc, 'b', LineWidth=2, LineStyle='-');
plot(xO,yOc, 'r', LineWidth=2, LineStyle='-');
plot(x,yC, 'k',LineWidth=2);

yyaxis right;
ylabel("Temperature [deg C]");
plot(ref(:,1), ref(:,2), 'g', LineWidth=2);
plot(x, yR, 'k', LineWidth=2);

legend("Freezer Accel. Fit","Oven Accel. Fit","Acceleration Fit","Temp. Raw","Temp. Fit", Location="north")
xlabel("Time [sec]")


%% Local Gravity
g_sl = 9.80665;             % [m/s^2] Acceleration at sea level
r_e =  6.3781 * 10^6;       % [m] Earth's radius
h = 1606;                   % [m] Local altitude
% https://www.advancedconverter.com/map-tools/find-altitude-by-coordinates
g = g_sl*(r_e/(r_e+h))^2;   % [m/s^2] Local acceleration

% Convert g's to m/s^2
y = y * g;
CNClim = 0.013 * g; % Max accel under CNC test

%% Make 0 Acceleration Correction at 20 deg C
correction_shift = y(find(t==20));
y = y - correction_shift;
y = -y;

correction_slope = (y(end) - y(1))/t(end);
correction_bias = y(1);

fprintf("Correction Bias: %.8f  SF: %.8f \n", correction_bias, correction_slope);

%% Plot Acceleration Correction
figure(3);
title("Acceleration Bias Correction With Temperature");
hold on;
grid on;
xlim([0,60]);
ylim([-CNClim,CNClim])
xlabel("Temperature [deg C]");
ylabel("Acceleration [m/s^2]")
text(20,0.075,strcat("Bias: ",num2str(correction_bias)),'FontSize',12)
text(20,0.06,strcat("Scale Factor: ",num2str(correction_slope)),'FontSize',12)
plot(t,y, LineWidth=2);

scatter(25,0.00756,MarkerEdgeColor='r', Marker='o', MarkerFaceColor='r');
text(8,0.015, "(25, 0.00756)",'FontSize',12,'Color','r');

scatter(25,0.00082,MarkerEdgeColor='g', Marker='o', MarkerFaceColor='g');
text(27,-0.005, "(25, 0.00082)",'FontSize',12,'Color','g');

legend("Acceleration Correction","GAINS Bias", "Datasheet Bias", Location="south")

%% Plot Raw Temperature and Acceleration
raw = [flip(stateclean);oven(:,1:2)];
raw(:,1) = raw(:,1) + abs(raw(1,1));

figure(1);
hold on;
grid on;
title("Raw Temperature and Acceleration");
xlabel("Time [sec]");
xlim([0,2800]);
yyaxis left;
ylabel("Acceleration [g]");
plot(raw(:,1), raw(:,2), 'b');
ylim([0.5,1.5]);

yyaxis right;
ylabel("Temperature [deg C]");
plot(ref(:,1), ref(:,2), 'r', LineWidth=2);
ylim([0,63]);
