
clc; close all; clear all;


%% First Test
%{
d1 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '11', '/accel.csv'));
d2 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '12', '/accel.csv'));
d3 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '13', '/accel.csv'));
d4 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '14', '/accel.csv'));
d5 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '15', '/accel.csv'));

d1(:,1) = (d1(:,1)-d1(1,1))./1E6;
ambient_bin = mean(d1(:,3));

d2(:,1) = (d2(:,1)-d2(1,1))./1E6;
d3(:,1) = (d3(:,1)-d3(1,1))./1E6+d2(end,1);
d4(:,1) = (d4(:,1)-d4(1,1))./1E6+d3(end,1);
d5(:,1) = (d5(:,1)-d5(1,1))./1E6+d4(end,1);

d = [d2;d3;d4;d5];

p = polyfit(d(:,1),d(:,3),1);
y = polyval(p,d(:,1));

figure;
hold on;
title('First Test')
xlabel('Seconds')
ylabel('Bins')
plot(d(:,1),y,'k',LineWidth=2)
plot1 = plot(d(:,1),d(:,3),'b');
plot1.Color(4) = 0.05;
%}

%% Second Test
ref = readmatrix(strcat('GAINS 8 Pos Test/temp/ref.csv'));
pref = polyfit(ref(:,1),ref(:,2),3);
yref = polyval(pref,ref(:,1));

d = readmatrix(strcat('GAINS 8 Pos Test/temp/accel.csv'));
d(:,1) = (d(:,1)-d(1,1))./1E6;
p1 = polyfit(d(:,1),d(:,2),1); % Accel
yd1 = polyval(p1,d(:,1));
p2 = polyfit(d(:,1),d(:,3),3); % Temp
yd2 = polyval(p2,d(:,1));

figure;
hold on;
title('Oven Test')
xlabel('Seconds')
yyaxis left;
ylabel('Bins')
plot(d(:,1),yd2,'b',LineWidth=2)
plot1 = plot(d(:,1),d(:,3),'b');
plot1.Color(4) = 0.05;

yyaxis right;
ylabel('Degrees C')
plot(ref(:,1),yref,'r',LineWidth=2)
scatter(ref(:,1),ref(:,2),'r',Marker='o')

legend('Raw Fit','Raw Data','Ref Temp Fit','Ref Temp Raw',Location='northwest')

figure;
hold on;
plot(d(:,1),d(:,2));
plot(d(:,1),yd1,'r');

