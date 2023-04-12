
clc; close all; clear all;


%% First Test

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

%% Second Test
t2 = readmatrix(strcat('GAINS 8 Pos Test/GAINS00', '15', '/accel.csv'));
t2(:,1) = (t2(:,1)-t2(1,1))./1E6;
p = polyfit(t2(:,1),t2(:,3),1);
y2 = polyval(p,t2(:,1));

figure;
hold on;
title('Second Test')
xlabel('Seconds')
ylabel('Bins')
plot(t2(:,1),y2,'k',LineWidth=2)
plot1 = plot(t2(:,1),t2(:,3),'b');
plot1.Color(4) = 0.05;
