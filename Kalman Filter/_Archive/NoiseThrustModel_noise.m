%% ASEN 4018
%% Project GAINS - Electronics
%% Re-Scope Thrust Modeling
%% Author: Addison Woodard
%% Date: 11/21/22

%% Housekeeping
clear; close all; clc;

%% Create Noise
timevec = 1:0.1:(60*60*2);
noisevec = randn(1,length(timevec));              %2 hours of noise
func = @(x) noisevec(x);

%% Create Thrust Curve
start = 0:1:20;
val1 = zeros(size(start));

leadup = 20:1:30;
val2 = -0.4*(20-leadup);

peak = 30:1:83;
val3 = val2(end)*ones(size(peak));

leadout = 83:1:93;
val4 = 0.4*(93-leadout);

ending = 93:1:113;
val5 = zeros(size(ending));


%% Add white noise
sn_white1 = awgn(val1,5);
sn_white2(1,1) = sn_white1(end);
sn_white2(1,2:size(val2,2)) = awgn(val2(2:end),5);
sn_white3(1,1) = sn_white2(end);
sn_white3(1,2:size(val3,2)) = awgn(val3(2:end),5);
sn_white4(1,1) = sn_white3(end);
sn_white4(1,2:size(val4,2)) = awgn(val4(2:end),5);
sn_white5(1,1) = sn_white4(end);
sn_white5(1,2:size(val5,2)) = awgn(val5(2:end),5);


figure
plot(start,val1,'--b','LineWidth',1);
hold on; grid minor;

plot(leadup,val2,'--b','LineWidth',1,'HandleVisibility','off');
plot(peak,val3,'--b','LineWidth',1,'HandleVisibility','off');
plot(leadout,val4,'--b','LineWidth',1,'HandleVisibility','off');
plot(ending,val5,'--b','LineWidth',1,'HandleVisibility','off');

plot(start,sn_white1,'-r','LineWidth',1.5);
plot(leadup,sn_white2,'-r','LineWidth',1.5,'HandleVisibility','off');
plot(peak,sn_white3,'-r','LineWidth',1.5,'HandleVisibility','off');
plot(leadout,sn_white4,'-r','LineWidth',1.5,'HandleVisibility','off');
plot(ending,sn_white5,'-r','LineWidth',1.5,'HandleVisibility','off');
xlim([0 113]);
xlabel('Time [sec]');
ylabel('Thrust [N]'); 
title('SAMPLE Thrust Curve');
legend('Ideal Thrust Curve','Measured Thrust Curve');

%% Integrate the noisy thrust
integ1 = trapz(sn_white1);
integ2 = trapz(sn_white2);
integ3 = trapz(sn_white3);
integ4 = trapz(sn_white4);
integ5 = trapz(sn_white5);
integFinal = integ1 + integ2 + integ3 + integ4 + integ5;
fprintf('Noisy Thrust Area: %0.3f\n',integFinal);

%% Integrate the ideal thrust
integ1i = trapz(val1);
integ2i = trapz(val2);
integ3i = trapz(val3);
integ4i = trapz(val4);
integ5i = trapz(val5);
integFinali = integ1i + integ2i + integ3i + integ4i + integ5i;
fprintf('Ideal Thrust Area: %0.3f\n',integFinali);

AreaDifference = integFinali-integFinal;
fprintf('Difference in Area: %0.6f [Ns]\n', AreaDifference);