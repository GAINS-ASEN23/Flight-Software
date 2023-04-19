%{
    bias.m
    Author -> Bennett Grow
    Last Modified -> 4/19/23

    Estimate accelerometer bias and scale factor with 8-position single-axis static testing

    References:
        - Estimation of Deterministic and Stochastic IMU Error Parameters
            By: Unsal and Demirbas, DOI: 978-1-4673-0387-3/12/

        - A Standard Testing and Calibration Procedure for Low Cost MEMS 
          Inertial Sensors and Units
            By: Aggarwal et al., DOI: 10.1017/S0373463307004560

        - Static Calibration of the Tactical Grade Inertial Measurement
          Units By: Hayal
%}

clc
clear
close all

%% Measured Data
accels = zeros(1,9);
stds = zeros(1,9);
for i = 1:9
    d = readmatrix(strcat('GAINS 8 Pos Test/GAINS000', string(i), '/accel.csv'));
    accels(i) = mean(d(:,2));
    stds(i) = std(d(:,2));
end

%% Truth Data
% Local Gravity
g_sl = 9.80665;             % [m/s^2] Acceleration at sea level
r_e =  6.3781 * 10^6;       % [m] Earth's radius
h = 1606;                   % [m] Local altitude
% https://www.advancedconverter.com/map-tools/find-altitude-by-coordinates
g = g_sl*(r_e/(r_e+h))^2;   % [m/s^2] Local acceleration
g_conv = g/g_sl;            % [g's]

% Reference Gravity
ang = linspace(-180,0,9);   % [deg] 
ref = g_conv.*cosd(ang);   %[g's]


%% Plotting
p = polyfit(ref,accels,1);
x = linspace(-1,1);
y = p(1)*x + p(2);
scatter(ref,accels, 20,'k')
hold on
% plot(x,y,'r',"LineWidth", 2)
text(-0.5,0.4,strcat("Bias: ",num2str(p(2))))
text(-0.5,0.5,strcat("Scale Factor: ",num2str(p(1))))

calibrated = accels/p(1)-p(2);
plot(ref,calibrated, 'LineWidth', 2)
plot(ref, ref, 'LineWidth', 2);
xlabel("Truth [g]", 'FontSize', 22)
ylabel("Measured [g]", 'FontSize', 22)
title("SDI 1521 Calibration: Measured vs. Truth Accelerations", 'FontSize', 22)
legend("Measurements", "Calibrated Measurements", "Ideal Calibration", 'FontSize', 13)
%xlim([-0.02,0.02])

grid on 
grid minor

%% 3rd Deg PolyFit
y = ref;
x = accels;

X = linspace(-1.1,1.1,100);

P = polyfit(x,y,3);
YP = polyval(P,X);

corrected_accels = polyval(P,accels);

figure;
hold on;
grid on;
title("3rd Degree Polynomial Fit");
scatter(accels, ref);
plot(X,YP);
scatter(accels, corrected_accels)
ylabel("Actual");
xlabel("measured");

polyerror = (corrected_accels - ref)./ref.*100;


%% Plot Error
figure;
hold on;
grid on;
error_measured = ref - accels;
plot(ref, error_measured);
error_corrected = ref - corrected_accels;
plot(ref, error_corrected);
xlabel("Actual Acceleration [g]")
ylabel("Difference [g]")
title("Difference Between Measured and Actual")
legend("Measured", "Corrected", Location="southwest");
