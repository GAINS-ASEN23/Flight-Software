clc
clear
close all

%% Measured Data
accels = zeros(1,9);
for i = 1:9
    d = readmatrix(strcat('GAINS 8 Pos Test/GAINS000', string(i), '/accel.csv'));
    accels(i) = mean(d(:,2));
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
scatter(ref,accels,400, '.k')
hold on
plot(x,y,'r',"LineWidth", 2)
text(-0.5,0.4,strcat("Bias: ",num2str(p(2))))
text(-0.5,0.5,strcat("Scale Factor: ",num2str(p(1))))
xlabel("Truth [g]")
ylabel("Measured [g]")
title("SDI 1521 Calibration: Measured vs. Truth Accelerations")
legend("Data", "Best Fit")
%xlim([-0.02,0.02])

grid on 
grid minor

%% Verifying
BRver = accels/p(1) - p(2);
bennettsK = 1.0876;
Bennettver = accels/bennettsK - p(2);
disp(abs((ref - BRver)))
disp(abs((ref - Bennettver)))
diff = ref-BRver;
bennett =bennettsK*x + p(2);
%plot(x,bennett,'b',"LineWidth", 2)

gRange = y(abs(x-y) <= 0.02);

error = (BRver - ref)./ref.*100

%% Determine INS g range
ref(4:6)
diff(4:6)
line = polyfit(ref(4:6),-diff(4:6),1);
% x = linspace(diff(4),diff(6), 1000);
range = linspace(-1,1,1000);
y2 = line(1)*x + line(2);

figure
plot(range*line(1) + line(2),range,'k',"LineWidth", 2)
hold on
xline(0.02,"r--","LineWidth", 2)
xline(-0.02,"r--","LineWidth", 2)
yline((0.02-line(2))/line(1), 'r--',"LineWidth", 2)
yline(-(0.02-line(2))/line(1), 'r--',"LineWidth", 2)
xlim([-0.025, 0.025])
grid on
grid minor
xlabel("Calibration Percentage")
ylabel("Acceleration [g]")
title("Acceleration vs Bias Calibration Percentage")

