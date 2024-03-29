clear
clc
close all

%% Segment Data 1 --------------------------------------------------------%
data = readmatrix('GAINS0001/state.csv');
testStart = 10.5;      %%%%[MANUALLY TYPE THIS IN]%%%%
testEnd = 79.78;       %%%%[MANUALLY TYPE THIS IN]%%%%
testTime = testEnd - testStart;

steadyStart = 5.97;    %%%%[MANUALLY TYPE THIS IN]%%%%
steadyEnd = 10.63;     %%%%[MANUALLY TYPE THIS IN]%%%%

logVecTest = data(:,1) >= testStart & data(:,1) <= testEnd;
logVecOffset = data(:,1) >= steadyStart & data(:,1) <= steadyEnd;

time = data(logVecTest,1) - testStart;

%%%%%%% COMPUTE OFFSETS %%%%%%%%
accOffset = mean(data(logVecOffset,2));
velOffset = accOffset * time + 0.17053; %  Extra offset due to static acc?
posOffset = velOffset .* time;

% Creat Plot Vectors
acc = data(logVecTest, 2) - accOffset;
pos = data(logVecTest, 5) + posOffset;
vel = data(logVecTest, 8) + velOffset;

%% Create Test Path
r = 0.15; % [m]
accIdeal = 0.137; % [m/s^2]
velIdeal = sqrt(accIdeal*r); % [m/s];

% testTime = 68.93; % tf - t0  [s]
numRev = 10.5;
wavelength = testTime/numRev; % [s]
frequency = 2*pi/(wavelength);
phaseShift = (testTime/numRev)/4; % Phase shift one quarter rotation

idealTime = linspace(0,testTime, 1000);

idealAcc = accIdeal * sin(frequency * (time + phaseShift));
idealPos = -r * sin(frequency * (time + phaseShift));
idealVel = velIdeal * cos(frequency * (time + phaseShift));



%% Process the Acceleration
dt = mean(diff(time));

for i = 1:(length(time) - 1)
    if (i == 1)
        vel_matlab(i) = acc(i)*dt;
        pos_matlab(i) = vel_matlab(i)*dt + (1/2)*(acc(i))*(dt^2);
    else
        vel_matlab(i) = vel_matlab(i-1) + acc(i)*dt;
        pos_matlab(i) = pos_matlab(i-1) + vel_matlab(i)*dt + (1/2)*(acc(i))*(dt^2);
    end
end



%% Plot Dataset 1
figure(1)
subplot(3,1,1) % Acceleration Plot
plot(time, acc);
hold on
plot(time, idealAcc, 'LineWidth', 2);
title("Raw Accelerations from GAINS", 'FontSize', 24);
xlabel("Time [sec]", 'FontSize', 24);
ylabel("Acceleration [m/s^2]", 'FontSize', 24);
legend('GAINS', 'Actual');

subplot(3,1,2) % Position Plot
plot(time, pos, 'LineWidth', 2)
hold on
plot(time, idealPos, 'LineWidth', 2)
title("Calculated Position from GAINS", 'FontSize', 24);
xlabel("Time [sec]", 'FontSize', 24);
ylabel("Position [m]", 'FontSize', 24);
legend('GAINS', 'Actual');

subplot(3,1,3) % Velocity Plot
plot(time, vel, 'LineWidth', 2)
hold on
plot(time, idealVel, 'LineWidth', 2)
title("Calculated Velocity from GAINS", 'FontSize', 24);
xlabel("Time [sec]", 'FontSize', 22);
ylabel("Velocity [m/s]", 'FontSize', 22);
legend('GAINS', 'Actual');

%% Plot Error Graphs for Dataset 1
figure(2)
subplot(1,2,1)
plot(time, pos-idealPos);
yline(1000, 'k--');
ylim([-100,1100]);
title("GAINS Position Error with Time", 'FontSize', 24);
xlabel("Time [sec]", 'FontSize', 24);
ylabel("Position Error [m]", 'FontSize', 24);
legend('Error', 'Requirement');

subplot(1,2,2)
plot(time, vel-idealVel);
yline(0.4, 'k--');
ylim([-0.1,0.5]);
title("GAINS Velocity Error with Time", 'FontSize', 24);
xlabel("Time [sec]", 'FontSize', 24);
ylabel("Velocity Error [m/s]", 'FontSize', 24);
legend('Error', 'Requirement');

%% Matlab Integration
figure(3)
% plot(time, acc)
hold on
% plot(time, pos);
plot(time, vel);
plot(time(1:length(time)-1), pos_matlab);
plot(time(1:length(time)-1), -vel_matlab);
% plot(time, cumtrapz(time,cumtrapz(time,acc)));
title("Matlab Computed Integrations");
xlabel("Time [sec]");
% ylabel("");
legend('Velocity (GAINS)', 'Position (Matlab)', 'Velocity (Matlab)', Location='northwest');

%% Plot Datarate
figure(5)
plot(diff(data(:,1)));
ylim([0,0.15]);
