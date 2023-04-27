clear
clc
close all

%% Load Data
data1 = readmatrix('GAINS0001/state.csv');


%% Smooth the data

% Segment test data
range = 1250:9350;
smoothRange = 10;
dataRange = 1:smoothRange;

dataTest = data1(range, 2);
timeTest = data1(range,1);

accMeasured = data1(range,2);
accSmooth = zeros(floor(length(dataTest)/smoothRange),1);
timeSmooth = zeros(length(accSmooth),1);

newLength = length(accSmooth);

for i = 1:(newLength-1)
    accSmooth(i) = mean(dataTest(smoothRange * (i-1) + dataRange));
    timeSmooth(i) = mean(timeTest(smoothRange * (i-1) + dataRange));
end

%% plot
figure(1)
plot(timeSmooth(1:(newLength-1)), accSmooth(1:(newLength-1)));
hold on
plot(timeSmooth(1:(newLength-1)),-0.2*sin(timeSmooth(1:(newLength-1))))

%% Find Wavelength
tolerance = 0.01;
logVector = accSmooth <= tolerance & accSmooth >= -tolerance;

% plot(timeSmooth(1:(newLength-1)),logVector(1:(newLength-1)));

figure(2)
P = polyfit(1:length(timeSmooth(logVector(1:(newLength-1)))), timeSmooth(logVector(1:(newLength-1))), 1);
plot(timeSmooth(logVector(1:(newLength-1))));
hold on
vec = 0:(length(timeSmooth(logVector(1:(newLength-1)))));
plot(vec*P(1) + P(2))

figure(3)
plot(diff(timeSmooth(logVector(1:(newLength-1)))));

% zeroTimesRaw = timeSmooth(logVector(1:(newLength-1);
count = 1;
for i = 1:(length(timeSmooth)-1)
    if(logVector(i) == 0 && logVector(i+1) == 1)
       zeroTimes(count) = timeSmooth(i+1);
       count = count + 1;
    end
end

otherVec = diff(zeroTimes);
otherVec = otherVec([1:11,13:17,19:22]);
plot(otherVec(1:20))
hold on
P = polyfit(1:20, otherVec(1:20), 1);
vec = 0:20;
plot(vec*P(1) + P(2))