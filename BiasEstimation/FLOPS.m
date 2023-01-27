% Basic FLOPS calcs 

N = 25; 
L = 6;

Ab = 2 * N^2 - N;
ANN = 2 * N^3 - N^2;
AN6 = 2 * N^2 * L - N * L;
A66 = 2 * L^3 - L^2;
A6N1 = 2 * N * L - L;
AN66 = 2 * N * L^2 - N*L;
I = N^3 + N^2 + N;

eqn1 = Ab + N + Ab;
eqn2 = ANN + ANN + N;
eqn3 = AN6 + AN6 + N + A66 + I + ANN;
eqn4 = N + L + A6N1 + A6N1;
eqn5 = N + AN66 + ANN + ANN + N + AN66 + N + AN66 + AN66;

flop = eqn1 + eqn2 + eqn3 + eqn4 + eqn5
clock = 84E6;
cycles = clock/flop;
flops = flop * 100


DUE = 84E6;         %[MHz]
time = 0.1882;      %[sec]
time/DUE;
