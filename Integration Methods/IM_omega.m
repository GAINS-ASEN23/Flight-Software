function [Vt, Xt] = IM_omega(A,t,Fs)
%{
Last Edited: Bennett Grow 10/11/22

REFERENCES: 
    Brandt & Brinker - Integrating time signals in frequency domain 
        Elsevier Measurement 58 (2014) 511-519

%}

% Preallocate vectors
n = length(A);
N = 2*n;
k = 1:N;

% Remove mean of A
A = A - mean(A);

% FFT
X = fft(A, N);

% Frequency Domain Integration
df = Fs/N;
dw = 2*pi*df;

w = 2j.*pi.*k./(2.*n);

YV = X./w;
YX = X./(w).^2;


% Inverse FFT
yV = ifft(YV);
yX = ifft(YX);

% Scaling factor
Vt = yV./Fs;
Xt = yX./(Fs).^2;


end