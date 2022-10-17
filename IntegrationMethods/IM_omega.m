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

% i1 = 1:(N/2);
% w1 = (dw.*i1).^n;
% i2 = (N/2+1):(N);
% w2 = (-dw.*N/2 + dw.*(i2-(N/2))).^n;
% w = [w2 w1];

w = 2j.*pi.*k./(2.*n);

YV = X./w;
YX = X./(w).^2;

% YV = [YV(1:n) conj(flip(YV(1:n)))];

% YV = -imag(YV) + 1j.*real(YV);
% YV = -real(YV) + 1j.*imag(YV);


% Inverse FFT
yV = ifft(YV);
yX = ifft(YX);

% Remove mean and detrend
% yV = yV - mean(yV);
% yX = yX - mean(yX);

% yV = detrend(yV);
% yX = detrend(yX);

% Scaling factor?
Vt = yV./Fs;
Xt = yX./(Fs).^2;

% Vt = yV.*wk;
% Xt = yX.*(wk).^2;

% Vt = yV;
% Xt = yX;

end