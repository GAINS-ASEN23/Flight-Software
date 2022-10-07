function [Vt, Xt] = IM_omega(A,t,Fs)
%{
Last Edited: Bennett Grow 10/6/22

REFERENCES: 
    Brandt & Brinker - Integrating time signals in frequency domain 
        Elsevier Measurement 58 (2014) 511-519

%}

% Preallocate vectors
L = length(A);
A = [A zeros(1,L)];  % Pad with zeros
k = 1:2*L;
X = fft(A);

wk = 2j.*pi.*k./(2.*L);

YV = X./wk;
YX = X./(wk).^2;

yV = ifft(YV);
yX = ifft(YX);

Vt = yV./Fs;
Xt = yX./Fs;







%{
n = [t zeros(1,L)];
Atz = [A zeros(1,L)];
Vfz = zeros(1,2*L);
Y = zeros(1,2*L);
y = zeros(1,2*L);

% Atz = Atz - mean(A);

for k = 1:L
    Vfz(k) = sum(Atz(k)*exp(-2j*pi*n.*k/(2*L)));

    if Vfz(k) == 0
        Y(k) = 0;
    else
        Y(k) = 2*L*Vfz(k)/(2j*pi*Fs*k);
    end

end

Y = [Y, flip(imag(Y))];

for k = 1:L
    y(k) = sum(Y(k).*exp(2j.*pi.*n.*k./(2.*L)))./(2.*L);
end

Vt = y(1:L);
%}

end