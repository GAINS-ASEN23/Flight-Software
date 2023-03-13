clc; close all; clear all;

syms n t s1 s2 s3 s4 s5 s6 real;


A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0];

simplify(expm(A));
F = [4-3*cos(n*t) 0 0 sin(n*t)/n -2*cos(n*t)/n+2/n 0; 6*(sin(n*t)-(n*t)) 1 0 -2*(1-cos(n*t))/n (4*sin(n*t)-3*n*t)/n 0; 0 0 cos(n*t) 0 0 sin(n*t)/n; 3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0; -6*n*(1-cos(n*t)) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0; 0 0 -n*sin(n*t) 0 0 cos(n*t)];
% F = sym(zeros(9));
% F(1:6, 1:6) = F66;

Q_a = sym(zeros(6));
Q_a(1,1) = s1^2;
Q_a(2,2) = s2^2;
Q_a(3,3) = s3^2;
Q_a(4,4) = s4^2;
Q_a(5,5) = s5^2;
Q_a(6,6) = s6^2;

Q = F*Q_a*F';
Q_latex = latex(Q);

d = eig(F)






