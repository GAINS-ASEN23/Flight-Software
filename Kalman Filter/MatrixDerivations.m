% Compute matricies for use in kalman filter
% June 2023
% Bennett Grow

clc; close all; clear all;

syms n t sigma_ax sigma_ay sigma_az tau t1 t2 dt real;

%% CW

% A
A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0];

% F
F = simplify(expm(A*dt));
F_latex = latex(F);

% Gamma
integrand = expm(A.*(t2-tau));
Gamma1 = int(integrand, tau, t1, t2);
Gamma2 = subs(Gamma1, t2-t1, dt);
Gamma3 = subs(Gamma2, t1 - t2, -dt);

Gamma1_latex = latex(Gamma1);
Gamma3_latex = latex(Gamma3);

Gamma1_h = matlabFunction(Gamma1);

% B
B = [zeros(3);eye(3)];

% G
G = Gamma3*B;
G_latex = latex(G);
G_func = matlabFunction(G);

%F = [4-3*cos(n*t) 0 0 sin(n*t)/n -2*cos(n*t)/n+2/n 0; 6*(sin(n*t)-(n*t)) 1 0 -2*(1-cos(n*t))/n (4*sin(n*t)-3*n*t)/n 0; 0 0 cos(n*t) 0 0 sin(n*t)/n; 3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0; -6*n*(1-cos(n*t)) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0; 0 0 -n*sin(n*t) 0 0 cos(n*t)];

% Q
Q_a = sym(zeros(3));
Q_a(1,1) = sigma_ax^2;
Q_a(2,2) = sigma_ay^2;
Q_a(3,3) = sigma_az^2;

Q = F*(B*Q_a*B')*F';
Q_latex = latex(Q);
Q_h = matlabFunction(Q);


%% Kinematics
% A
AK = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];

% F
FK = simplify(expm(AK*dt));
FK_latex = latex(FK);

% Gamma
integrandK = expm(AK.*(t2-tau));
Gamma1K= int(integrandK, tau, t1, t2);
Gamma2K = subs(Gamma1K, t2-t1, dt);
Gamma3K = subs(Gamma2K, t1 - t2, -dt);

Gamma1K_latex = latex(Gamma1K);
Gamma3K_latex = latex(Gamma3K);

Gamma1K_h = matlabFunction(Gamma1K);

% B
BK = [zeros(3);eye(3)];

% G
GK = Gamma3K*BK;
GK_latex = latex(GK);
GK_func = matlabFunction(GK);

%F = [4-3*cos(n*t) 0 0 sin(n*t)/n -2*cos(n*t)/n+2/n 0; 6*(sin(n*t)-(n*t)) 1 0 -2*(1-cos(n*t))/n (4*sin(n*t)-3*n*t)/n 0; 0 0 cos(n*t) 0 0 sin(n*t)/n; 3*n*sin(n*t) 0 0 cos(n*t) 2*sin(n*t) 0; -6*n*(1-cos(n*t)) 0 0 -2*sin(n*t) 4*cos(n*t)-3 0; 0 0 -n*sin(n*t) 0 0 cos(n*t)];

% Q
QK_a = sym(zeros(3));
QK_a(1,1) = sigma_ax^2;
QK_a(2,2) = sigma_ay^2;
QK_a(3,3) = sigma_az^2;

QK = FK*(BK*QK_a*BK')*FK';
QK_latex = latex(QK);
QK_h = matlabFunction(QK);










