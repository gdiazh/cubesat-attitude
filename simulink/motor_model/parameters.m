%% Parametros Motor
% Eletricos
R = 4*2;			%[Ohm]
L = (5e-3)*2;		%[H]
% Mecanicos
KM = 5.54;			%[Nm/A]
mr = 25e-3;			%[Kg]
rr = 1.25e-2;		%[m]
Ir = 0.5*mr*rr^2;	%[Kgm^2]
B = 0.0022;			%[Nm/rads^-1]
Kv = 0.003;			%[V/rads^-1]
%% Parametros Disco
Iw = 2.7744e-5;		%[Kgm^2]
I = Ir+Iw			%[Kgm^2]

% Factores de conversion
rad2rpm = 60/(2*pi);

tau_m = I/B
tau_e = L/R

% A = 0.00050844;
% B = 0.00050602;
% C = 0.00052641;
% I = 2.0587e-05;
% J = 1.0298e-05;
% Isx = A+I+2*J;
% Isy = B+I+2*J;
% Isz = C+I+2*J;