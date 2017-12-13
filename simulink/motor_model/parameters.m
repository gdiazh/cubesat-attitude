%% Parametros Motor

% Parametros Rotor
mr = 27e-3;										%[Kg]

Vr1 = (0.8e-2)*pi*(0.25e-2)^2;					%[m^3]
Vr2 = (0.4e-2)*pi*((0.7e-2)^2-(0.25e-2)^2);		%[m^3]
Vr3 = (0.7e-2)*pi*((1.65e-2)^2-(1.4e-2)^2);		%[m^3]
Vr4 = (1.05e-2)*pi*(1.25e-2)^2;					%[m^3]
Vr = Vr1+Vr2+Vr3+Vr4;

ro_r = mr/Vr;

Ir1 = 0.5*(ro_r*Vr1)*(0.25e-2)^2;
Ir2 = 0.5*(ro_r*Vr2)*((0.7e-2)^2+(0.25e-2)^2);
Ir3 = 0.5*(ro_r*Vr3)*((1.65e-2)^2+(1.4e-2)^2);
Ir4 = 0.5*(ro_r*Vr4)*(1.25e-2)^2;
Irt = Ir1+Ir2+Ir3+Ir4;

%% Parametros Disco
m_disc = 23e-3;
rext = 0.5*9.5e-2;								%[m]
rint = 0.5*2.5e-2;								%[m]
Iw = 0.5*m_disc*(rext^2+rint^2);				%[Kgm^2]

I = Irt+Iw;										%[Kgm^2]

% Eletricos
Rfn = 1.9;										%[Ohm]
Rff = 4.1;										%[Ohm]
R = 2*Rfn;										%[Ohm]
L = (5e-3)*2;									%[H]

% Modelo
KM = 0.0065;									%[Nm/A]
B = 0.21e-05;									%[Nm/rads^-1]
Kv = KM;										%[V/rads^-1]

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