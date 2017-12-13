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
Irt = Ir1+Ir2+Ir3+Ir4

%% Parametros Disco
m_disc = 23e-3;
rext = 0.5*9.5e-2;								%[m]
rint = 0.5*2.5e-2;								%[m]
h_disc = 0.2e-2;								%[m]
Iw = 0.5*m_disc*(rext^2+rint^2)					%[Kgm^2]

I = Irt+Iw										%[Kgm^2]
J = 0.25*m_disc*(rext^2);						%[Kgm^2]

% Eletricos
Rfn = 1.9;										%[Ohm]
Rff = 4.1;										%[Ohm]
R = 2*Rfn;										%[Ohm]
L = (5e-3)*2;									%[H]

% Modelo
KM = 0.075;										%[Nm/A]
B = 2.8645e-05;									%[Nm/rads^-1]
Kv = KM;										%[V/rads^-1]

% Factores de conversion
rad2rpm = 60/(2*pi);

tau_m = I/B
tau_e = L/R

%% Parametros Satélite

IA = 0.00050844;
IB = 0.00050602;
IC = 0.00052641;
Ix = IA+I+2*J;
Iy = IB+I+2*J;
Iz = IC+I+2*J;

Bz = 1.8645e-05;								%[Nm/rads^-1]