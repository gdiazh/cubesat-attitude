close all
%Input
vt = 2.9;
t0 = 0.1;
%Get from graph
x0 = 0.1011;			%Starting point of the slope
y0 = 213.8;
ti = 0.09;				%Starting time
ts = 0.15;				%Setting time
w_w_final = w_w(100);
im_final = im(100);
dt = time(101)-time(100);

%Slope Prior estimation
m = 440000;

% Calculation
x = linspace(x0, ts);
y = m*(x-x0)+y0;
k = w_w_final*ones(length(w_w));

KMi = m*I/(rad2rpm*im_final)
Tmeci = x0+(w_w_final-y0)/m-t0
Bi = I/Tmeci

% Graph
figure(1)
plot(time, w_w, 'Color',[0,0.4,0.6])
hold on
plot(x, y, 'r')
hold on
plot(time, k, 'b')
xlim([ti ts])
ylim([0 w_w_final+100])


vt_i = [2.1, 2.9];
KM_i = [7.6063, 7.3510];
Bi = [0.0028, 0.0027];

Km_estimeted = mean(KM_i)
Bi_estimated = mean(Bi)