close all
%Input
vt = [2.00, 2.31, 2.62, 2.93, 3.24, 3.55, 3.86, 4.17, 4.48, 4.80]
t0 = [10.99, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
%Get from graph
x0 = [16.41, 0.1011, 0.1011, 0.1011, 0.1011, 0.1011, 0.1011, 0.1011, 0.1011, 0.1011];			%Starting point of the slope
y0 = [0.19, 213.8, 213.8, 213.8, 213.8, 213.8, 213.8, 213.8, 213.8, 213.8];
ti = [10, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09];				%Starting time
ts = [50, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15];				%Setting time
w_w_final = [1705, w_w(100), w_w(100), w_w(100), w_w(100), w_w(100), w_w(100), w_w(100), w_w(100), w_w(100)];
im_final = [im(100), im(100), im(100), im(100), im(100), im(100), im(100), im(100), im(100), im(100)];
dt = time(101)-time(100);

%Slope Prior estimation
m = [1000, 440000, 440000, 440000, 440000, 440000, 440000, 440000, 440000, 440000];

% Calculation
i = 1;
x = linspace(x0(i), ts(i));
y = m(i)*(x-x0(i))+y0(i);
k = w_w_final(i)*ones(1000);

KMi = m(i)*I/(rad2rpm*im_final(i))
Tmeci = x0(i)+(w_w_final(i)-y0(i))/m(i)-t0(i)
Bi = I/Tmeci

% Graph
figure(1)
plot(time, w_w, 'Color',[0,0.4,0.6])
hold on
plot(x, y, 'r')
hold on
plot(k, 'b')
xlim([ti(i) ts(i)])
ylim([0 w_w_final(i)+500])


KM_i = [7.6063, 7.3510];
Bi = [0.0028, 0.0027];

Km_estimeted = mean(KM_i)
Bi_estimated = mean(Bi)