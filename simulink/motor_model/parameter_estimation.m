close all
%Input
vt = 2.1;
%Get from graph
x0 = 0.1011;			%Starting point of the slope
y0 = 154.9;
ti = 0.09;				%Starting time
ts = 0.14;				%Setting time
w_w_final = w_w(100);
im_final = im(100);
dt = time(101)-time(100);

%Prior estimation
m = 330000;

% Calculation
x = linspace(x0, ts);
y = m*(x-x0)+y0;
k = w_w_final*ones(length(w_w));

KM1 = m*I/(rad2rpm*im_final)

% Graph
figure(1)
plot(time, w_w)
hold on
plot(x, y)
hold on
plot(time, k)
xlim([ti ts])
ylim([0 w_w_final+100])