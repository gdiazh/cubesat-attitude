close all
%Input
vt = [2.00, 2.31, 2.62, 2.93, 3.24, 3.55, 3.86, 4.17, 4.48, 4.80];
t0 = [16.41, 18.73, 3.612, 15.96, 13.74, 9.621, 6.939, 1.751, 8.778, 13.4];												%Input starting time
%Get from graph
x0 = [16.41, 19.04, 3.835, 15.5, 13.8, 10.02, 8.03, 3.018, 10.51, 14.52];						%Starting point of the slope
y0 = [0.19, 155.8, 139.4, 187.5, 181.9, 241.7, 352, 853.8, 801.3, 1235];
ti = [10, 18, 3, 15, 13, 9, 7, 2, 9, 13];												%Slope Starting time
ts = [50, 60, 30, 40, 20, 40, 40, 30, 30, 30];												%Setting time
w_w_final = [1705, 1939, 2596, 3221, 3762, 4819, 5514, 6457, 8694, 9231];
im_final = [0.18, 0.18, 0.20, 0.22, 0.23, 0.26, 0.29, 0.32, 0.4, 0.43]-0.14;
dt = time_data(101)-time_data(100);

%Slope Prior estimation
m = [1000, 1100, 1300, 1400, 7000, 1300, 1300, 2000, 2200, 2800];

% Calculation
i = 1;
x = linspace(x0(i), ts(i));
y = m(i)*(x-x0(i))+y0(i);
k = w_w_final(i)*ones(1000);

KMi = m(i)*I/(rad2rpm*im_final(i))
Tmeci = (x0(i)+(w_w_final(i)-y0(i))/m(i))-t0(i)
Bi = I/Tmeci

% Graph
figure(1)
plot(time_data, w_w_data, 'Color',[0,0.4,0.6])
hold on
plot(x, y, 'r')
hold on
plot(k, 'b')
xlim([ti(i) ts(i)])
ylim([0 w_w_final(i)+500])


KM_i = [0.2981, 0.3280, 0.2584, 0.2087, 0.9276, 0.1292, 0.1034, 0.1325, 0.1009, 0.1151];
Tmec_i = [7.1248, 1.9311, 2.1127, 1.7068, 0.5714, 3.9200, 5.0618, 4.0686, 12.1665, 3.9757];
B_i = [1.5984e-05, 5.8974e-05, 5.3904e-05, 6.6724e-05, 1.9929e-04, 2.9052e-05, 2.2499e-05, 2.7991e-05, 9.3604e-06, 2.8645e-05];

Km_estimeted = mean(KM_i)
Bi_estimated = mean(Bi)