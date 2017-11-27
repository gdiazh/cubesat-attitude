%Update real data
file_path = '../../scripts/motor_model_data/';
n_test = input('test number: ','s');
file_name = 'kt_test';
file_ext = '.txt';
full_name = strcat(file_path, file_name, n_test, file_ext);

data = dlmread(full_name);
im_data = data(:,3)-0.1;
w_w_data = data(:,4);

f = 5000/11;
dt = 1/f;
time_data = 0:dt:(length(im_data)-1)*dt;

%Update simulated data
parameters;
%Input
vt = [2.00, 2.31, 2.62, 2.93, 3.24, 3.55, 3.86, 4.17, 4.48, 4.80];
t0 = [16.41, 18.73, 3.612, 15.96, 13.74, 9.621, 6.939, 1.751, 8.778, 13.4];
i = str2num(n_test);
sim('motor_model');

%Calculate error in steady state

t_steady_sim = 70;
t_steady_data = 20;
n_sim = int32(t_steady_sim/(time_sim(1001)-time_sim(1000)));
n_data = int32(t_steady_data/(time_data(1001)-time_data(1000)));

im_error = im_sim(n_sim) - im_data(n_data)
w_w_error = w_w_sim(n_sim) - w_w_data(n_data)

im_error_p = 100*im_error/im_data(n_data)
w_w_error_p = 100*w_w_error/w_w_data(n_data)

%Plot

close all
figure(1)
fig1 = axes;
ax1 = subplot(1,2,1);
plot(ax1, time_data, im_data, 'b')
hold on
plot(ax1, time_sim, im_sim, 'r')
hold on
plot(ax1, time_data(n_data), im_data(n_data), 'ko')
hold on
plot(ax1, time_sim(n_sim), im_sim(n_sim), 'ko')
grid on
hold on
set(gca,'fontsize',14)
title('Corriente Motor', 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel('Corriente [A]', 'fontsize', 14);
legend('Datos','Modelo')
str1 = strcat('Error E.E. = ', num2str(im_error_p, '%2.2f'), '%');
annotation('textbox',[.2 .5 .3 .3],'String',str1,'FitBoxToText','on');

ax2 = subplot(1,2,2);
plot(ax2, time_data, w_w_data, 'b')
hold on
plot(ax2, time_sim, w_w_sim, 'r')
hold on
plot(ax2, time_data(n_data), w_w_data(n_data), 'ko')
hold on
plot(ax2, time_sim(n_sim), w_w_sim(n_sim), 'ko')
title('Velocidad Motor', 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel('Velocidad [RPM]', 'fontsize', 14);
legend('Datos','Modelo')
str2 = strcat('Error E.E. = ', num2str(w_w_error_p, '%2.2f'), '%');
annotation('textbox',[.75 .5 .3 .3], 'String',str2,'FitBoxToText','on');