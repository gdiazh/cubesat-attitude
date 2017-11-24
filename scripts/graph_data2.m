%/%%
  % @brief Continuously plot file data
  %/
 
% Author: Gustavo Diaz

close all;
clc

file_path = 'motor_model_data/';
file_name = input('file_name: ','s');
file_ext = '.txt';
full_name = strcat(file_path, file_name, file_ext);

var1 = 0;
var2 = 0;
var3 = 0;
var4 = 0;

sz = 0;
window_sz = 1000;

figure(1)
fig1 = axes;
ax1 = subplot(2,2,1);
plot(ax1, var1, 'b')
grid on
set(gca,'fontsize',14)
title('Var1', 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);
% hold on
% figure(2)
% fig2 = axes;
ax2 = subplot(2,2,2);
plot(ax2, var2, 'r')
grid on
set(gca,'fontsize',14)
title('Var2', 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

ax3 = subplot(2,2,3);
plot(ax3, var3, 'g')
grid on
set(gca,'fontsize',14)
title('Var3', 'fontsize', 14)
xlabel(ax3, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);

ax4 = subplot(2,2,4);
plot(ax4, var4, 'k')
grid on
set(gca,'fontsize',14)
title('Var4', 'fontsize', 14)
xlabel(ax4, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);

b1 = [6.300711827278427e-10 2.520284730911371e-09 3.780427096367056e-09 2.520284730911371e-09 6.300711827278427e-10];
a1 = [1 -3.973730046565231 5.921534645073681 -3.921876509896709 0.974071921469398];
% [b1,a1]=butter(4,0.0032,'low');       % Coef. función de transferencia del filtro

% while 1
	data = dlmread(full_name);
	var1 = data(:,1);
	var2 = data(:,2);
	var3 = data(:,3);
	var4 = data(:,4);
	sz = length(var1);
	% speed_filt = filtroPasaBajos(var4, 100, [0.1], [0.5]);
	% yaw_filt = filter(b1,a1,var1);
	% speed_ref = filter(b1,a1,var2);
	% current = filter(b1,a1,var3);
	% wheel_speed_filt = filter(b1,a1,var4);

	f = 5000/11;
	dt = 1/f;
	time_data = 0:dt:(length(var4)-1)*dt;

	yaw_filt = var1;
	speed_ref = var2;
	current = var3-0.1;
	wheel_speed_filt = var4;

	subplot(2,2,1);
	% plot(fig1, var1, 'b')
	plot(ax1, time_data, yaw_filt, 'b')
	title('Angulo Yaw', 'fontsize', 10)
	xlabel(ax1, 'Tiempo [s]', 'fontsize', 10); ylabel('Yaw [DEG]', 'fontsize', 10);
	grid on
	% ylim([min_var1-10 max_var1+10])
	ylim([0 3])
	% xlim([sz-window_sz sz+window_sz])
	% hold on
	subplot(2,2,2)
	% plot(fig1, var2, 'r')
	plot(ax2, time_data, speed_ref, 'r')
	title('Angulo Pitch', 'fontsize', 10)
	xlabel(ax2, 'Tiempo [s]', 'fontsize', 10); ylabel('Pitch [DEG]', 'fontsize', 10);
	grid on
	% ylim([min_var2-10 max_var2+10])
	ylim([0 3])
	% xlim([sz-window_sz sz+window_sz])

	subplot(2,2,3)
	% plot(fig1, var2, 'r')
	plot(ax3, time_data, current, 'g')
	title('Corriente', 'fontsize', 10)
	xlabel(ax3, 'Tiempo [s]', 'fontsize', 10); ylabel('Corriente [A]', 'fontsize', 10);
	grid on
	% ylim([min_var3-10 max_var3+10])
	ylim([0 6])
	% xlim([sz-window_sz sz+window_sz])

	subplot(2,2,4)
	% plot(fig1, var2, 'r')
	plot(ax4, time_data, wheel_speed_filt, 'k')
	title('Velocidad Rueda Filtrada', 'fontsize', 10)
	xlabel(ax4, 'Tiempo [s]', 'fontsize', 10); ylabel('Velocidad Rueda [RPM-NC]', 'fontsize', 10);
	grid on
	% ylim([min_var4-10 max_var4+10])
	ylim([0 15000])
	% xlim([sz-window_sz sz+window_sz])
	im_data = current;
	w_w_data = wheel_speed_filt;

	% pause(1)
% end