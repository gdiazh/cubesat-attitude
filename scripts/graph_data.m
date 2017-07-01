%/%%
  % @brief Continuously plot file data
  %/
 
% Author: Gustavo Diaz

close all;
clc

file_path = 'data/';
file_name = input('file_name: ','s');
file_ext = '.txt';
full_name = strcat(file_path, file_name, file_ext);

var1 = 0;
var2 = 0;

max_var1 = 0;
min_var1 = 0;
max_var2 = 0;
min_var2 = 0;

sz = 0;
window_sz = 1000;

figure(1)
fig1 = axes;
ax1 = subplot(2,1,1);
plot(ax1, var1, 'b')
grid on
set(gca,'fontsize',14)
title('Var1', 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);
% hold on
% figure(2)
% fig2 = axes;
ax2 = subplot(2,1,2);
plot(ax2, var2, 'r')
grid on
set(gca,'fontsize',14)
title('Var2', 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel('Variable', 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

while 1
	data = dlmread(full_name);
	var1 = data(:,1);
	var2 = data(:,2);
	sz = length(var1);

	if (max(var1)>=max_var1)
		max_var1 = max(var1);
	end
	if (min(var1)<=min_var1)
		min_var1 = min(var1);
	end
	if (max(var2)>=max_var2)
		max_var2 = max(var2);
	end
	if (min(var2)<=min_var2)
		min_var2 = min(var2);
	end

	subplot(2,1,1);
	% plot(fig1, var1, 'b')
	plot(ax1, var1, 'b')
	ylim([min_var1-10 max_var1+10])
	xlim([sz-window_sz sz+window_sz])
	% hold on
	subplot(2,1,2)
	% plot(fig1, var2, 'r')
	plot(ax2, var2, 'r')
	ylim([min_var2-10 max_var2+10])
	xlim([sz-window_sz sz+window_sz])
	pause(1)
end