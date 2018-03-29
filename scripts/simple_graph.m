%/%%
  % @brief simple plot csv data
  %/
 
% Author: Gustavo Diaz

close all;
clc;

file_path = 'jp-5feb/';
file_ext = '.txt';

file_name = input('file_name: ','s');

full_name1 = strcat(file_path, file_name, '_1', file_ext);
full_name2 = strcat(file_path, file_name, '_2', file_ext);

dataA = dlmread(full_name1);
dataB = dlmread(full_name2);

varA1 = dataA(:,1);
varA2 = dataA(:,2);
varA3 = dataA(:,3);
varA4 = dataA(:,4);
varB1 = dataB(:,1);
varB2 = dataB(:,2);
varB3 = dataB(:,3);
varB4 = dataB(:,4);

%Data meaning
varA1_name = 'qw';
varA2_name = 'q1';
varA3_name = 'q2';
varA4_name = 'q3';
varB1_name = 'time';
varB2_name = 'current Mx';
varB3_name = 'current My';
varB4_name = 'speed Mx';

figure(1)
set(gca,'fontsize',14)

ax1 = subplot(2,2,1);
plot(ax1, varA1, 'b')
grid on
title(varA1_name, 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel(varA1_name, 'fontsize', 14);

ax2 = subplot(2,2,2);
plot(ax2, varA2, 'r')
grid on
title(varA2_name, 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel(varA2_name, 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

ax3 = subplot(2,2,3);
plot(ax3, varA3, 'g')
grid on
title(varA3_name, 'fontsize', 14)
xlabel(ax3, 'Tiempo [s]', 'fontsize', 14); ylabel(varA3_name, 'fontsize', 14);

ax4 = subplot(2,2,4);
plot(ax4, varA4, 'k')
grid on
title(varA4_name, 'fontsize', 14)
xlabel(ax4, 'Tiempo [s]', 'fontsize', 14); ylabel(varA4_name, 'fontsize', 14);

% --------------------------------------------------------------------------------

figure(2)
set(gca,'fontsize',14)

ax1 = subplot(2,2,1);
plot(ax1, varB1, 'b')
grid on
title(varB1_name, 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel(varB1_name, 'fontsize', 14);

ax2 = subplot(2,2,2);
plot(ax2, varB2, 'r')
grid on
title(varB2_name, 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel(varB2_name, 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

ax3 = subplot(2,2,3);
plot(ax3, varB3, 'g')
grid on
title(varB3_name, 'fontsize', 14)
xlabel(ax3, 'Tiempo [s]', 'fontsize', 14); ylabel(varB3_name, 'fontsize', 14);

ax4 = subplot(2,2,4);
plot(ax4, varB4, 'k')
grid on
title(varB4_name, 'fontsize', 14)
xlabel(ax4, 'Tiempo [s]', 'fontsize', 14); ylabel(varB4_name, 'fontsize', 14);

% --------------------------------------------------------------------------------

figure(3)
set(gca,'fontsize',14)
N_tmax1 = 11439;
N_tmax2 = 11440;
N_tmax3 = 30680;
N_tmax4 = 30690;
N_tmax5 = 47420;
N_tmax6 = 47440;
N_tmax7 = 63960;
N_tmax8 = 63970;
N_tmax9 = 67640;
varTime1 = varB1(1:N_tmax1)/1000;
varTime2 = varB1(N_tmax2:N_tmax3)/1000 + varTime1(end);
varTime3 = varB1(N_tmax4:N_tmax5)/1000 + varTime2(end);
varTime4 = varB1(N_tmax6:N_tmax7)/1000 + varTime3(end);
varTime5 = varB1(N_tmax8:N_tmax9)/1000 + varTime4(end);
varTime = [varTime1; varTime2; varTime3; varTime4; varTime5];
n_time = length(varTime);
% plot(varTime)

ax1 = subplot(2,2,1);
plot(ax1, varTime, varA1(1:n_time), 'b')
grid on
title(varA1_name, 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel(varA1_name, 'fontsize', 14);

ax2 = subplot(2,2,2);
plot(ax2, varTime, varA2(1:n_time), 'r')
grid on
title(varA2_name, 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel(varA2_name, 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

ax3 = subplot(2,2,3);
plot(ax3, varTime, varA3(1:n_time), 'g')
grid on
title(varA3_name, 'fontsize', 14)
xlabel(ax3, 'Tiempo [s]', 'fontsize', 14); ylabel(varA3_name, 'fontsize', 14);

ax4 = subplot(2,2,4);
plot(ax4, varTime, varA4(1:n_time), 'k')
grid on
title(varA4_name, 'fontsize', 14)
xlabel(ax4, 'Tiempo [s]', 'fontsize', 14); ylabel(varA4_name, 'fontsize', 14);