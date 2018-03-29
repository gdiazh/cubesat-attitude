%/%%
  % @brief simple ypr calculate from quaternion data
  %/
 
% Author: Gustavo Diaz

ypr = [0, 0, 0];
for i=1:length(varA1)
	qS = [varA2(i); varA3(i); varA4(i); varA1(i)];
	ypr(i, :) = q2ypr(qS)*180/pi;
end

yaw = ypr(:,1);
pitch = ypr(:,2);
roll = ypr(:,3);

figure(1)
set(gca,'fontsize',14)

ax1 = subplot(2,2,1);
plot(ax1, varTime, yaw(1:n_time), 'b')
grid on
title('yaw', 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel('yaw', 'fontsize', 14);

ax2 = subplot(2,2,2);
plot(ax2, varTime, pitch(1:n_time), 'r')
grid on
title('pitch', 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel('pitch', 'fontsize', 14);
% legend('Ángulo del Cubo [°]', 'Velocidad del Cubo [°/s]', 'Torque aplicado [Nm]')

ax3 = subplot(2,2,3);
plot(ax3, varTime, roll(1:n_time), 'g')
grid on
title('roll', 'fontsize', 14)
xlabel(ax3, 'Tiempo [s]', 'fontsize', 14); ylabel('roll', 'fontsize', 14);
