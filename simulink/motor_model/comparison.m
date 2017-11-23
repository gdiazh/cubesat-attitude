figure(1)
fig1 = axes;
ax1 = subplot(1,2,1);
plot(ax1, time_data, im_data, 'b')
hold on
plot(ax1, time_sim, im_sim, 'r')
grid on
hold on
set(gca,'fontsize',14)
title('Corriente Motor', 'fontsize', 14)
xlabel(ax1, 'Tiempo [s]', 'fontsize', 14); ylabel('Corriente Motor [A]', 'fontsize', 14);
legend('Datos','Modelo')


ax2 = subplot(1,2,2);
plot(ax2, time_data, w_w_data, 'b')
hold on
plot(ax2, time_sim, w_w_sim, 'r')
title('pwm(voltage)', 'fontsize', 14)
xlabel(ax2, 'Tiempo [s]', 'fontsize', 14); ylabel('Velocidad Motor [RPM]', 'fontsize', 14);
legend('Datos','Modelo')