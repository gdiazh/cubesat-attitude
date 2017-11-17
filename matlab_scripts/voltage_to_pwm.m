pwm_cmd = [1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000];
v_rms = [2.21, 2.74, 3.08, 3.39, 3.64, 3.86, 4.03, 4.26, 4.48, 4.70, 4.64, 4.60];
x1 = linspace(0,5);
x2 = linspace(1400,2000);

p2 = polyfit(pwm_cmd, v_rms, 2);
y2 = polyval(p2,x2);

figure(1)
fig1 = axes;
ax1 = subplot(1,2,1);
plot(ax1, pwm_cmd, v_rms, 'bo-')
hold on
plot(ax1, x2, y2, 'r')
grid on
hold on
set(gca,'fontsize',14)
title('voltage(pwm)', 'fontsize', 14)
xlabel(ax1, 'Ciclo de trabajo PWM [ms]', 'fontsize', 14); ylabel('Voltage [V]', 'fontsize', 14);
legend('Datos','Ajuste')

p = polyfit(v_rms, pwm_cmd, 2);
y1 = polyval(p,x1);

ax2 = subplot(1,2,2);
plot(ax2, v_rms, pwm_cmd, 'bo-')
hold on
plot(ax2, x1, y1, 'r')
title('pwm(voltage)', 'fontsize', 14)
xlabel(ax2, 'Voltage [V]', 'fontsize', 14); ylabel('Ciclo de trabajo PWM [ms]', 'fontsize', 14);
legend('Datos','Ajuste')
fprintf('Coeficientes de ajuste polinomial:%5.4f\n', p);