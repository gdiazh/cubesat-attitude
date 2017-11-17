rpm = [1570, 2607, 3445, 4101, 4701, 5107, 5467, 6000, 6582, 7129, 7320, 7301];		%Tacometro
rpm_nc = [1611, 2637, 3575, 4508, 4740, 5712, 6265, 7267, 8779, 9400, 9554, 9372];	%Hall
pwm_cmd = [1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000];


rpm_c = rpm_nc*0.68+870;

figure(1)
plot(pwm_cmd, rpm, 'bo-')
hold on
plot(pwm_cmd, rpm_nc, 'ro-')
plot(pwm_cmd, rpm_c, 'g*-')
grid on
hold on
set(gca,'fontsize',14)
title('Calibración medicón de Velocidad', 'fontsize', 14)
xlabel('Ciclo de trabajo PWM [ms]', 'fontsize', 14); ylabel('Velocidad [RPM]', 'fontsize', 14);
legend('Tacómetro','Sensor Efecto Hall', 'Sensor Efecto Hall Calibrado')