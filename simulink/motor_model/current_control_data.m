%Update real data
file_path = '../../scripts/motor_model_data/';
file_name = 'current_control_test2';
file_ext = '.txt';
full_name = strcat(file_path, file_name, file_ext);

data = dlmread(full_name);
current_reference = data(:,1);
current_feedback = data(:,2);
voltage_control = data(:,3);
speed_openloop = data(:,3);

f = 5000/11;
dt = 1/f;
time_data = 0:dt:(length(current_reference)-1)*dt;

figure(1)
plot(time_data, current_feedback, 'r'); hold on;
plot(time_data, current_reference, 'b'); grid on;
set(gca,'fontsize',14); title('Control de Corriente', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Corriente [A]', 'fontsize', 14);
legend('Salida', 'Referencia')

figure(2)
plot(time_data, voltage_control, 'r'); grid on;
set(gca,'fontsize',14); title('Voltaje de Control', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Voltaje [V]', 'fontsize', 14);