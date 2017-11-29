%Update real data
close all;
file_path = '../../scripts/motor_model_data/';
file_name = 'attitude_control_test3';
file_ext = '.txt';
full_name = strcat(file_path, file_name, file_ext);

data = dlmread(full_name);
attitude_reference = data(:,1);
attitude_feedback = data(:,2);
torque_control = data(:,3);
speed_feedback = data(:,4);

f = 5000/11;
dt = 1/f;
time_data = 0:dt:(length(attitude_feedback)-1)*dt;

figure(1)
plot(time_data, attitude_feedback, 'r'); hold on;
plot(time_data, attitude_reference, 'b'); grid on;
set(gca,'fontsize',14); title('Control de Orientación', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Orientación [deg]', 'fontsize', 14);
legend('Salida', 'Referencia')

figure(2)
plot(time_data, torque_control, 'r'); grid on;
set(gca,'fontsize',14); title('Torque de Control', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Torque [Nm]', 'fontsize', 14);

figure(3)
plot(time_data, speed_feedback, 'r'); grid on;
set(gca,'fontsize',14); title('Velocidad', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Velocidad [RPM]', 'fontsize', 14);