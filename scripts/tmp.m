% plot(varTime, yaw(1:n_time), 'b')
% grid on
% title('Ángulo yaw (desde qS) prueba de equilibrio', 'fontsize', 14)
% xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Ángulo [°] ', 'fontsize', 14);

% plot(varTime, pitch(1:n_time), 'b')
% grid on
% title('Ángulo pitch (desde qS) prueba de equilibrio', 'fontsize', 14)
% xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Ángulo [°] ', 'fontsize', 14);

plot(varTime, roll(1:n_time), 'b')
grid on
title('Ángulo roll (desde qS) prueba de equilibrio', 'fontsize', 14)
xlabel('Tiempo [s]', 'fontsize', 14); ylabel('Ángulo [°] ', 'fontsize', 14);