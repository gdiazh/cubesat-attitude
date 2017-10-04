clc
close all
%Constants
Ts = 0.01;  %[s]
Fs = 1/Ts %[Hz]

%%----------------------------------------------------------------------
%--------------- Speed -------------------------------------------------
%%----------------------------------------------------------------------
speed_value = var4;
speed_time = (1:length(var4))';
%Graph in time and frecuency of wheel speed
speed = struct('signal', speed_value,'time', speed_time);
[speed_f, speed_S] = Fourier(speed.signal, Fs);
graficarEnTyF(speed.time, speed.signal, speed_f, speed_S, 'Wheel speed NOT Filtered');
%Low-pass filter
speedFiltered_t = filtroPasaBajos(speed.signal, Fs, [0.1], [0.5]);
%Graph in time and frecuency of wheel speed filtered
[speedFiltered_f, speedFiltered_S] = Fourier(speedFiltered_t, Fs);
graficarEnTyF(speed.time, speedFiltered_t, speedFiltered_f, speedFiltered_S, 'Wheel speed Filtered');