s = tf('s');
currentSys = 1/(2*L*s+2*R)
pidTuner(currentSys, 'PI')
Kp_current = 13.961;
Ki_current=10;