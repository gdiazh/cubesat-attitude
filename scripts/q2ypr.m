function [ypr]  = q2ypr(q)
    %%
    %Inputs
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);  %scalar part
    %%
    %Calcs
    roll = atan2(2*(q4*q1+q2*q3), 1-2*(q1^2+q2^2));
    pitch = asin(2*(q4*q2-q3*q1));
    yaw = atan2(2*(q4*q3+q1*q2), 1-2*(q2^2+q3^2));
    %%
    %Ouputs
    ypr = [yaw pitch roll];
end