vt2_i = [2.00, 2.31, 2.62, 2.93, 3.24, 3.55, 3.86, 4.17, 4.48, 4.80];	%[V]
im2_i = [0.08, 0.085, 0.1, 0.12, 0.13, 0.16, 0.19, 0.22, 0.31, 0.33];	%[A]
ww2_i = [1740, 1931, 2584, 3289, 3798, 4760, 5581, 6608, 8715, 9344]*(2*pi/60);	%[rad/s]

KM2_i = zeros(1, length(vt2_i));

for i=1:length(vt2_i)
	KM2_i(i) = (vt2_i(i)-R*im2_i(i))/ww2_i(i);
end

KM2_estimated = mean(KM2_i)

B2_i = zeros(1, length(vt2_i));

for i=1:length(vt2_i)
	B2_i(i) = (KM2_estimated*im2_i(i))/ww2_i(i);
end

B2_estimated = mean(B2_i)