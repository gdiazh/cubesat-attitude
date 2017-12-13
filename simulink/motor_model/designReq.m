wn_current = 16.3520;	%[Hz]
coef_amort = 0.8;	%[]
ts_current = 0.4/(wn_current*coef_amort)
Mp = 100*exp(-(pi*coef_amort)/(sqrt(1-coef_amort^2)))