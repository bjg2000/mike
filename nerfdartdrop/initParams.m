function alpha = initParams()
%3.28 feet in 1 meter
%speed of sound 343 m/s or 1125 ft/s

rho = 1.293;                            %kg * m^-3
c_d = 0.6212;                           %const
A = pi * (13/1000/2)^2;                 %m^2
m = 1.3e-3;                             %kg
%m = 2.6e-3;
alpha = 0.5 * rho * c_d * A / m;