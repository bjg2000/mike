clc;clear;close all;
%3.28 feet in 1 meter
%speed of sound 343 m/s or 1125 ft/s

alpha = initParams;

vi = 300/3.28                           %m/s

t_f = 3;
n = 500;

t = linspace(0,t_f,n);

v = @(t) vi ./ (alpha * vi * t + 1);
v_tot = v(t);

%x = @(t) cumtrapz(t_f/n, v_tot);
x = @(t) reallog(1 + vi * alpha * t)/alpha;
x_tot = x(t);

%x_t = integral(v, 0, t_f)*3.28
v_t_f = v_tot(end)*3.28
x_t_f = x_tot(end)*3.28

plot(t, v_tot*3.28)
figure
plot(t, x_tot*3.28)