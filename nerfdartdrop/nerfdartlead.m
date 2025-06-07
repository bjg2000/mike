clc;clear;close all;
%3.28 feet in 1 meter
%speed of sound 343 m/s or 1125 ft/s

alpha = initParams;
vi = 300/3.28                   %m/s

v_target = 25/3.28;             %m/s
y_range = 200/3.28;             %m
theta = -90:90;

t_f = 5;
n = 1;

f = @(theta) vi/v_target * alpha*y_range*tand(theta) ./ (exp(alpha * y_range ./ cosd(theta)) - 1) - 1;
df = @(theta, h) (f(theta + h) - f(theta - h))/(2*h);

%we want angle closest to 0
theta_n = 0;

for i=1:5
    theta_n = theta_n - f(theta_n)/df(theta_n, 1/n);
end
theta_n

plot(theta, f(theta))

%t = linspace(0,t_f,n);

%we want angle closest to 0
theta_lead = fzero(f, 0)

t_to_target = (exp(alpha * y_range / cosd(theta_lead)) - 1) / (vi*alpha)
v_dart = @(t) vi ./ (alpha * vi * t + 1);

theta = theta_lead
pos_dart_t_final = integral(v_dart, 0, t_to_target)*3.28
pos_dart_t_final_y = integral(v_dart, 0, t_to_target)*cosd(theta_lead)*3.28
pos_dart_t_final_x = integral(v_dart, 0, t_to_target)*sind(theta_lead)*3.28



