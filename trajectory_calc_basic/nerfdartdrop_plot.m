clc;clear;close all;
%3.28 feet in 1 meter

rho = 1.293;                            %kg * m^-3
c_d = 0.6712;                             %const
A = pi * (13/1000/2)^2;                 %m^2
m = 1.3e-3;                             %kg
a = 0.5 * rho * c_d * A / m;
g = 9.81;                               % gravitational acceleration

v_0 = 300/3.28;                     
%phi = 2; 
phi = 27.4; 
theta = 0;

v_x_0 = v_0 * cos(phi * pi/180) * cos(theta * pi/180);
v_y_0 = v_0 * sin(phi * pi/180) * cos(theta * pi/180);
v_z_0 = v_0 * sin(theta * pi/180);

tspan = linspace(0,5,10000);
[t_sol, x_sol] = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [0, 0, 0, v_x_0, v_y_0, v_z_0]);
x_sol = x_sol * 3.28;

figure;
grid on;
plot(x_sol(:,1), x_sol(:,2))
xlabel('r_x (ft)'); ylabel('r_y (ft)'); ylabel('r_y (ft)')

%norm([(0)/3.28 (0)/3.28])

% figure; 
% subplot(7,1,1); grid on;
% plot3(x_sol(:,1), x_sol(:,2), x_sol(:,3))
% xlabel('r_x (ft)'); ylabel('r_y (ft)'); ylabel('r_y (ft)')
% 
%subplot(7,1,2); grid on;
% plot(t_sol, x_sol(:,1))
% xlabel('time (s)'); ylabel('r_x (ft)')
% 
% subplot(7,1,3); grid on;
% plot(t_sol, x_sol(:,2))
% xlabel('time (s)'); ylabel('r_y (ft)')
% 
% subplot(7,1,4); grid on;
% plot(t_sol, x_sol(:,3))
% xlabel('time (s)'); ylabel('r_z (ft)')
% 
% subplot(7,1,5); grid on;
% plot(t_sol, x_sol(:,4))
% xlabel('time (s)'); ylabel('v_x (ft/s)')
% hold on
% %plot(t_sol, cummean(x_sol(:,4)));
% 
% subplot(7,1,6); grid on;
% plot(t_sol, x_sol(:,5))
% xlabel('time (s)'); ylabel('v_y (ft/s)')
% hold on
% plot(t_sol, cummean(x_sol(:,5)));
% 
% subplot(7,1,7); grid on;
% plot(t_sol, x_sol(:,6))
% xlabel('time (s)'); ylabel('v_z (ft/s)')
% hold on
% plot(t_sol, cummean(x_sol(:,6)));

% subplot(6,1,6); grid on;
% plot(t_sol, atand(x_sol(:,2)./x_sol(:,1)))
% xlabel('time (s)'); ylabel('angle (deg)')

sol = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [0, 0, 0, v_x_0, v_y_0, v_z_0]);



function dxdt = drag_ode_fun(t,x,a,g)
   dxdt = zeros(6,1);
   dxdt(1) = x(4);                                          %r_x(t)
   dxdt(2) = x(5);                                          %r_y(t)
   dxdt(3) = x(6);                                          %r_z(t)
   dxdt(4) = -a.*x(4).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_x(t)
   dxdt(5) = -a.*x(5).*sqrt(x(4).^2+x(5).^2+x(6).^2)-g;     %v_y(t)  
   dxdt(6) = -a.*x(6).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_z(t)
end

function y = cummean(a)
    y = cumsum(a)./(1:numel(a))';
end