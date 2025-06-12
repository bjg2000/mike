clc;clear;close all;
%3.28 feet in 1 meter

rho = 1.293;                            %kg * m^-3
c_d = 0.6712;                             %const
A = pi * (13/1000/2)^2;                 %m^2
m = 1.3e-3;                             %kg
a = 0.5 * rho * c_d * A / m;
g = 9.81;                               % gravitational acceleration

v_0 = 300/3.28;                    
phi = 0;                                
theta = 0;

target_r_x_0 = 50/3.28;
target_r_y_0 = 40/3.28;
target_r_z_0 = 0/3.28;

target_v_x_0 = 0/3.28;
target_v_y_0 = 0/3.28;
target_v_z_0 = 0/3.28;

t_min = 0;
t_max = 5;

tn = 0;

J_cost = 

function dxdt = drag_ode_fun(t,x,a,g)
   dxdt = zeros(6,1);
   dxdt(1) = x(4);                                          %r_x(t)
   dxdt(2) = x(5);                                          %r_y(t)
   dxdt(3) = x(6);                                          %r_z(t)
   dxdt(4) = -a.*x(4).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_x(t)
   dxdt(5) = -a.*x(5).*sqrt(x(4).^2+x(5).^2+x(6).^2)-g;     %v_y(t)  
   dxdt(6) = -a.*x(6).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_z(t)
end