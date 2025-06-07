clc;clear;close all;
%3.28 feet in 1 meter
%solve with shooting method for boundary value problems:
% r_x(t_h) = r_x_target, r_y(t_h) = r_y_target

rho = 1.293;                            %kg * m^-3
c_d = 0.6712;%0.6212;                           %const
A = pi * (13/1000/2)^2;                 %m^2
m = 1.3e-3;                             %kg
a = 0.5 * rho * c_d * A / m;
g = 9.81;                               %gravitational acceleration

v_0_mag = 150/3.28;                         %muzzle velocity
phi = 0;        
theta = 0;

r_x_0 = 0/3.28;
r_y_0 = 0/3.28;
r_z_0 = 0/3.28;

r_x_target = (100 + .01)/3.28;
r_y_target = (0 + .01)/3.28;
r_z_target = (0 + .01)/3.28;

v_x_target = 0/3.28;
v_y_target = 0/3.28;
v_z_target = 25/3.28;

tspan = linspace(0, 4, 200);

tol_m = 3/12/3.28;
r_y_err = tol_m;
r_z_err = tol_m;

i=0;
while (abs(r_y_err) >= tol_m) || (abs(r_z_err) >= tol_m)

v_x_0 = v_0_mag * cos(phi * pi/180) * cos(theta * pi/180);
v_y_0 = v_0_mag * sin(phi * pi/180) * cos(theta * pi/180);
v_z_0 = v_0_mag * sin(theta * pi/180);
[t_sol, x_sol] = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [r_x_0, r_y_0, 0, v_x_0, v_y_0, v_z_0]);

    [y, idx_y] = closestValue(x_sol(:,1), r_x_target);
    [z, idx_z] = closestValue(x_sol(:,1), r_z_target);

    % if (abs(x_sol(idx,1) - r_x_target) > tol_m)
    %     fprintf("Target out of range!\n");
    %     return
    % end

    r_y_err = x_sol(idx_y,2) - r_y_target/t_sol(idx_y);
    r_z_err = x_sol(idx_z,3) - r_z_target/t_sol(idx_z);

    %try to converge on a projectile launch angle
    phi = phi - atand(r_y_err/r_x_target);
    theta = theta - atand(r_z_err/sqrt(r_x_target^2 + r_y_target^2));

    abs(r_y_err)
    abs(r_z_err)

    i=i+1;

    if (i > 10)
        if (abs(x_sol(idx_y,1) - r_x_target) > tol_m || abs(x_sol(idx_z,1) - r_x_target) > tol_m)
            fprintf("Target out of range!\n");
            return
        end
        fprintf("Reached max iterations, stopping...\n");
        return
    end
end

r_y_err*3.28*12
r_z_err*3.28*12

i

x_sol = x_sol * 3.28;

figure; 
subplot(2,1,1); grid on; axis equal;
plot3(x_sol(:,1), x_sol(:,2), x_sol(:,3))
xlabel('r_x (ft)'); ylabel('r_y (ft)'); zlabel('r_z (ft)');

subplot(2,1,2); grid on;
plot(t_sol, sqrt(x_sol(:,4).^2 + x_sol(:,5).^2 + x_sol(:,6).^2));
xlabel('time (s)'); ylabel('v (ft/s)')

figure
subplot(3,1,1); grid on;
plot(t_sol, x_sol(:,1))
xlabel('time (s)'); ylabel('r_x (ft)')

subplot(3,1,2); grid on;
plot(t_sol, x_sol(:,2))
xlabel('time (s)'); ylabel('r_y (ft)')

subplot(3,1,3); grid on;
plot(t_sol, x_sol(:,3))
xlabel('time (s)'); ylabel('r_z (ft)')

figure
subplot(3,1,1); grid on;
plot(t_sol, x_sol(:,4))
xlabel('time (s)'); ylabel('v_x (ft/s)')
% hold on
% plot(t_sol, v_x_avg)
% plot(t_sol, v_x_target_avg)
% hold off
% ylim([-300 300])

subplot(3,1,2); grid on;
plot(t_sol, x_sol(:,5))
xlabel('time (s)'); ylabel('v_y (ft/s)')
% hold on
% plot(t_sol, v_y_avg)
% plot(t_sol, v_y_target_avg)
% hold off

subplot(3,1,3); grid on;
plot(t_sol, x_sol(:,6))
xlabel('time (s)'); ylabel('v_z (ft/s)')
% hold on
% plot(t_sol, v_z_avg)
% plot(t_sol, v_z_target_avg)
% hold off

% function dxdt = drag_ode_fun(t,x,a,g)
%    dxdt = zeros(4,1);
%    dxdt(1) = x(3);                                  %r_x(t)
%    dxdt(2) = x(4);                                  %r_y(t)
%    dxdt(3) = (-a*x(3)*sqrt(x(3)^2+x(4)^2));         %v_x(t)
%    dxdt(4) = (-a*x(4)*sqrt(x(3)^2+x(4)^2)-g);       %v_y(t)         
% end

function dxdt = drag_ode_fun(t,x,a,g)
   dxdt = zeros(6,1);
   dxdt(1) = x(4);                                          %r_x(t)
   dxdt(2) = x(5);                                          %r_y(t)
   dxdt(3) = x(6);                                          %r_z(t)
   dxdt(4) = -a.*x(4).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_x(t)
   dxdt(5) = -a.*x(5).*sqrt(x(4).^2+x(5).^2+x(6).^2)-g;     %v_y(t)  
   dxdt(6) = -a.*x(6).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_z(t)
end

function [y, idx] = closestValue(f, n)
    [val, idx]=min(abs(f-n));
    y=f(idx);
end