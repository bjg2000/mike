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

%convergence testing
figure
hold on
%convergence testing

tic
for i=1:10

    v_x_0 = v_0 * cos(phi * pi/180) * cos(theta * pi/180);
    v_y_0 = v_0 * sin(phi * pi/180) * cos(theta * pi/180);
    v_z_0 = v_0 * sin(theta * pi/180);

    for j = 1:5
        tspan = [t_min tn t_max];

        [t_sol, x_sol] = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [0, 0, 0, v_x_0, v_y_0, v_z_0]);

        target_v_x = target_v_x_0;
        target_v_y = target_v_y_0;
        target_v_z = target_v_z_0;
        target_r_x = target_r_x_0 + target_v_x_0*tn;

        tn = tn - (x_sol(2, 1) - target_r_x)/(x_sol(2, 4) - target_v_x);

        if ((tn > t_max))
            %target is out of range, velocity component must be increased
            fprintf("Target out of range\n");
        end
    end

    target_r_x = target_r_x_0 + target_v_x*tn;
    target_r_y = target_r_y_0 + target_v_y*tn;
    target_r_z = target_r_z_0 + target_v_z*tn;

    target_v_x = target_v_x_0;
    target_v_y = target_v_y_0;
    target_v_z = target_v_z_0;

    %convergence testing
    [tt, xx] = ode45(@(t,x) drag_ode_fun(t,x,a,g), [t_min, t_max], [0, 0, 0, v_x_0, v_y_0, v_z_0]);
    plot(xx(:,1)*3.28, xx(:,2)*3.28)
    plot(target_r_x*3.28, target_r_y*3.28, 'o')
    %convergence testing

end

xpos = x_sol(2, 1)*3.28
ypos = x_sol(2, 2)*3.28
zpos = x_sol(2, 3)*3.28
tn

%convergence testing
hold off
%convergence testing

%calculate final trajectory
v_x_0 = v_0 * cos(phi * pi/180) * cos(theta * pi/180);
v_y_0 = v_0 * sin(phi * pi/180) * cos(theta * pi/180);
v_z_0 = v_0 * sin(theta * pi/180);

tspan = [t_min t_max];
[t_sol, x_sol] = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [0, 0, 0, v_x_0, v_y_0, v_z_0]);

timetorun = toc

% tspan = linspace(0, tn);
% [t_sol, x_sol] = ode45(@(t,x) drag_ode_fun(t,x,a,g), tspan, [0, 0, 0, v_x_0, v_y_0, v_z_0]);
% v = [x_sol(:, 4) x_sol(:, 5) x_sol(:, 6)];
% target_v_x_avg = target_r_x_0/tn + target_v_x_0;
% target_a_x_avg = -target_r_x_0/(tn^2) + target_v_x_0/tn;

% dart_r_x = x_sol(2, 1) * 3.28
% dart_v_x_avg = x_sol(2, 1)/tn * 3.28
% target_v_x = target_v_x * 3.28

figure; 
subplot(7,1,1); grid on;
plot3(x_sol(:,1)*3.28, x_sol(:,2)*3.28, x_sol(:,3)*3.28)
xlabel('r_x (ft)'); ylabel('r_y (ft)'); ylabel('r_y (ft)')

subplot(7,1,2); grid on;
plot(t_sol, x_sol(:,1)*3.28)
xlabel('time (s)'); ylabel('r_x (ft)')

subplot(7,1,3); grid on;
plot(t_sol, x_sol(:,2)*3.28)
xlabel('time (s)'); ylabel('r_y (ft)')

subplot(7,1,4); grid on;
plot(t_sol, x_sol(:,3)*3.28)
xlabel('time (s)'); ylabel('r_z (ft)')

function dxdt = drag_ode_fun(t,x,a,g)
   dxdt = zeros(6,1);
   dxdt(1) = x(4);                                          %r_x(t)
   dxdt(2) = x(5);                                          %r_y(t)
   dxdt(3) = x(6);                                          %r_z(t)
   dxdt(4) = -a.*x(4).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_x(t)
   dxdt(5) = -a.*x(5).*sqrt(x(4).^2+x(5).^2+x(6).^2)-g;     %v_y(t)  
   dxdt(6) = -a.*x(6).*sqrt(x(4).^2+x(5).^2+x(6).^2);       %v_z(t)
end