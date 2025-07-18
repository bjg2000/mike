clc; clear; close all
rho = 1.293;                            %kg * m^-3
c_d = 0.6712;                           %const
A = pi * (13/1000/2)^2;                 %m^2
m = 1.3e-3;                             %kg
alpha = 0.5 * rho * c_d * A / m;        %drag function constant
arrsize = 100;                          %grid discretization  

% Set the target (assuming that the trajectory starts at x=0, y=0, z=0)
target.x0 = (150)/3.28;  
target.y0 = (0)/3.28;
target.z0 = (0)/3.28;

target.dx = (0)/3.28;
target.dy = (0)/3.28;
target.dz = 100/11.61;%(22)/3.28;

%set the initial projectile velocity
init.speed = 300/3.28;

%init guesses
init.anglephi = 2*pi/180;       %2 degrees for ranges 100 feet or less on level ground at 300fps
init.angletheta = pi/180;

%start timer
tic

% traj = simulateBlaster(init,params);
% guess.theta = atan(traj.dy(1)/traj.dx(1));
% guess.phi = atan(traj.dz(1)/traj.dx(1));
% guess.T = traj.t(end);  %Trajectory duration

guess.phi = init.anglephi; 
guess.theta = init.angletheta; 
guess.t = 1;

%pack constants
params.c = alpha;                       
params.nGrid = arrsize;  
params.g = 9.81;

% Set up the decision variables and bounds:
problem.x0 = [guess.phi; guess.theta; guess.t];
problem.objective = @(decVar)objective(decVar(3));  %Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar,target,params,init.speed);   %NonLinear constraints

% Set up the options for the solver:
problem.solver = 'fmincon';
problem.options = optimset(...
    'MaxIter',50,...
    'Display','off');

%Use FMINCON to solve the constrained optimization problem:
[sSoln, fVal, exitFlag] = fmincon(problem);

%Call the constraint function one final time to get the trajectory:
[~, ~, t, sTraj] = nonLinCst(sSoln,target,params,init.speed);

%end timer and get time to solve
t_solve = toc

% Store the trajectory in a nice format:
soln.t = t;
soln.x = sTraj(1,:);
soln.y = sTraj(2,:);
soln.z = sTraj(3,:);
soln.dx = sTraj(4,:);
soln.dy = sTraj(5,:);
soln.dz = sTraj(6,:);

soln.success = exitFlag;
soln.cost = fVal;
soln.method = 'Single Shooting';

plot3(soln.x*3.28, soln.y*3.28, soln.z*3.28)
xlabel('r_x (ft)'); ylabel('r_y (ft)'); zlabel('r_z (ft)');

target_out_of_range = ~soln.success
phi_launch = mod(sSoln(1), pi/2 * sSoln(1)/abs(sSoln(1)))*180/pi
theta_launch = mod(sSoln(2), pi/2 * sSoln(2)/abs(sSoln(2)))*180/pi
t_hit = sSoln(3)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for single shooting            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq, t, s] = nonLinCst(decVar,target,params,v0)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are implemented using a single simulation
% The initial boundary conditions are directly enforced, and the final
% boundary conditions are enforced via the nonlinear constraint: Ceq
% The simulation is implemented using 4th-order runge-kutta*.

x0 = 0; y0 = 0; z0 = 0; %Trajectory starts at the origin

dx0 = v0 * cos(decVar(1)) * cos(decVar(2));
dy0 = v0 * sin(decVar(1)) * cos(decVar(2));
dz0 = v0 * sin(decVar(2));

T = decVar(3);  %Unpack decision variables

s0 = [x0;y0;z0;dx0;dy0;dz0];  %Assemble initial dynamical state for simulation

t = linspace(0,T,params.nGrid);  %Build grid in time

s = rk4_blaster(t,s0,params.c,params.g); %Simulate the trajectory

xFinal = s(1,end);
yFinal = s(2,end);
zFinal = s(3,end);

target.x = target.x0 + target.dx*T;
target.y = target.y0 + target.dy*T;
target.z = target.z0 + target.dz*T;

C = [];  %No inequality constraints
Ceq = [xFinal - (target.x); yFinal - (target.y); zFinal - (target.z)];  %Boundary Condition

end