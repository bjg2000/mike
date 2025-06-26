function traj = simulateBlaster(init,param)
%basically simulateCannon
v0 = init.speed;
ph0 = init.anglephi;
th0 = init.angletheta;

c = param.c;  %Quadratic drag coefficient
nGrid = param.nGrid;

% Set up initial conditions for ode45
x0 = 0;  y0 = 0; z0 = 0; %Start at the origin

dx0 = v0 * cos(ph0) * cos(th0);
dy0 = v0 * sin(ph0) * cos(th0);
dz0 = v0 * sin(th0);

if dy0 < 0, error('Cannot point blaster through ground! sin(ph0) > 0 required.'); end;

% Set up arguments for ode45
userFun = @(t,r)dartdrag(t,r,c);  %Dynamics function
tSpan = [0,100];  %Never plan on reaching final time
s0 = [x0;y0;z0;dx0;dy0;dz0];  %Initial condition
options = odeset('Events',@groundEvent,'Vectorized','on');

% Run a simulation
sol = ode45(userFun, tSpan, s0, options);

% Extract the solution on uniform grid:
traj.t = linspace(sol.x(1), sol.x(end), nGrid);
s = deval(sol,traj.t);
traj.x = s(1,:); 
traj.y = s(2,:);
traj.z = s(3,:); 
traj.dx = s(4,:); 
traj.dy = s(5,:); 
traj.dz = s(6,:); 

end