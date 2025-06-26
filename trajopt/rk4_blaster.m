function s = rk4_blaster(t,s0,a)
% z = rk4_blaster(dyn,t,z0,a)
%
% This function is used to perform a 4th-order Runge-Kutta
% integration of the cannon dynamical system
%
% INPUTS:
%   t = [1 x nTime] vector of times, created by linspace
%   z0 = [nState (x nSim)] matrix of initial states
%   P = parameter to pass to dynamics
%
% OUTPUTS:
%   z = [nState (x nSim) x nTime ] matrix of trajectories
%
% NOTES:
%   1) See rk4.m for the generic version of this function
%   2) I've hard-coded the dynamics function into this version because it
%      is faster than calling the anonymous function with an embedded
%      parameter.
%

[nState,nSim] = size(s0);
nTime = length(t);

if nSim==1
    s = zeros(nState,nTime);
    s(:,1) = s0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = dartdrag(t(i),  s(:,i),a);
        k2 = dartdrag(t(i)+0.5*dt,  s(:,i) + 0.5*dt*k1,a);
        k3 = dartdrag(t(i)+0.5*dt,  s(:,i) + 0.5*dt*k2,a);
        k4 = dartdrag(t(i)+dt,  s(:,i) + dt*k3,a);
        s(:,i+1) = s(:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end
else
    s = zeros(nState,nSim,nTime);
    s(:,:,1) = s0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = dartdrag(t(i),  s(:,:,i),a);
        k2 = dartdrag(t(i)+0.5*dt,  s(:,:,i) + 0.5*dt*k1,a);
        k3 = dartdrag(t(i)+0.5*dt,  s(:,:,i) + 0.5*dt*k2,a);
        k4 = dartdrag(t(i)+dt,  s(:,:,i) + dt*k3,a);
        s(:,:,i+1) = s(:,:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end    
end

end