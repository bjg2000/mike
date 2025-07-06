function s = rk4_blaster(t,s0,a)

    [nState,nSim] = size(s0);
    nTime = length(t);

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
end