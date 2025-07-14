function s = rk4_blaster(t,s0,a,g)

    nState = 6;
    nTime = length(t);

    s = zeros(nState,nTime);
    s(:,1) = s0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = dartdrag(s(:,i),a,g);
        k2 = dartdrag(s(:,i) + 0.5*dt*k1,a,g);
        k3 = dartdrag(s(:,i) + 0.5*dt*k2,a,g);
        k4 = dartdrag(s(:,i) + dt*k3,a,g);
        s(:,i+1) = s(:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end
end