function dx = dartdrag(~,x,a)
   %basically cannonDynamics
   dx = zeros(size(x));
   dx(1) = x(4);                                            %r_x(t)
   dx(2) = x(5);                                            %r_y(t)
   dx(3) = x(6);                                            %r_z(t)
   dx(4) = -a.*x(4).*sqrt(x(4).^2+x(5).^2+x(6).^2);         %v_x(t)
   dx(5) = -a.*x(5).*sqrt(x(4).^2+x(5).^2+x(6).^2) - 9.81;  %v_y(t)
   dx(6) = -a.*x(6).*sqrt(x(4).^2+x(5).^2+x(6).^2);         %v_z(t)
end