function [value, isterminal, direction] = groundEvent(~,s)
%
% This function is called by ode45 event-detection to determine when a
% collision with the ground occurs.
%

height = s(2,:);  %Vertical position of the nerf dart

value = height;  
isterminal = true;
direction = -1;

end