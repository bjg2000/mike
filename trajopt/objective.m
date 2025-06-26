% function cost = objective(dx,dy)
% % Objective function for optimization. Here we set the objective function
% % to be proportional to the initial energy of the nerf dart.
% 
% cost = dx.*dx + dy.*dy;
% 
% 
% end

function cost = objective(t)
% Objective function for optimization. Here we set the objective function
% to be proportional to the initial energy of the nerf dart.

cost = t;

end