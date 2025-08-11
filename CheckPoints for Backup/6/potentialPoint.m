 function [U, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, point)
%       INPUTS:
%       map         map of the environment defined by circles. First row is the map boundary.
%                   k x 3 matrix [x_center y_center radius]
%       goal        goal point
%                   1 x 2 array [x_goal y_goal]
%       c_att       scaling factor for the atractive force (potential fun).
%       c_rep       scaling factor for the repulsive force (potential fun).
%       Q           influence range of the obstacles (real number)
%       point       a point to be evaluated [x, y],i.e., the center location of the robot
%       OUTPUTS:
%       U           Potential value at point
%       U_grad      1 x 2 array [gradient-x gradient-y] gradient of the potential function at point
% 
% 
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A


% Attractive Potential
diff_goal = point - goal;  % vector from goal to point
U_att = 0.5 * c_att * norm(diff_goal)^2;
grad_U_att = c_att * diff_goal;

% Initialize Repulsive Potential
U_rep = 0;
grad_U_rep = [0, 0];

% Loop over obstacles (skip first row which is boundary)
for i = 2:size(map,1)
    obs_center = map(i, 1:2);
    obs_radius = map(i, 3);
    
    diff_obs = point - obs_center;
    dist_to_obs = norm(diff_obs) - obs_radius;
    
    if dist_to_obs <= Q && dist_to_obs > 0
        % Only if within influence range and not inside obstacle
        U_rep = U_rep + 0.5 * c_rep * (1/dist_to_obs - 1/Q)^2;
        grad_U_rep = grad_U_rep + c_rep * (1/Q - 1/dist_to_obs) * (1/(dist_to_obs^2)) * (diff_obs / norm(diff_obs));
    elseif dist_to_obs <= 0
        % Inside obstacle: very high potential
        U_rep = U_rep + 0.5 * c_rep * (1/0.001 - 1/Q)^2;
        grad_U_rep = grad_U_rep + [1e6, 1e6];  % Huge repulsion
    end
end

% Total Potential and Gradient
U = U_att + U_rep;
U_grad = grad_U_att + grad_U_rep;

 
end
 
 