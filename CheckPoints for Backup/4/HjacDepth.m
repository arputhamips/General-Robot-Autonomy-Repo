function H = HjacDepth(x, map, sensor_pos, K)
% HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       map          of environment, n x [x1 y1 x2 y2] walls
%       sensor_pos   sensor position in the body frame [1x2]
%       K            number of measurements (rays) the sensor gets between 27 to -27 
%
%   OUTPUTS
%       H            Kx3 jacobian matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   NIRMAL A J L A

% Small perturbation for finite differences
epsilon = max(1e-6, 1e-4 * norm(x));  % Adaptive epsilon for numerical stability

% Initialize H matrix (Kx3)
H = zeros(K, 3);

% Define sensor angles (rays evenly spaced from -27° to 27°)
angles = linspace(deg2rad(27), deg2rad(-27), K);

% Compute depth measurement at original state
depth_original = depthPredict(x, map, sensor_pos, angles);

% Compute Jacobian using finite differences
for i = 1:3
    x_perturbed = x;
    x_perturbed(i) = x_perturbed(i) + epsilon;  % Perturb one state variable
    
    % Compute perturbed depth
    depth_perturbed = depthPredict(x_perturbed, map, sensor_pos, angles);
    
    % Compute finite difference approximation
    H(:, i) = (depth_perturbed - depth_original) / epsilon;
end

% H = H / max(abs(H(:))); 

end