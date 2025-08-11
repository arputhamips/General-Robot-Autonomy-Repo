function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   NIRMAL, A J L A

% Extract state and control inputs
theta = x(3);  % Current heading
d = u(1);      % Distance traveled
phi = u(2);    % Change in orientation

% Compute Jacobian matrix G
G = eye(3); % Initialize as identity matrix

% Update non-trivial elements
G(1,3) = -d * sin(theta + phi/2); % ∂x'/∂theta
G(2,3) = d * cos(theta + phi/2);  % ∂y'/∂theta

    
    
end