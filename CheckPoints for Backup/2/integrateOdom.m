function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   ANTONYSELVARAJ JHON LEONARD AR, NIRMAL


% Number of odometry updates
N = length(d);

% Initialize pose storage (3×N matrix)
finalPose = zeros(3, N);

% Set the initial pose
x_t = initPose(1);
y_t = initPose(2);
theta_t = initPose(3);

% Loop through each odometry reading
for i = 1:N
    % Extract distance and angle
    d_i = d(i);
    phi_i = phi(i);
    
    
    % deg_i = rad2deg(phi_i);
    % 
    % if deg_i < 0
    %     phi_i = deg2rad(360 + deg_i);
    % end
    

    % Handle straight-line motion case (phi = 0)
    if abs(phi_i) < 1e-6  % Threshold to avoid division by zero
        x_t = x_t + d_i * cos(theta_t);
        y_t = y_t + d_i * sin(theta_t);
    else
        % Compute radius of curvature
        R = d_i / phi_i;
        
        % Apply odometry update equations
        x_t = x_t + R * (sin(theta_t + phi_i) - sin(theta_t));
        y_t = y_t - R * (cos(theta_t + phi_i) - cos(theta_t));

        % x_t = x_t + R * (sin(theta_t + phi_i) - sin(theta_t));
        % y_t = y_t - R * (cos(theta_t + phi_i) - cos(theta_t));
    end
    
    % Update orientation
    theta_t = theta_t + phi_i;
    
    % Store updated pose
    finalPose(:, i) = [x_t; y_t; theta_t];
end
end