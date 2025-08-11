function[lidarXY] = lidar_range2xy(lidarR, robotRad, angRange)
% LIDAR_RANGE2XY: convert lidar measurements (range & bearing) into x/y
% coordinates (in robot local frame)
% 
%   LIDARXY = LIDAR_RANGE2XY(LIDARR,ROBOTRAD,ANGRANGE) returns the
%   x/y coordinates (in robot local frame) of lidar measurements.
% 
%   INPUTS
%       lidarR      1-by-N vector of scan ranges (meters)
%       robotRad    robot radius (meters)
%       angRange    total angular field of view of lidar (radians) (1-by-2)
% 
%   OUTPUTS
%       lidarXY     2-by-N matrix of x/y scan locations
% 
%   NOTE: Assume lidar is located at front of robot and pointing forward 
%         (along robot's x-axis).
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   ANTONYSELVARAJ JHON LEONARD AR , NIRMAL

% number of range measurements
N = length(lidarR);

% generate angle values for each scan
angles = linspace(angRange(1), angRange(2), N);


% convert polar coordinates to cartesian
x = lidarR .* cos(angles) + robotRad;  % Adjust for sensor offset
y = lidarR .* sin(angles);

% store results
lidarXY = [x; y];

end

