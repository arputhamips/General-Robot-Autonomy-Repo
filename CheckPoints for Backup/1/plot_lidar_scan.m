% Lidar Scan Transformation to Global Frame
% Author: Antonyselvaraj John Leonard AR, Nirmal
% Course: Autonomous Mobile Robots - Cornell University
% Description: This script loads a lidar scan, converts it to Cartesian
% coordinates using lidar_range2xy.m, and transforms it to global coordinates
% using robot2global.m, then plots the results.


% Load Lidar Scan Data
data = load('lidarScan.mat');  
lidarR = data.lidarScan;  % Extract scan data

% Define Lidar Parameters
robotRad = 0.2;  % Robot radius (meters)
angRange = deg2rad([-120, 120]);  % Convert angle range to radians

% Convert Lidar Scan to Cartesian
lidarXY_local = lidar_range2xy(lidarR, robotRad, angRange);  % Output is 2×N

% Robot Poses
poses = { [3, -4, 2*pi/3], [0, 3, pi/2] };
poseLabels = { 'Pose A (3, -4, 2π/3)', 'Pose B (0, 3, π/2)' }; %for final figure

% Number of lidar points
N = size(lidarXY_local, 2);

% Initialize varibles
globalXY_A = zeros(N, 2);  % Store results in an N×2 matrix
globalXY_B = zeros(N, 2);

% go throughh array and convert each point
for i = 1:N
    globalXY_A(i, :) = robot2global(poses{1}, lidarXY_local(:, i)'); % Convert single point
    globalXY_B(i, :) = robot2global(poses{2}, lidarXY_local(:, i)'); % Convert single point
end

% Results
figure; hold on; grid on;
scatter(globalXY_A(:,1), globalXY_A(:,2), 2, 'r', 'DisplayName', poseLabels{1});
scatter(globalXY_B(:,1), globalXY_B(:,2), 2, 'b', 'DisplayName', poseLabels{2});
plot(poses{1}(1), poses{1}(2), 'ro', 'MarkerSize', 8, 'DisplayName', 'Robot Position A');
plot(poses{2}(1), poses{2}(2), 'bo', 'MarkerSize', 8, 'DisplayName', 'Robot Position B');

% Label
xlabel('X (meters)');
ylabel('Y (meters)');
title('Lidar Scan in Global Reference Frame');
legend;
axis equal;
hold off;
