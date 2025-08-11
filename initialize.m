function [dataStore, initPose] = initialize(Robot, maxTime)
%INITIALIZE Hybrid initialization using beacon for waypoint and particles for orientation
%
% [dataStore, initPose] = initialize(Robot, maxTime)
%
% This function uses a two-stage approach:
% 1. Use beacon detection to identify the correct waypoint
% 2. Use particles at that waypoint with different orientations to find best orientation
%
% Inputs:
%   Robot - Simulator/Create port
%   maxTime - Maximum time for initialization (optional)
%
% Outputs:
%   dataStore - Data struct with sensor readings
%   initPose - Initial pose [x; y; theta]

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

% Load map and configuration
load('practicemap2025update.mat');

% Parameters
rotationSpeed = 0.5;         % Rotation speed (rad/s)
maxRotationTime = 15;        % Maximum time to rotate (s)
depthThreshold = 0.15;       % Threshold for depth validation
sensorOrigin = [0, 0.08];    % Sensor origin in robot frame
angles = linspace(-27*pi/180, 27*pi/180, 9)'; % Sensor angles
numOrientationParticles = 360; % One particle per degree

global dataStore;

% Initialize data store
dataStore = struct('odometry', [], 'rsdepth', [], 'bump', [], 'beacon', [], 'truthPose', [], 'particles', []);

% Step 1: Stop the robot
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.5);

fprintf('Starting beacon detection...\n');

% Step 2: Rotate until a beacon is detected
beaconDetected = false;
rotationStartTime = tic;

% Start rotation
SetFwdVelAngVelCreate(Robot, 0, rotationSpeed);

while ~beaconDetected && toc(rotationStartTime) < maxRotationTime
    % Read beacon data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Check if we detected a beacon
    if ~isempty(dataStore.beacon)
        % Store beacon data
        beaconId = dataStore.beacon(end, 2);
        beaconX = dataStore.beacon(end, 3);  % x = forward distance from camera
        beaconY = dataStore.beacon(end, 4);  % y = lateral offset from camera
        
        fprintf('Beacon detected! ID: %d, Forward distance: %.2f m, Lateral offset: %.2f m\n', ...
            beaconId, beaconX, beaconY);
        
        beaconDetected = true;
    end
    
    % Small pause
    pause(0.1);
end

% Stop rotation
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.5);

% Check if we detected a beacon
if ~beaconDetected
    error('Failed to detect any beacons during rotation');
end

% Step 3: Calculate distance to the detected beacon
beaconId = dataStore.beacon(end, 2);
beaconX = dataStore.beacon(end, 3); % Forward distance
beaconY = dataStore.beacon(end, 4); % Lateral offset
beaconDist = hypot(beaconX, beaconY); % Euclidean distance to beacon

% Find this beacon in the beacon locations
beaconIdx = find(beaconLoc(:,1) == beaconId, 1);
if isempty(beaconIdx)
    error('Detected beacon ID %d not found in beaconLoc', beaconId);
end

% Get beacon's global position
beaconGlobalX = beaconLoc(beaconIdx, 2);
beaconGlobalY = beaconLoc(beaconIdx, 3);

fprintf('Beacon global position: [%.2f, %.2f]\n', beaconGlobalX, beaconGlobalY);
fprintf('Distance to beacon: %.2f m\n', beaconDist);

% Step 4: Find which waypoint is at this distance from the beacon
bestWaypoint = [];
bestDistDiff = inf;
bestWaypointIdx = 0;

for i = 1:size(waypoints, 1)
    waypointX = waypoints(i, 1);
    waypointY = waypoints(i, 2);
    
    % Distance from waypoint to beacon
    waypointToBeaconDist = norm([waypointX, waypointY] - [beaconGlobalX, beaconGlobalY]);
    
    % Difference between measured and expected distance
    distDiff = abs(waypointToBeaconDist - beaconDist);
    
    fprintf('Waypoint %d: [%.2f, %.2f], Distance to beacon: %.2f m, Diff: %.2f m\n', ...
        i, waypointX, waypointY, waypointToBeaconDist, distDiff);
    
    if distDiff < bestDistDiff
        bestDistDiff = distDiff;
        bestWaypoint = waypoints(i, :);
        bestWaypointIdx = i;
    end
end

% Check if we found a reasonable match
if bestDistDiff > 0.3 % Threshold for acceptable distance match (30 cm)
    warning('Best waypoint match has large distance difference (%.2f m)', bestDistDiff);
end

fprintf('Most likely waypoint: %d at [%.2f, %.2f]\n', bestWaypointIdx, bestWaypoint(1), bestWaypoint(2));

% Step 5: Calculate initial robot position (at the identified waypoint)
robotX = bestWaypoint(1);
robotY = bestWaypoint(2);

% Direction from robot to beacon in global frame
dirToBeaconGlobal = atan2(beaconGlobalY - robotY, beaconGlobalX - robotX);

% Direction from robot to beacon in robot frame
dirToBeaconRobot = atan2(beaconY, beaconX);

% Initial orientation estimate (from beacon)
initialTheta = wrapToPi(dirToBeaconGlobal - dirToBeaconRobot);

fprintf('Initial orientation estimate from beacon: %.2f degrees\n', initialTheta * 180/pi);

% Step 6: Initialize particles at the waypoint with all possible orientations
fprintf('Initializing %d orientation particles at waypoint...\n', numOrientationParticles);

% Use the proper function to initialize particles at the identified waypoint
dataStore = initializeWaypointParticles(dataStore, map, bestWaypoint, numOrientationParticles, optWalls);

% Step 7: Perform a full 360-degree rotation to collect sensor data for all orientations
fprintf('Performing 360-degree rotation to collect sensor data...\n');

% Calculate rotation time for a full circle
fullRotationTime = 2*pi / rotationSpeed;
numStops = 12; % Number of stops during rotation
stopAngle = 2*pi / numStops;
stopTime = fullRotationTime / numStops;

% Rotate in small increments, stopping to collect data
for i = 1:numStops
    % Rotate to next position
    SetFwdVelAngVelCreate(Robot, 0, rotationSpeed);
    pause(stopTime);
    
    % Stop to collect data
    SetFwdVelAngVelCreate(Robot, 0, 0);
    pause(0.5);
    
    % Collect sensor data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Update particle weights based on latest measurements
    dataStore = updateParticleWeights(dataStore, map, sensorOrigin, angles, optWalls, beaconLoc);
    
    fprintf('Rotation stop %d of %d completed\n', i, numStops);
end

% Step 8: Find the best particle (orientation)
fprintf('Finding best orientation from particles...\n');
[bestPose, bestScore] = findBestParticle(dataStore, robotX, robotY);

fprintf('Best particle orientation: %.2f degrees (score: %.4f)\n', bestPose(3)*180/pi, bestScore);

% Step 9: Rotate to the best orientation
fprintf('Rotating to best orientation...\n');
currentTheta = wrapToPi(initialTheta + 2*pi); % Current orientation after rotation
angleToRotate = wrapToPi(bestPose(3) - currentTheta);
rotationTime = abs(angleToRotate) / rotationSpeed;

SetFwdVelAngVelCreate(Robot, 0, sign(angleToRotate) * rotationSpeed);
pause(rotationTime);
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.5);

% Step 10: Fine-tune orientation with small adjustments
fprintf('Fine-tuning orientation...\n');

% Best pose so far
bestPose = [robotX; robotY; bestPose(3)];

% Predict depth at best pose
bestPredDepth = depthPredict(bestPose, map, sensorOrigin, angles);

% Get latest depth reading
[~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
if ~isempty(dataStore.rsdepth)
    realDepth = dataStore.rsdepth(end, 3:11)';
    bestDepthError = norm(bestPredDepth - realDepth);
else
    bestDepthError = inf;
end

% Small adjustments around best orientation
adjustAngles = [-10, -5, 5, 10] * pi/180;
bestAdjError = bestDepthError;
bestOrientation = bestPose(3);

for i = 1:length(adjustAngles)
    % Test orientation with small adjustment
    testTheta = wrapToPi(bestOrientation + adjustAngles(i));
    
    % Rotate to test orientation
    angleToRotate = adjustAngles(i);
    rotationTime = abs(angleToRotate) / (rotationSpeed/2); % Slower for precision
    
    SetFwdVelAngVelCreate(Robot, 0, sign(angleToRotate) * (rotationSpeed/2));
    pause(rotationTime);
    SetFwdVelAngVelCreate(Robot, 0, 0);
    pause(0.5);
    
    % Get depth readings
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Calculate expected depth readings
    testPose = [robotX; robotY; testTheta];
    predDepth = depthPredict(testPose, map, sensorOrigin, angles);
    
    % Get latest depth
    if ~isempty(dataStore.rsdepth)
        realDepth = dataStore.rsdepth(end, 3:11)';
        
        % Calculate error
        depthError = norm(predDepth - realDepth);
        
        fprintf('  Adjustment %.1f°: Depth error = %.4f\n', adjustAngles(i)*180/pi, depthError);
        
        % Update best if improved
        if depthError < bestAdjError
            bestAdjError = depthError;
            bestOrientation = testTheta;
            bestPredDepth = predDepth;
            fprintf('  New best orientation: %.1f°\n', bestOrientation*180/pi);
        end
    end
end

% Final position
initPose = [robotX; robotY; bestOrientation];

% Final output
fprintf('\n=============== INITIALIZATION COMPLETE ===============\n');
fprintf('Initial position: [%.4f, %.4f, %.1f°]\n', ...
    initPose(1), initPose(2), initPose(3)*180/pi);
fprintf('Waypoint: #%d [%.4f, %.4f]\n', bestWaypointIdx, bestWaypoint(1), bestWaypoint(2));
fprintf('Depth error: %.4f\n', bestAdjError);
fprintf('===================================================\n\n');

end