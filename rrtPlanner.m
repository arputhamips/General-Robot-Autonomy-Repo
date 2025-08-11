function [dataStore] = runRRTPlanner(Robot, maxTime)
% runRRTPlanner
% NIRMAL A J L A - AMR Final Integration

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;
dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'trajectory', [], ...
                   'RRT', []);

% Load map and goal
load('practicemap2025update.mat');  % includes map, optWalls, beaconLoc, waypoints, etc.
mapBoundary = [-5 -5 5 5];
robotRadius = 0.2;

% Set initial and goal pose
start = [2.440, -1.690];
goal = [0.41, 0.009];

% Build RRT
routepoints = buildRRT(map, mapBoundary, start, goal, robotRadius);
if isempty(routepoints)
    disp('No path found by RRT.');
    SetFwdVelAngVelCreate(Robot, 0, 0);
    return;
end

% Run particle filter to initialize and update pose estimate
pause(1); % let sensors initialize
[~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
runParticleFilter(map, beaconLoc, optWalls, waypoints);  % sets dataStore.truthPose

% RRT Planning Parameters
epsilon = 0.2;
closeEnough = 0.1;
maxV = 0.5;
wheel2Center = 0.13;
dt = 0.1;
gotopt = 1;

SetFwdVelAngVelCreate(Robot, 0, 0);
pause(1);

% Control Loop
tic;
while gotopt <= size(routepoints, 1) && toc < maxTime
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    runParticleFilter(map, beaconLoc, optWalls, waypoints, toc);  % update pose

    currPose = dataStore.truthPose(end, 2:4);  % [x, y, theta]
    x = currPose(1);
    y = currPose(2);
    theta = currPose(3);

    % Current target
    target = routepoints(gotopt,:);
    error = target - [x y];
    distToTarget = norm(error);

    % Velocity planning
    [cmdV, cmdW] = feedbackLin(error(1), error(2), theta, epsilon);
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);

    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    dataStore.trajectory = [dataStore.trajectory; toc, x, y, theta, cmdV, cmdW];

    if distToTarget < closeEnough
        gotopt = gotopt + 1;
        disp('Next routepoint reached.');
    end

    % timeElapsed = timeElapsed + dt;
    % pause(dt);  % optional throttling
end

SetFwdVelAngVelCreate(Robot, 0, 0);
disp('Route complete.');
end