function [dataStore] = rrtPlanner(Robot, maxTime)
% rrtPlanner
% 
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A



%----- Boilerplate (DO NOT CHANGE) -----
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, define the appropriate ports.
    CreatePort = Robot.CreatePort;
catch
    % If not a real robot, use the simulator object.
    CreatePort = Robot;
end

% Declare dataStore as a global variable so it can be accessed even if the program is stopped
global dataStore;
dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'trajectory', []);
%----- End Boilerplate -----


% ----------------------
% Control Starts Here
% ----------------------

% Load the map
load('cornerMap.mat'); % assumes "map" variable is loaded
load('waypoints.mat');
mapBoundary = [-5, -5, 5, 5];  % Based on simulator
robotRadius = 0.2;             % Robot physical radius

% Read current starting position from robot
pause(1); % Give sensors a second to initialize
[~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
if isempty(dataStore.truthPose)
    warning('No truthPose available. Cannot start.');
    SetFwdVelAngVelCreate(Robot, 0, 0);
    return;
end
start = dataStore.truthPose(end, 2:3); % [x, y]
goal = [2, 2.5];                       % Target goal

% Build RRT to generate waypoints
% waypoints = buildRRT(map, mapBoundary, start, goal, robotRadius);

% If no path found, stop and exit
% if isempty(waypoints)
%     disp('No path found by RRT. Exiting.');
%     SetFwdVelAngVelCreate(Robot, 0, 0);
%     return;
% end

% Parameters for motion control
epsilon = 0.2;         % Turn radius
closeEnough = 0.1;    % Distance threshold to consider waypoint reached
maxV = 0.5;            % Max wheel speed
wheel2Center = 0.13;   % Robot wheel offset
dt = 0.1;              % Time step
gotopt = 1;            % Start with first waypoint
timeElapsed = 0;       % Elapsed time

% Stop robot initially
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(1);

% Start control loop
tic;
while gotopt <= size(waypoints, 1) && timeElapsed < maxTime
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    if ~isempty(dataStore.truthPose)
        timeStamp = dataStore.truthPose(:,1);
        x_stored = dataStore.truthPose(:,2);
        y_stored = dataStore.truthPose(:,3);
        theta_stored = dataStore.truthPose(:,4);

        currentTime = toc;

        if size(dataStore.truthPose,1) > 1
            x = interp1(timeStamp, x_stored, currentTime, 'linear', 'extrap');
            y = interp1(timeStamp, y_stored, currentTime, 'linear', 'extrap');
            theta = interp1(timeStamp, theta_stored, currentTime, 'linear', 'extrap');
        else
            x = x_stored(end);
            y = y_stored(end);
            theta = theta_stored(end);
        end
    else
        warning('No valid pose data. Exiting control.');
        break;
    end

    % Current waypoint
    targetX = waypoints(gotopt,1);
    targetY = waypoints(gotopt,2);

    % Error in global frame
    errorX = targetX - x;
    errorY = targetY - y;
    distanceToTarget = norm([errorX, errorY]);

    % Desired velocities
    cmdVx = errorX;
    cmdVy = errorY;

    % Feedback linearization
    [cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, theta, epsilon);

    % Limit speeds
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);

    % Safety stop if localization is lost
    % if noRobotCount >= 3
    %     SetFwdVelAngVelCreate(Robot, 0, 0);
    % else
    %     SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    % end

    % Save trajectory
    dataStore.trajectory = [dataStore.trajectory; toc, x, y, theta, cmdV, cmdW];

    % Check if reached current waypoint
    if distanceToTarget < closeEnough
        gotopt = gotopt + 1; % Switch to next waypoint
        disp('next waypoint')
    end

    % Update time
    timeElapsed = timeElapsed + dt;
    pause(dt);
end

% Stop robot cleanly
SetFwdVelAngVelCreate(Robot, 0, 0);
disp('All waypoints completed. RRT plan executed.');

end