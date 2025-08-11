function[dataStore] = visitWaypoints(Robot, maxTime)
% VISITWAYPOINTS: Moves the robot to a series of waypoints using feedback linearization.
%
%   dataStore = VISITWAYPOINTS(Robot, maxTime, waypoints, epsilon, closeEnough) moves the robot to
%   a series of waypoints, calculating the required forward velocity and angular velocity using
%   the feedback linearization technique.
%
%   INPUTS:
%       Robot        - Port configurations and robot name (get from running CreatePiInit)
%       maxTime      - Maximum time to run program (in seconds)
%       waypoints    - nx2 matrix of (x,y) coordinates of waypoints
%       epsilon      - Turning radius (m), typically the robot radius
%       closeEnough  - Distance (m) within which the robot is considered to have reached the waypoint
%
%   OUTPUTS:
%       dataStore    - struct containing logged data
%

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

% If using a real robot, get CreatePort, else use simulator object
try 
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

% Declare dataStore as a global variable to access it from workspace
global dataStore;

% Initialize dataStore for logging
dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);

% Initialize variables for waypoint tracking
%waypoints =[-3 0;0 -3;3 0;0 3]
% waypoints = [-1 0; 1 0 ];

start = [0.410,	0.009]; % need update to PF result
path = runRoadmap2(start);

gotopt = 2; %index for watpoints starts from the second node. 
waypoints = path;
% wayPoints = [-1 0; 1 0];
N = size(waypoints, 1);

epsilon = 0.2;  % Robot radius (m)
closeEnough = 0.1;  % Close enough threshold (m)

% Initialize variables for waypoint tracking
gotopt = 1;  % Index of the current waypoint
targetWaypoint = waypoints(gotopt, :);  % Set initial target waypoint

% Variable used to keep track of whether the overhead localization has lost the robot
noRobotCount = 0;

% Stop the robot at the start
SetFwdVelAngVelCreate(Robot, 0, 0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % Get robot's current position (truthPose: [x, y, theta])
    currentPose = dataStore.truthPose(end, :); 
    robotX = currentPose(2); 
    robotY = currentPose(3); 
    robotTheta = currentPose(4);

    % Check if the robot has reached the target waypoint (within closeEnough distance)
    distanceToWaypoint = sqrt((robotX - targetWaypoint(1))^2 + (robotY - targetWaypoint(2))^2);
     if distanceToWaypoint <= closeEnough
        % If close enough, move to the next waypoint and update the 'gotopt' index
        gotopt = gotopt + 1;
        if gotopt <= size(waypoints, 1)
            targetWaypoint = waypoints(gotopt, :);  % Update target waypoint
        else
            % If all waypoints are visited, break the loop
            break;
        end
    end
    
    % Calculate desired velocities using feedback linearization
    [cmdV, cmdW] = feedbackLin(targetWaypoint(1) - robotX, targetWaypoint(2) - robotY, robotTheta, epsilon);
    
    % Limit velocities to avoid motor saturation
    wheel2Center = 0.13;  % Distance from wheels to center (m)
    maxV = 0.45;  % Maximum velocity (m/s), less than 0.5 as specified
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
    
    % If overhead localization loses the robot, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
    
    % Pause to allow robot to respond
    pause(0.1);
end

% Stop robot before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

% Plot the robot's trajectory after reaching the last waypoint
figure;
hold on;
plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'b', 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory');
grid on;
hold off;

end