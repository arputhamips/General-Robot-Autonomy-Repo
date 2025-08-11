function [dataStore] = hexagonDrive(Robot, maxTime)
% HEXAGONDRIVE: Makes the robot move in a hexagonal pattern.
% The robot:
%   - Moves forward 1m
%   - Turns 60° counterclockwise
%   - Repeats 6 times to complete a hexagon
%
% Cornell University - MAE 5180
% ANTONYSELVARAJ JHON LEONARD AR , NIRMAL

% from turnInPlace.m
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % Check if running on real robot or simulator
    CreatePort = Robot.CreatePort;
catch
    % If not real robot, use the simulator object
    CreatePort = Robot;
end

% Declare dataStore as global
global dataStore;

% Initialize dataStore structure
dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', []);

% boilerplate ends



% Robot Parameters
speed = 0.5;        % Linear speed (m/s)
turnSpeed = 0.4;    % Angular speed (rad/s)
sideLength = 1.0;   % Length of each hexagon side (m)
turnAngleDeg = 60;  % Turn angle (degrees, counterclockwise)

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(0.5);

% Execute hexagonal movement (6 sides)
for i = 1:6
    [~, dataStore] = readStoreSensorData(Robot,0,dataStore);

    % Move forward 1m
    travelDist(Robot, speed, sideLength);
    
    % Turn 60° counterclockwise
    turnAngle(Robot, turnSpeed, turnAngleDeg);

    
    pause(0.5); % Small pause before next move
end

% Stop robot before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end