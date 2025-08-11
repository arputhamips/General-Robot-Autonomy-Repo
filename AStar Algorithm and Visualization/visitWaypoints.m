function[cmdV, cmdW] = visitWaypoints(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023
%   Zheng, Chengle


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0,0);
tic

gotopt = 1; %index for watpoints
wayPoints = [-3 0; 0 -3; 3 0; 0 3];
% wayPoints = [-1 0; 1 0];
N = size(wayPoints, 1);

epsilon = 0.2; %set to r
closeEnough = 0.1;

while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    cur_x = dataStore.truthPose(end, 2);
    cur_y = dataStore.truthPose(end, 3);
    theta = dataStore.truthPose(end, 4);

    target_x = wayPoints(gotopt, 1);
    target_y = wayPoints(gotopt, 2);
    
    difference = ((cur_x - target_x)^2 + (cur_y - target_y)^2)^(1/2);

    % CONTROL FUNCTION (send robot commands)
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        % if it reaches waypoint, go to find the next waypoint
        if difference <= closeEnough
            if(gotopt < N)
                gotopt = gotopt + 1;
            else
                SetFwdVelAngVelCreate(Robot, 0,0 );
            end
        else
            cmdVx = target_x - cur_x;
            cmdVy = target_y - cur_y;

            [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon);
            [cmdV, cmdW] = limitCmds(cmdV,cmdW,0.45,0.13);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
            pause(0.1);
        end
    end
    
    
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );
