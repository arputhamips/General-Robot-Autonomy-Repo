function [dataStore] = AstarPlanner(Robot, maxTime)
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
                   'beacon', [],...
                   'visitedPt',[]);


% numParticles = 1000;

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0,0);
tic
% [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);


% load map infromation (it is repeated with runRoadmap2...)
load("practicemap2025update.mat");
walls = [map; optWalls];            % combine real map and optWalls
wayPt = [waypoints; ECwaypoints];
visited = zeros(size(wayPt,1),1) ; % record which waypoints has been visited, initially none of them are visited
[noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
% dataStore = particleFilterUpdate(Robot, dataStore, map, beaconLoc,optWalls,waypoints);

% localize the initial pose
% dataStore = particleFilterUpdate(Robot, dataStore, map, beaconLoc, optWalls, waypoints);
p = [dataStore.truthPose(2), dataStore.truthPose(3)]; % need replacement

% Find out the closes real point
dists = vecnorm(wayPt - p, 2, 2);
[~, idx] = min(dists);
start = wayPt(idx,:);

% We will start from this nearest waypoint
path = runRoadmap2(start, walls, wayPt); % path is a list of vertices coordinate

gotopt = 1; %index always starts from 1 
N = size(path, 1);

epsilon = 0.2; %set to r
closeEnough = 0.15;

while toc < maxTime
    
    % READ & STORE SENSOR DATA
%     [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
%     dataStore = particleFilterUpdate(Robot, dataStore, map, beaconLoc,optWalls,waypoints);
    
    % Interperlation
    
    timeStamp = dataStore.truthPose(:,1);
    x_stored = dataStore.truthPose(:,2);
    y_stored = dataStore.truthPose(:,3);
    theta_stored = dataStore.truthPose(:,4);

    currentTime = toc;

    if size(dataStore.truthPose,1) > 1
        %––– unwrap the recorded angles to avoid discontinuities –––
        theta_unwrapped = unwrap(theta_stored);  % %%% NEW LINE

        % (optional) clamp how far ahead we extrapolate
        delay      = currentTime - timeStamp(end);
        maxExtrap  = 0.5;  % seconds
%         queryTime  = timeStamp(end) + min(delay, maxExtrap);
        queryTime  = timeStamp(end) + delay;

        % interpolate / extrapolate
        x_hat     = interp1(timeStamp, x_stored,          queryTime, 'linear', 'extrap');
        y_hat     = interp1(timeStamp, y_stored,          queryTime, 'linear', 'extrap');
        theta_hat = interp1(timeStamp, theta_unwrapped,   queryTime, 'linear', 'extrap');  % use unwrapped

        % wrap back into [‑π,π]
        x     = x_hat;
        y     = y_hat;
        theta = wrapToPi(theta_hat);
    else
        x = x_stored(end);
        y = y_stored(end);
        theta = theta_stored(end);
    end

    finalPose = [x,y,theta];
    %}
    
    %{
    % odometry + truthPose
    old_x = dataStore.truthPose(end, 2);
    old_y = dataStore.truthPose(end, 3);
    old_theta = dataStore.truthPose(end, 4);

    [finalPose] = integrateOdom([old_x;old_y; old_theta],dataStore.odometry(end,2),dataStore.odometry(end,3));
    %}

    
    target_x = path(gotopt, 1);
    target_y = path(gotopt, 2);
    
    difference = ((finalPose(1) - target_x)^2 + (finalPose(2) - target_y)^2)^(1/2);

    % CONTROL FUNCTION (send robot commands)
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        % if it reaches waypoint, go to find the next waypoint
        if difference <= closeEnough
            % check if the target is wayPt
            isElement = ismember(wayPt, [target_x,target_y],'rows');
            exists = any(isElement);

            % check is target is unvisited wayPt
            if exists && visited(isElement) == 0
                % beep a sound
                % ////// beep //////
%                 BeepCreate(Robot);
                
                % visual provement
                fprintf("reach waypoint: (%.2f, %.2f)\n", target_x, target_y);
                BeepCreate(Robot);
                % ///// mapping code ///////
                visualizeMappingResults4(dataStore);

                visited(isElement) = 1;
                dataStore.visitedPt(end+1,:) = [target_x,target_y];
                
                %%================ broken turn around =================
                % turn around for 360 degrees
%                 SetFwdVelAngVelCreate(Robot, 0, 0.13 );
%                 pause(8);
%                 SetFwdVelAngVelCreate(Robot, 0, 0 );
%                 pause(1);
%                 [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
                % Senario: after turn some angle, it starts to go to the
                % designed path but it never followed the path
                %%======================================================
               
            end


            if(gotopt < N)
                gotopt = gotopt + 1;
%                 SetFwdVelAngVelCreate(Robot, 0,0 );

            else % finish the current rout, may have next rout
                SetFwdVelAngVelCreate(Robot, 0,0 );

                % construct mapping here 
                % ///// mapping code ///////
                visualizeMappingResults3(dataStore);
                load("updatepracticemap2025update.mat");
                    
                % check if all waypoints has been visited:
                if ~all(visited==1)
                    % extract the rest of waypoints and new map
                    wayPt = wayPt(visited == 0,:);
                    walls = [updatedMap; updatedOptWalls];   % only use updated map

                    visited = zeros(size(wayPt,1),1);
                    start = [target_x, target_y];
                    wayPt = [start;wayPt]; % put start in wayPt so it can generate a correct path

                    visited = zeros(size(wayPt,1),1) ;
                    visited(1) = 1;

                    path = runRoadmap2(start, walls, wayPt); % path is a list of vertices coordinate

                    gotopt = 1; %index always starts from 1 
                    N = size(path, 1);
                else
                    break; % finish
                end
                
            end
        else
            cmdVx = target_x - finalPose(1);
            cmdVy = target_y - finalPose(2);

            [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,finalPose(3),epsilon);
            [cmdV, cmdW] = limitCmds(cmdV,cmdW,0.1,0.08);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
            pause(0.1);
        end
    end
    
    
end % end while

% construct mapping here 
% ///// mapping code ///////
visualizeMappingResults4(dataStore);


% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );

end