function [dataStore] = potentialPlannar(Robot, maxTime)
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.


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

[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);

% Information for potential field
map = dlmread("hw6Sphere.txt");
goal = [0,0];
c_att = 15;
c_rep = 3;
Q = 10;


while toc < maxTime
    disp("RUN potentialPlannar");
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        x = dataStore.truthPose(end,2);
        y = dataStore.truthPose(end,3);
        theta = dataStore.truthPose(end,4);

        [U, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, [x,y]);
        cmd_x = -U_grad(1);
        cmd_y = -U_grad(2);
%         cmd_x = -sign(U_grad(1))*log(abs(U_grad(1)));  % negative gradient and log scale
%         cmd_y = -sign(U_grad(2))*log(abs(U_grad(2)));
        disp(cmd_x);
        disp(cmd_y);
        [cmdV, cmdW] = feedbackLin(cmd_x,cmd_y,theta,0.2);

        [cmdV, cmdW] = limitCmds(cmdV,cmdW,0.45,0.13);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
        pause(0.01);

       
        
        disp("cmdV");
        disp(cmdV);
        disp("cmdW");
        disp(cmdW);

%         SetFwdVelAngVelCreate(Robot, cmdV, 0 );
% 
%         pause(0.01);
        
    end

    % after one time movement: READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    
%     pause(0.01);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );





end