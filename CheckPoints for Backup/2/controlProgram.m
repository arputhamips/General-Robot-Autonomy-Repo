function[dataStore] = controlProgram(Robot,maxTime)
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
% 	Modified: Liran 2023 --> Nirmal 2025 (updated to use limitCmds.m)


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 25;
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
% global rdistance;
% global rangle;
% global truthpose;


% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [],...
                   'trajectory', [],...
                   'bump', [],...
                   'rsdepth',[]);
% rdistance = [];
% rangle  = [];
% truthpose = [];

% Stop the robot initially
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(1);

% Define motion parameters
fwdVel = 0.6;      % Forward velocity (m/s)
angVel = 0.3;      % Angular velocity (rad/s)
maxV = 0.5;        % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to center

% Control loop
tic;
timeElapsed = 0;
noRobotCount = 0;  % Count of missing overhead localization data

while timeElapsed < maxTime
    % Read and store sensor data
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % Get robot's estimated pose
    if ~isempty(dataStore.truthPose)
        x = dataStore.truthPose(end, 2);
        y = dataStore.truthPose(end, 3);
        theta = dataStore.truthPose(end, 4);
    else
        warning('No valid pose data. Exiting.');
        break;
    end
    
    if ~isempty(dataStore.bump) && any(dataStore.bump(end, 2:end)) % Any bump sensor triggered
        disp('Bump detected! Executing avoidance maneuver.');
        
        % Stop robot
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);
        
        % Backup 0.2m
        travelDist(Robot, 0.2, -0.2); 
        pause(1);
        
        % Rotate 180°
        turnAngle(Robot, 0.4, 90);
        pause(1);
        
        % Reset time tracking
        timeElapsed = toc;
        continue; % Skip this iteration and continue motion
    end

    % Apply velocity limits
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
    
    % Stop if localization is lost
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
    
    % Store trajectory data 
    % dataStore.trajectory = [dataStore.trajectory; toc, x, y, theta, cmdV, cmdW];
    
    
    % Pause for next control step
    pause(0.1);
    % rdistance = [rdistance , DistanceSensorRoomba(Robot)];
    % rangle = [rangle , AngleSensorRoomba(Robot)];
    % truthpose = [truthpose ; x, y, theta];
    timeElapsed = toc;
end

% Stop robot after reaching maxTime
SetFwdVelAngVelCreate(Robot, 0, 0);
disp('Control program completed.');

end