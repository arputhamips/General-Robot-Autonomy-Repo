function [dataStore] = backupBump(Robot, maxTime)
% Cornell University - MAE 5180
% ANTONYSELVARAJ JHON LEONARD AR , NIRMAL


%from turnInPlace.m
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

%boilerplate ends


maxV = 0.5;         % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to robot center (m)
fwdVel = 0.2;       % Forward velocity (m/s)
angVel = 0;         % Initial angular velocity (rad/s)
turnAngle = -pi/6;  % Clockwise turn of 30° (-π/6 rad)
backupDist = -0.25; % Reverse 0.25m
backupVel = -0.2;  % Speed while reversing (m/s)
turnSpeed = -0.3;   % Speed for turning (rad/s)

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);

tic;
while toc < maxTime
    % Read sensor data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Check if bump data is available and detect bump
    if isfield(dataStore, 'bump') && ~isempty(dataStore.bump)
        bumpDetected = any(dataStore.bump(end, 2:end)); % Check if any bump sensor is triggered
    else
        bumpDetected = false;
    end
    
    if bumpDetected
        % Stop robot immediately
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);
        
        % Move backward for 0.35m
        backupTime = abs(backupDist / backupVel);
        SetFwdVelAngVelCreate(Robot, backupVel, 0);
        pause(backupTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after backing up

        stopRobot(Robot);
        
        % Rotate in place 45° clockwise
        
        turnTime = turnAngle / turnSpeed;
        SetFwdVelAngVelCreate(Robot, 0, turnSpeed);
        pause(turnTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after turning
        
        % Small pause before moving forward again
        pause(0.5);
    end
    
    % Continue driving forward
    
    SetFwdVelAngVelCreate(Robot, 0.3, 0);
    
    pause(0.1);
end

% Stop robot before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end

function stopRobot(Robot)
    SetFwdVelAngVelCreate(Robot, 0, 0);
    disp('pause');
    pause(3);
    disp('unpause');
end
