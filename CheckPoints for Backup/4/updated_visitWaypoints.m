function[dataStore] = updated_visitWaypoints(Robot, maxTime)
% Cornell University - MAE 5180
% ANTONYSELVARAJ JHON LEONARD AR , NIRMAL

% from turnInPlace.m
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 100;
end

try 
    
    CreatePort = Robot.CreatePort;
catch
    
    CreatePort = Robot;
end


global dataStore;


dataStore = struct('truthPose', [], 'odometry', [], 'trajectory', []);

% Variable to track if the overhead localization lost the robot
noRobotCount = 0;

% Define waypoints
% waypoints = [-3 0; 0 -3; 3 0; 0 3];  % First set of waypoints
waypoints = [-0.314 -1.67; 0.919 1.676; 2.133 -0.768]; 


% Parameters
epsilon = 0.2;      % Robot radius (m)
closeEnough = 0.1;  % Distance threshold (m)
maxV = 0.5;         % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to center

% Initialize tracking variables
gotopt = 1; % Start with the first waypoint
dt = 0.1;   % Control loop time step
timeElapsed = 0;

% Stop the robot initially
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(1); % Allow sensors to initialize

% Control loop
tic;
while gotopt <= size(waypoints, 1) && timeElapsed < maxTime
    % READ & STORE SENSOR DATA
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % Get robot's estimated pose from truth data
    if ~isempty(dataStore.truthPose)
        timeStamp = dataStore.truthPose(:, 1);
        x_stored = dataStore.truthPose(:, 2);
        y_stored = dataStore.truthPose(:, 3);
        theta_stored = dataStore.truthPose(:, 4);
        
        % Get Current Time
        currentTime = toc;

        if size(dataStore.truthPose, 1) > 1 
            % interpolate to manage lags in truthpose data
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
    
    % Extract current target waypoint
    targetX = waypoints(gotopt, 1);
    targetY = waypoints(gotopt, 2);
    
    % Compute error in the global frame
    errorX = targetX - x;
    errorY = targetY - y;
    distanceToTarget = norm([errorX, errorY]);
    
    % Compute desired velocity in the global frame
    cmdVx = errorX; 
    cmdVy = errorY;
    
    % Apply feedback linearization
    [cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, theta, epsilon);
    
    % Scale velocity commands to avoid exceeding motor limits
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
    
    % If localization is lost, stop the robot for safety
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
    
    % Store trajectory data
    dataStore.trajectory = [dataStore.trajectory; toc, x, y, theta, cmdV, cmdW];
    
    % Check if waypoint is reached
    if distanceToTarget < closeEnough
        gotopt = gotopt + 1; % Move to next waypoint
    end
    
    % Update time and pause for next control step
    timeElapsed = timeElapsed + dt;
    pause(dt);
end

% Stop the robot after reaching the last waypoint
SetFwdVelAngVelCreate(Robot, 0, 0);
disp('Waypoints completed.');

end
