function [dataStore] = potentialPlanner(Robot, maxTime)
% Cornell University - MAE 5180
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A



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


% maxV = 0.5;         % Max wheel speed (m/s)
% wheel2Center = 0.13; % Distance from wheel to robot center (m)
% fwdVel = 0.2;       % Forward velocity (m/s)
% angVel = 0;         % Initial angular velocity (rad/s)
% turnAngle = -pi/6;  % Clockwise turn of 30° (-π/6 rad)
% backupDist = -0.25; % Reverse 0.25m
% backupVel = -0.15;  % Speed while reversing (m/s)
% turnSpeed = -0.4;   % Speed for turning (rad/s)

hw6aMap=[1.991	-1.990	1.991	-3.050
1.991	-3.050	3.0090	-3.050
3.009	-3.050	3.009	-1.990
3.009	-1.990	1.991	-1.990
-0.957	3.990	-0.957	1.990
-0.957	1.990	1.009	1.990
1.009	1.990	1.009	3.990
1.009	3.990	-0.957	3.990
-1.957	-0.990 -1.957	-3.010
-1.957	-3.010	-3.991	-3.010
-3.991	-3.010	-3.991	-0.990
-3.991	-0.990	-1.957	-0.990];

% generateSphereWorld
sphereWorld = generateSphereWorld(hw6aMap);


goal = [0, 0];  % Fixed at origin

% Potential field tuning
c_att = 1.0;     % Attractive potential coefficient
c_rep = 500.0;   % Repulsive potential coefficient
Q = 1.5;         % Obstacle influence range (meters)

% Feedback linearization parameter
epsilon = 0.2;   % Small positive number for feedback linearization

% Robot physical limits
maxV = 0.5;         % Maximum wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to center (m)

% Other initializations
V = 0;             % Forward velocity initialized
omega = 0;         % Angular velocity initialized

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);

pause(1.0);


tic;
while toc < maxTime
    % Read sensor data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Extract current robot position (x, y)
    pose = dataStore.truthPose(end, 2:4);
    
    % Compute potential gradient at current position
    [~, gradU] = potentialPoint(sphereWorld, goal, c_att, c_rep, Q, pose(1:2));
    
    % Desired global velocity is along negative gradient
    desVel = -gradU(:);
    
    % Compute forward and angular velocity using feedback linearization
    [V, omega] = feedbackLin(desVel(1), desVel(2), pose(3), epsilon);
    
    % Limit the commands based on robot's max velocity and geometry
    [cmdV, cmdW] = limitCmds(V, omega, maxV, wheel2Center);
    
    % Send the commands to robot
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

    % CHECK IF GOAL IS REACHED AND EXIT
    if norm(pose(1:2) - goal) < 0.08
        disp('Goal reached!');
        break;
    end
    
    
end


% Stop robot before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end