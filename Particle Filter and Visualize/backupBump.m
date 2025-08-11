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
    CreatePort = Robot.CreatePort;
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

% boilerplate ends

% define map (2‐decimals)
map = [
   -2.00  2.00  -2.00 -2.00;
   -2.00 -2.00   2.00 -2.00;
    2.00 -2.00   2.00  2.00;
    2.00  2.00  -2.00  2.00;
    1.00 -1.00   1.00 -2.00;
    1.00 -2.00   2.00 -2.00;
    2.00 -2.00   2.00 -1.00;
    2.00 -1.00   1.00 -1.00;
   -2.00  2.00  -2.00  1.00;
   -2.00  1.00  -1.00  1.00;
   -1.00  1.00  -1.00  2.00;
   -1.00  2.00  -2.00  2.00;
   -1.00 -1.00  -0.50  0.00;
   -0.50  0.00   0.50  0.00;
    0.50  0.00   1.00  1.00
];

maxV = 0.5;         % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to robot center (m)
fwdVel = 0.2;       % Forward velocity (m/s)
angVel = 0;         % Initial angular velocity (rad/s)
turnAngle = -pi/6;  % Clockwise turn of 30° (-π/6 rad)
backupDist = -0.25; % Reverse 0.25m
backupVel = -0.15;  % Speed while reversing (m/s)
turnSpeed = -0.4;   % Speed for turning (rad/s)

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);

tic;
while toc < maxTime
    % Read sensor data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);

    % --- PARTICLE FILTER + DEBUG PLOT ---
    particles = particleFilterStep(map);    % now return particles
    figure(1); clf; hold on; axis equal;
    % draw walls
    for i = 1:size(map,1)
        plot( map(i,[1,3])', map(i,[2,4])', 'k-','LineWidth',1 );
    end
    % draw particles
    scatter( particles(:,1), particles(:,2), 5, '.', 'MarkerEdgeAlpha', 0.2 );
    drawnow;


    % Check for bump
    if isfield(dataStore, 'bump') && ~isempty(dataStore.bump)
        bumpDetected = any(dataStore.bump(end, 2:end));
    else
        bumpDetected = false;
    end
    
    if bumpDetected
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);
        
        [cmdV, cmdW] = limitCmds(backupVel, 0, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(abs(backupDist / backupVel));
        SetFwdVelAngVelCreate(Robot, 0, 0);
        
        [cmdV, cmdW] = limitCmds(0, turnSpeed, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(turnAngle / cmdW);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);
    end
    
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    pause(0.03);
end

% Stop robot before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end
