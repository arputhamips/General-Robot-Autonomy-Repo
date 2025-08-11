function [dataStore] = backupBump(Robot, maxTime)
% backupBump – drive forward, back‑up on bump, localise with PF, and plot particles/truth
%
% USAGE: backupBump(simRobot, 300);

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;

load('practicemap2025update.mat');


dataStore = struct('truthPose',[], 'odometry',[], 'rsdepth',[], 'bump',[], 'beacon',[]);

SetFwdVelAngVelCreate(Robot,0,0);

initialPose = getInitialPose(Robot, map, optWalls, beaconLoc, waypoints, ECwaypoints);
[~, dataStore] = readStoreSensorData(Robot,0,dataStore);





tic; % main clock
while toc < maxTime
  % 1) read sensors safely
  [~, dataStore] = readStoreSensorData(Robot,0,dataStore);

   dataStore = updateEKF(dataStore, initialPose, map, optWalls, waypoints,ECwaypoints, beaconLoc);
 
    % dataStore = updateRobustLocalization(dataStore, initialPose, map, optWalls, waypoints, beaconLoc);
    %     % Check if orientation scan is needed
    % if isfield(dataStore, 'localization') && isfield(dataStore.localization, 'needOrientationScan') && ...
    %    dataStore.localization.needOrientationScan
    %     % Perform orientation scan
    %     disp('Performing orientation scan to improve localization...');
    % 
    %     % Save current position and stop robot
    %     currentPose = dataStore.truthPose(end, 2:4)';
    %     savedPos = currentPose(1:2);
    %     SetFwdVelAngVelCreate(Robot, 0, 0);
    %     pause(0.5);
    % 
    %     % Rotate slowly in place to collect depth data
    %     SetFwdVelAngVelCreate(Robot, 0, 0.3);
    % 
    %     % Rotate for about 1 full revolution
    %     for i = 1:24
    %         pause(2*pi/24/0.3);  % Time to rotate 15 degrees
    % 
    %         % Collect data
    %         [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    % 
    %         % Update pose without moving (just update orientation)
    %         dataStore = updateRobustLocalization(dataStore, initialPose, map, optWalls, waypoints, beaconLoc);
    %     end
    % 
    %     % Stop robot
    %     SetFwdVelAngVelCreate(Robot, 0, 0);
    % 
    %     % Reset orientation scan flag
    %     dataStore.localization.needOrientationScan = false;
    % 
    %     % Restore position (keep updated orientation)
    %     currentPose = dataStore.truthPose(end, 2:4)';
    %     currentPose(1:2) = savedPos;
    %     dataStore.localization.state(1:2) = savedPos;
    %     dataStore.truthPose(end, 2:3) = savedPos';
    % 
    %     disp('Orientation scan complete.');
    % end

   SetFwdVelAngVelCreate(Robot, 0.1, 0);
   pause(0.1);

end

SetFwdVelAngVelCreate(Robot,0,0);

end

