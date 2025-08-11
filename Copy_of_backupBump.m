function [dataStore] = backupBump(Robot, maxTime)
% backupBump  –  drive forward, back‑up on bump, localise with PF,
%                and plot particles/truth in a *separate* figure
%                so it doesn’t clash with the simulator window.
%
%   USAGE:  backupBump(simRobot, 300);

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

global dataStore;

% ----- load map / waypoints / beacons -----------------------------------
load ('practicemap2025update.mat');


dataStore = struct('truthPose',[], 'odometry',[], 'rsdepth',[], ...
                   'bump',[],     'beacon',[]);

SetFwdVelAngVelCreate(Robot,0,0);

% motion constants
maxV   = 0.5;  wheel2Center = 0.13;
fwdVel = 0.2;  angVel = 0;
backupVel = -0.15;  backupDist = -0.35;
turnSpeed = -0.4;   turnAngle  = -pi/4;

[~,dataStore] = readStoreSensorData(Robot,0,dataStore);
dataStore = pfInitial(dataStore, map, waypoints, optWalls, beaconLoc);


% -------- create a dedicated debug figure -------------------------------

   % plot timer
tic;            % main timer
while toc < maxTime
    [~,dataStore] = readStoreSensorData(Robot,0,dataStore);
    % dataStore    = particleFilterUpdate(Robot,dataStore,map,beaconLoc,optWalls,waypoints);
    % inside your main control loop
    % dataStore = particleFilterUpdate(dataStore, Robot, map, beaconLoc, optWalls, waypoints);

    % % bump‑and‑backup behaviour
    % bumpDetected = ~isempty(dataStore.bump) && any(dataStore.bump(end,2:end));
    % if bumpDetected
    %     [v,w] = limitCmds(backupVel,0,maxV,wheel2Center);
    %     SetFwdVelAngVelCreate(Robot,v,w);
    %     pause(abs(backupDist/backupVel));
    %     SetFwdVelAngVelCreate(Robot,0,0);
    % 
    %     [v,w] = limitCmds(0,turnSpeed,maxV,wheel2Center);
    %     SetFwdVelAngVelCreate(Robot,v,w);
    %     pause(abs(turnAngle/w));
    %     SetFwdVelAngVelCreate(Robot,0,0);
    % end
    % 
    % % forward drive
    % [v,w] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
    % SetFwdVelAngVelCreate(Robot,v,w);

    pause(0.1);
end
SetFwdVelAngVelCreate(Robot,0,0);
end  % ============================== end backupBump ======================


