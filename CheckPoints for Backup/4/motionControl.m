function [dataStore] = motionControl(Robot, maxTime)
% Cornell University - MAE 5180
% ANTONYSELVARAJ JHON LEONARD AR , NIRMAL
%
% motionControl: Controls the robot, handling bump detection and switching 
% between EKF with GPS, EKF with Depth, and PF with Depth.
%
% INPUTS:
%   Robot   - Robot hardware or simulator object
%   maxTime - Maximum runtime for motion control (optional, default: 500s)
%
% OUTPUT:
%   dataStore - Struct containing stored sensor readings, localization data, etc.

% Boilerplate Setup
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    % When running with the real robot, define the appropriate ports
    CreatePort = Robot.CreatePort;
catch
    % If not real robot, use simulator object
    CreatePort = Robot;
end

% Declare dataStore as a global variable
global dataStore;

% Initialize Data Store
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'GPS',[], ...
                   'particles',[]);


[~, dataStore] = readStoreSensorData(Robot, 0, dataStore);

% initpose = dataStore.truthPose(1, 2:4);
% dataStore.deadReck = initpose;
% dataStore.ekfMu = initpose;

% EKF & PF Initialization
% dataStore.ekfMu = [0, 0, 0];  % EKF Mean (robot pose)

R = 0.1 * eye(3); % Process noise covariance
Q = 0.09 * eye(3); % Measurement noise covariance

% Initialize Particles (for Particle Filter)
N_particles = 50;
%   - weight: uniform weight (1/N_particles)
initialParticles = [unifrnd(-5, 0, N_particles, 1), ...
                    unifrnd(-5, 5, N_particles, 1), ...
                    unifrnd(-0.2, 0.2, N_particles, 1), ...
                    ones(N_particles, 1) / N_particles];
initialParticles = repmat([dataStore.truthPose(1,2:4),1/N_particles],N_particles);


% Store the initial particle set as the first time step (T = 1)
dataStore.particles(:,:,1) = initialParticles;


% Motion Parameters
maxV = 0.5;         % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to robot center (m)
fwdVel = 0.25;       % Forward velocity (m/s)
angVel = 0;         % Initial angular velocity (rad/s)
turnAngle = -pi/6;  % Clockwise turn of 30° (-π/6 rad)
backupDist = -0.25; % Reverse 0.25m
backupVel = -0.15;  % Speed while reversing (m/s)
turnSpeed = -0.4;   % Speed for turning (rad/s)

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);

% Timer Start
tic;
while toc < maxTime
    % Read Sensor Data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);


    % Check for Bump Sensor Trigger
    if isfield(dataStore, 'bump') && ~isempty(dataStore.bump)
        bumpDetected = any(dataStore.bump(end, 2:end));
    else
        bumpDetected = false;
    end

    if bumpDetected
        % Stop Robot
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);

        % Move Backward
        backupTime = abs(backupDist / backupVel);
        [cmdV, cmdW] = limitCmds(backupVel, 0, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(backupTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after backing up

        % Rotate in Place
        [cmdV, cmdW] = limitCmds(0, turnSpeed, maxV, wheel2Center);
        turnTime = turnAngle / cmdW;
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(turnTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after turning

        % Pause before resuming
        pause(0.5);
    end

    
    % Continue Driving Forward
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    pause(0.1);


    % -----------

    % create deadreck data
    dataStore.deadReck = integrateOdom(dataStore.truthPose(1,2:4), ...
                                    [dataStore.odometry(:,2)]', ...
                                    [dataStore.odometry(:,3)]');
    % dataStore.deadReck = integrateOdom([-2, -1.5, pi/2], ...
    %                                 [dataStore.odometry(:,2)]', ...
    %                                 [dataStore.odometry(:,3)]');
    dataStore.deadReck = dataStore.deadReck';

    % add GPS noise
    gpsNoise = mvnrnd([0, 0, 0], Q); % Gaussian noise with mean 0 and covariance Q
    noisyGPS = dataStore.truthPose(end, 2:4) + gpsNoise; % Add noise to true pose
    dataStore.GPS = [dataStore.GPS; toc, noisyGPS];     % Store in dataStore.GPS

    if isempty(dataStore.ekfMu)
        dataStore.ekfMu = dataStore.truthPose(1, 2:4);
        % dataStore.ekfMu = [-2, -1.5, pi/2];
    end       
    
    dataStore.ekfSigma(:,:,1) = [2, 0, 0; 0, 2, 0; 0, 0, 0.2];
    % dataStore.ekfSigma(:,:,1) = [4, 0, 0; 0, 4, 0; 0, 0, 0.02]; % EKF Covariance

    % ---- CHOOSE LOCALIZATION METHOD ----
    % Uncomment the desired method:



    % INPUTS FOR EKF
    map = [5,  2,  1,  2;
           1,  3,  1,  2;
           -1,  5, -1,  1;
           -5,  1, -3,  1;
            1, -3,  1, -1;
            1, -1,  3, -1;
           -1, -4, -1, -5;
           -3, -2, -1, -2;
           -5,  5,  5,  5;
            5,  5,  5, -5;
            5, -5, -5, -5;
           -5, -5, -5,  5];
    
    if ~isempty(dataStore.GPS)
        % Extract last recorded noisy GPS data
        z_gps = dataStore.GPS(end, 2:4)';  % Ensure it is a (3x1) column vector
    else
        z_gps = dataStore.truthPose(end, 2:4)'; % If no GPS data, use truthPose (for initialization)
    end

    % Extract last odometry reading for control input
    if isfield(dataStore, 'odometry') && ~isempty(dataStore.odometry)
        ut = [dataStore.odometry(end, 2); dataStore.odometry(end, 3)]; % [d; phi]
    else
        ut = [0; 0];  % Default to zero motion if no odometry data
    end

    sigma_input = dataStore.ekfSigma(:,:,1);
    
    z_depth = dataStore.rsdepth(end,2:end);
    
    sigma_depth = 0.05;  % Standard deviation of depth noise (adjust based on sensor)
    Q_depth = (sigma_depth^2) * eye(9);
    
  
    
    % EKF FUNCITON
    % COMMENT OUT WHEN USING PF FUNCTION
    % ------------------------------
    % [mu_next_gps, sigma_next_gps, mu_next_depth, sigma_next_depth] = testEKF([dataStore.ekfMu(end,:)]', ut, sigma_input, R, ...
    %                                               z_gps, Q, z_depth', Q_depth, map, [0.13 0], 54); % No depth input for GPS update
    
    
    
    % ----------------------
    % EKF with GPS Data
    % Update dataStore with new EKF estimate
    % COMMENT NEXT THREE LINES OUT

    % mu_next_gps = mu_next_gps';
    % dataStore.ekfMu = [dataStore.ekfMu ; mu_next_gps];
    % dataStore.ekfSigma(:,:,end+1) = sigma_next_gps;


    % ---------------
    % EKF with depth Data
    % COMMENT NEXT THREE LINES OUT IF NEEDED


    % mu_next_depth = mu_next_depth';
    % dataStore.ekfMu = [dataStore.ekfMu ; mu_next_depth];
    % dataStore.ekfSigma(:,:,end+1) = sigma_next_depth;





   
    % Assume dataStore.particles(:,:,t) is the current set
    currentParticles = dataStore.particles(:,:,end); 
    % Call PF with appropriate function handles (using integrateOdom for predFunc)
    updatedParticles = PF(currentParticles, ut, z_depth', ...
                        @(x,u) integrateOdom(x, u(1), u(2)), @updateFunc, map, 54);
    % Append updatedParticles as the new time step in the datastore
    dataStore.particles(:,:,end+1) = updatedParticles;



end

% Stop Robot Before Exiting
SetFwdVelAngVelCreate(Robot, 0, 0);



end