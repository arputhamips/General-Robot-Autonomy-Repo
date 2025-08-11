function [dataStore] = localizeWithBumpHandling(Robot, maxTime)
% Localization with bump handling for robot in competition
% ANTONYSELVARAJ JHON LEONARD AR, NIRMAL

%----- Boilerplate (DO NOT CHANGE) -----
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end
try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end
noRobotCount = 0;
global dataStore;
dataStore = struct('truthPose', [], ...
                  'odometry', [], ...
                  'rsdepth', [], ...
                  'bump', [], ...
                  'beacon', [], ...
                  'trajectory', [], ...
                  'RRT', [], ...
                  'particles', [], ...
                  'detectedWalls', []); % Added detectedWalls field
%----- End Boilerplate -----

% Load the map file
load('practicemap2025update.mat'); % This loads map, optWalls, waypoints, ECwaypoints, beaconLoc

% Initialize detectedWalls array
dataStore.detectedWalls = false(size(optWalls, 1), 1);

% Display map information
fprintf('Loaded map with %d walls, %d optional walls, %d waypoints, %d ECwaypoints, and %d beacons\n', ...
    size(map, 1), size(optWalls, 1), size(waypoints, 1), size(ECwaypoints, 1), size(beaconLoc, 1));

% Setup figure for visualization
h_fig = figure(1);
clf;
set(h_fig, 'Position', [100, 100, 800, 600]); % Make figure larger
ax = axes();
hold(ax, 'on');

% Plot walls
for i = 1:size(map, 1)
    line(ax, [map(i,1), map(i,3)], [map(i,2), map(i,4)], 'Color', 'k', 'LineWidth', 2);
end

% Plot optional walls with dashed lines
for i = 1:size(optWalls, 1)
    line(ax, [optWalls(i,1), optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--');
end

% Plot waypoints
scatter(ax, waypoints(:,1), waypoints(:,2), 50, 'b', 'filled', 'DisplayName', 'Waypoints');

% Plot beacons
if ~isempty(beaconLoc)
    scatter(ax, beaconLoc(:,2), beaconLoc(:,3), 80, 'g', 'd', 'filled', 'DisplayName', 'Beacons');
end

% Create empty plots for updating later
h_particles = scatter(ax, [], [], 10, 'b', '.', 'DisplayName', 'Particles');
h_best_particle = plot(ax, NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best Particle');
h_robot_dir = quiver(ax, NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Robot Direction');

% Add legend and labels
legend('show', 'Location', 'northeastoutside');
axis(ax, 'equal');
grid(ax, 'on');
title(ax, 'Particle Filter Visualization');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');

% Parameters for movement
maxV = 0.5;          % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to robot center (m)
fwdVel = 0.1;        % Forward velocity (m/s), slower for more accurate sensing
angVel = 0.0;        % Initial angular velocity (rad/s)
backupDist = -0.25;  % Reverse 0.25m when hitting obstacle
backupVel = -0.15;   % Speed while reversing (m/s)
turnAngle = -pi/6;   % Turn angle after backup (radians)
turnSpeed = -0.4;    % Speed for turning (rad/s)

% Set confidence threshold and maximum duration
targetConfidence = 5.0;   % Target confidence level
maxDuration = 30;        % Maximum 30 seconds to localize
t0 = tic;
lastReportTime = 0;
lastPlotTime = 0;
confidence = 0;
bumpHandled = false;

% Begin localization with motion
SetFwdVelAngVelCreate(Robot, fwdVel, angVel);
fprintf('Starting localization with slow forward motion...\n');

while toc(t0) < maxDuration && confidence < targetConfidence
    currentTime = toc(t0);
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore, currentTime);
    
    % Check for collision and handle it
    if ~isempty(dataStore.bump) && size(dataStore.bump, 1) > 0 && any(dataStore.bump(end, 2:end))
        disp('Collision detected! Handling bump...');
        bumpHandled = true;
        
        % Stop robot immediately
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(0.5);
        
        % Move backward
        backupTime = abs(backupDist / backupVel);
        [cmdV, cmdW] = limitCmds(backupVel, 0, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(backupTime);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        
        % Update particles to reflect collision
        dataStore.particles = handleCollision(dataStore.particles);
        
        % Rotate in place
        [cmdV, cmdW] = limitCmds(0, turnSpeed, maxV, wheel2Center);
        turnTime = abs(turnAngle / turnSpeed);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(turnTime);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        
        % Resume forward motion
        [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
    
    % Run the particle filter
    particles = dataStore.particles;
    particleFilter(map, optWalls, beaconLoc, waypoints, currentTime);
    
    % Calculate current confidence
    if ~isempty(dataStore.particles) && length(dataStore.particles) > 0
        weights = [dataStore.particles.weight];
        if ~isempty(weights)
            maxWeight = max(weights);
            sumWeights = sum(weights);
            confidence = maxWeight / sumWeights * length(dataStore.particles);
        end
    end
    
    % Update visualization
    if currentTime - lastPlotTime >= 0.2 && ~isempty(dataStore.particles) && length(dataStore.particles) > 0
        % Extract particle data
        particleX = zeros(1, length(dataStore.particles));
        particleY = zeros(1, length(dataStore.particles));
        particleW = zeros(1, length(dataStore.particles));
        
        for i = 1:length(dataStore.particles)
            particleX(i) = dataStore.particles(i).pose(1);
            particleY(i) = dataStore.particles(i).pose(2);
            particleW(i) = dataStore.particles(i).weight;
        end
        
        % Update particle plot with size based on weight
        normalizedWeights = (particleW / max(particleW + 1e-10)) * 30 + 3;
        set(h_particles, 'XData', particleX, 'YData', particleY, 'SizeData', normalizedWeights);
        
        % Update best particle
        [~, bestIdx] = max(particleW);
        bestPose = dataStore.particles(bestIdx).pose;
        set(h_best_particle, 'XData', bestPose(1), 'YData', bestPose(2));
        
        % Update direction arrow
        arrowLength = 0.3;
        set(h_robot_dir, 'XData', bestPose(1), 'YData', bestPose(2), ...
            'UData', arrowLength * cos(bestPose(3)), 'VData', arrowLength * sin(bestPose(3)));
        
        % Set dynamic title with confidence
        title(ax, sprintf('Particle Filter (t=%.1fs) - Confidence: %.2f', currentTime, confidence));
        
        % Adjust axis limits to follow robot position
        axis(ax, [bestPose(1)-3, bestPose(1)+3, bestPose(2)-3, bestPose(2)+3]);
        
        % Update display
        drawnow;
        lastPlotTime = currentTime;
    end
    
    % Print progress every second
    if currentTime - lastReportTime >= 1 && ~isempty(dataStore.truthPose)
        currentPose = dataStore.truthPose(end, 2:4);
        fprintf('t=%.1fs: PF pose: x=%.3f, y=%.3f, theta=%.3f, confidence=%.2f\n', ...
            currentTime, currentPose(1), currentPose(2), currentPose(3), confidence);
        lastReportTime = currentTime;
    end
    
    % Small pause
    pause(0.05);
    
    % If we've been localizing for a while and confidence is still low,
    % try a pure rotation to get more sensor data
    if currentTime > 10 && confidence < 1.0 && ~bumpHandled
        fprintf('Low confidence after 10 seconds, switching to rotation...\n');
        SetFwdVelAngVelCreate(Robot, 0, 0.4); % Pure rotation
        pause(3); % Rotate for 3 seconds
        SetFwdVelAngVelCreate(Robot, fwdVel, angVel); % Resume forward motion
        bumpHandled = true; % Don't repeat this
    end
end

% Stop the robot
SetFwdVelAngVelCreate(Robot, 0, 0);

% Create a final visualization
figure(2);
clf;
h_fig2 = gcf;
set(h_fig2, 'Position', [150, 150, 800, 600]);
ax2 = axes();
hold(ax2, 'on');

% Plot the map
for i = 1:size(map, 1)
    line(ax2, [map(i,1), map(i,3)], [map(i,2), map(i,4)], 'Color', 'k', 'LineWidth', 2);
end

% Plot detected and optional walls
for i = 1:size(optWalls, 1)
    if dataStore.detectedWalls(i)
        line(ax2, [optWalls(i,1), optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], ...
            'Color', 'g', 'LineWidth', 2, 'LineStyle', '-', 'DisplayName', sprintf('Detected Wall %d', i));
    else
        line(ax2, [optWalls(i,1), optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], ...
            'Color', 'r', 'LineWidth', 1, 'LineStyle', '--', 'DisplayName', sprintf('Optional Wall %d', i));
    end
end

% Plot all particles
if ~isempty(dataStore.particles) && length(dataStore.particles) > 0
    particleX = [dataStore.particles.pose];
    particleX = particleX(1:3:end);
    particleY = [dataStore.particles.pose];
    particleY = particleY(2:3:end);
    particleW = [dataStore.particles.weight];
    
    % Plot particles with size based on weight
    scatter(ax2, particleX, particleY, (particleW/max(particleW+1e-10))*50+5, 'b', '.', 'DisplayName', 'Particles');
    
    % Plot best particle and its direction
    [~, bestIdx] = max(particleW);
    bestPose = dataStore.particles(bestIdx).pose;
    plot(ax2, bestPose(1), bestPose(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best Particle');
    quiver(ax2, bestPose(1), bestPose(2), 0.5*cos(bestPose(3)), 0.5*sin(bestPose(3)), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Direction');
end

% Add waypoints and beacons
scatter(ax2, waypoints(:,1), waypoints(:,2), 50, 'c', 'filled', 'DisplayName', 'Waypoints');
if ~isempty(beaconLoc)
    scatter(ax2, beaconLoc(:,2), beaconLoc(:,3), 80, 'g', 'd', 'filled', 'DisplayName', 'Beacons');
end

% Add trajectory if available
if ~isempty(dataStore.truthPose) && size(dataStore.truthPose, 1) > 1
    plot(ax2, dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'm-', 'LineWidth', 2, 'DisplayName', 'Trajectory');
end

title(ax2, sprintf('Final State - Confidence: %.2f', confidence));
xlabel(ax2, 'X (m)');
ylabel(ax2, 'Y (m)');
axis(ax2, 'equal');
grid(ax2, 'on');
legend('show', 'Location', 'northeastoutside');

% Print final results
if ~isempty(dataStore.truthPose)
    finalPose = dataStore.truthPose(end, 2:4);
    fprintf('\nFinal PF pose after localization: x = %.3f, y = %.3f, theta = %.3f\n', ...
        finalPose(1), finalPose(2), finalPose(3));
    
    % Report detected walls if any
    if any(dataStore.detectedWalls)
        fprintf('Detected %d optional walls during localization\n', sum(dataStore.detectedWalls));
        detectedWallIdx = find(dataStore.detectedWalls);
        fprintf('Detected optional walls indices: ');
        fprintf('%d ', detectedWallIdx);
        fprintf('\n');
    else
        fprintf('No optional walls detected during localization\n');
    end
    
    % Report final confidence
    fprintf('Final localization confidence: %.2f\n', confidence);
    
    if confidence >= targetConfidence
        fprintf('Successfully reached target confidence threshold (%.2f)\n', targetConfidence);
    else
        fprintf('Warning: Did not reach target confidence threshold (%.2f) in allocated time\n', targetConfidence);
    end
else
    fprintf('Warning: No pose estimate available after localization\n');
end

end

% COLLISION HANDLING FUNCTION
function particles = handleCollision(particles)
    if isempty(particles)
        return;
    end
    
    N = length(particles);
    
    % Get current best estimate
    weights = [particles.weight];
    [~, bestIdx] = max(weights);
    bestPose = particles(bestIdx).pose;
    
    % Spread particles when collision is detected, but maintain general position
    for i = 1:N
        if rand() < 0.3  % Randomly adjust 30% of particles
            particles(i).pose(1) = particles(i).pose(1) + 0.1 * randn();
            particles(i).pose(2) = particles(i).pose(2) + 0.1 * randn();
            particles(i).pose(3) = particles(i).pose(3) + 0.3 * randn();
        end
    end
    
    % Reset weights to prevent overconfidence in a potentially wrong position
    for i = 1:N
        particles(i).weight = 1/N;
    end
    
    fprintf('Collision handled: Particles dispersed around (%.2f, %.2f)\n', bestPose(1), bestPose(2));
end

% LIMIT COMMANDS HELPER FUNCTION
function [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center)
    % This function limits the forward and angular velocity commands
    % to ensure they are within the robot's capabilities
    
    % Convert to wheel velocities
    rightWheelVel = fwdVel + wheel2Center * angVel;
    leftWheelVel = fwdVel - wheel2Center * angVel;
    
    % Check if wheel velocities exceed the maximum
    if abs(rightWheelVel) > maxV || abs(leftWheelVel) > maxV
        % Scale both velocities down by the same factor
        scale = max(abs(rightWheelVel), abs(leftWheelVel)) / maxV;
        rightWheelVel = rightWheelVel / scale;
        leftWheelVel = leftWheelVel / scale;
        
        % Convert back to forward and angular velocity
        cmdV = (rightWheelVel + leftWheelVel) / 2;
        cmdW = (rightWheelVel - leftWheelVel) / (2 * wheel2Center);
    else
        cmdV = fwdVel;
        cmdW = angVel;
    end
end