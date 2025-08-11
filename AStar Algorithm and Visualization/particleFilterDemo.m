function [dataStore] = particleFilterDemo(Robot, maxTime)
% Particle Filter Demo with Visualization
% Implements backupBump behavior with particle filtering and visualization

% Check input arguments
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 300; % Default max time: 5 minutes
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

% Initialize datalog struct
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);
               
% Robot Parameters
maxV = 0.5;         % Max wheel speed (m/s)
wheel2Center = 0.13; % Distance from wheel to robot center (m)
fwdVel = 0.2;        % Forward velocity (m/s)
angVel = 0;          % Initial angular velocity (rad/s)
turnAngle = -pi/6;   % Clockwise turn angle after bump (30 degrees)
backupDist = -0.25;  % Reverse distance after bump (meters)
backupVel = -0.15;   % Speed while reversing (m/s)
turnSpeed = -0.4;    % Speed for turning (rad/s)

% Particle filter parameters
numParticles = 100;  % Number of particles

% Get the map, beacons and optional walls from workspace
map = evalin('base', 'map');
beaconLoc = evalin('base', 'beaconLoc');
optWalls = evalin('base', 'optWalls');

% Make sure optWalls is in base workspace
assignin('base', 'optWalls', optWalls);

% Initialize visualization
figure(1);
clf;
h_map = subplot(1, 1, 1);
hold(h_map, 'on');
axis(h_map, 'equal');
grid(h_map, 'on');

% Plot the known map
plotMap(map, 'b', h_map);

% Plot optional walls in a different color
plotMap(optWalls, 'r--', h_map);

% Plot beacon locations
plotBeacons(beaconLoc, h_map);

% Initial pose estimate
x = 0;
y = 0;
theta = 0;

% Create accPose field for accumulated pose tracking
dataStore.accPose = [0, x, y, theta]; % [time, x, y, theta]

% Read initial sensor data
[~, dataStore] = readStoreSensorData(Robot, 0, dataStore);

% Initialize particle filter
dataStore = particleFilterUpdate(Robot, dataStore, map, beaconLoc, optWalls);

% Handles for visualization updates
h_particles = [];
h_robot = [];
h_laser = [];

% Stop robot before starting
SetFwdVelAngVelCreate(Robot, 0, 0);

tic;
lastLoopTime = toc;
% Visualization update tracking
lastVisUpdateTime = 0;

% Main control loop
while toc < maxTime
    currentTime = toc;
    deltaT = currentTime - lastLoopTime;
    lastLoopTime = currentTime;
    
    % Read sensor data
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Update accumulated pose based on odometry
    if size(dataStore.odometry, 1) > 0
        lastOdom = dataStore.odometry(end, :);
        
        if size(dataStore.accPose, 1) > 0
            lastPose = dataStore.accPose(end, :);
            
            % Extract the latest delta distance and angle
            deltaDist = lastOdom(2);
            deltaAngle = lastOdom(3);
            
            % Update position using simple odometry model
            lastX = lastPose(2);
            lastY = lastPose(3);
            lastTheta = lastPose(4);
            
            % Calculate new position
            newTheta = wrapToPi(lastTheta + deltaAngle);
            newX = lastX + deltaDist * cos(newTheta);
            newY = lastY + deltaDist * sin(newTheta);
            
            % Add to accumulated pose
            dataStore.accPose = [dataStore.accPose; currentTime, newX, newY, newTheta];
        end
    end
    
    % Update particle filter
    dataStore = particleFilterUpdate(Robot, dataStore, map, beaconLoc, optWalls);
    
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
        
        % Move backward for backupDist
        backupTime = abs(backupDist / backupVel);
        [cmdV, cmdW] = limitCmds(backupVel, 0, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(backupTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after backing up
        
        % Rotate in place
        [cmdV, cmdW] = limitCmds(0, turnSpeed, maxV, wheel2Center);
        turnTime = turnAngle / cmdW;
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        pause(turnTime);
        SetFwdVelAngVelCreate(Robot, 0, 0); % Stop after turning
        
        % Small pause before moving forward again
        pause(0.5);
    end
    
    % Continue driving forward
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    % Update visualization (every ~0.5 seconds to avoid slowing down execution)
    if (currentTime - lastVisUpdateTime) > 0.5
        updateVisualization(dataStore, map, optWalls, h_map, h_particles, h_robot, h_laser);
        lastVisUpdateTime = currentTime;
        drawnow;
    end
    
    % Small pause to keep the loop from running too fast
    pause(0.1);
end

% Stop robot before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

% Final visualization update
updateVisualization(dataStore, map, optWalls, h_map, h_particles, h_robot, h_laser);
drawnow;

end

function [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center)
    % Limit commands to avoid saturating motors

    % Compute wheel velocities
    velR = fwdVel + wheel2Center * angVel;
    velL = fwdVel - wheel2Center * angVel;
    
    % Check if wheel velocities exceed maximum
    if abs(velR) > maxV || abs(velL) > maxV
        % Find scaling factor
        scale = max(abs(velR), abs(velL)) / maxV;
        velR = velR / scale;
        velL = velL / scale;
        
        % Recompute cmd_V and cmd_W
        fwdVel = (velR + velL) / 2;
        angVel = (velR - velL) / (2 * wheel2Center);
    end
    
    cmdV = fwdVel;
    cmdW = angVel;
end

function plotMap(walls, lineSpec, ax)
    % Plot the walls of the map
    if isempty(walls)
        return;
    end
    
    for i = 1:size(walls, 1)
        plot(ax, [walls(i, 1), walls(i, 3)], [walls(i, 2), walls(i, 4)], lineSpec, 'LineWidth', 2);
    end
    
    % Set axis limits with some padding
    xMin = min(min(walls(:, 1)), min(walls(:, 3))) - 0.5;
    xMax = max(max(walls(:, 1)), max(walls(:, 3))) + 0.5;
    yMin = min(min(walls(:, 2)), min(walls(:, 4))) - 0.5;
    yMax = max(max(walls(:, 2)), max(walls(:, 4))) + 0.5;
    
    axis(ax, [xMin xMax yMin yMax]);
end

function plotBeacons(beaconLoc, ax)
    % Plot beacon locations
    if isempty(beaconLoc)
        return;
    end
    
    for i = 1:size(beaconLoc, 1)
        plot(ax, beaconLoc(i, 2), beaconLoc(i, 3), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
        text(ax, beaconLoc(i, 2) + 0.1, beaconLoc(i, 3) + 0.1, num2str(beaconLoc(i, 1)), 'FontSize', 10);
    end
end

function updateVisualization(dataStore, map, optWalls, h_map, h_particles, h_robot, h_laser)
    % Update visualization with current particles and robot position
    
    % Get current best pose estimate
    if isfield(dataStore, 'truthPose') && ~isempty(dataStore.truthPose)
        currentPose = dataStore.truthPose(end, 2:4);
        x = currentPose(1);
        y = currentPose(2);
        theta = currentPose(3);
    elseif isfield(dataStore, 'accPose') && ~isempty(dataStore.accPose)
        % Use accumulated odometry if no particle filter estimate yet
        currentPose = dataStore.accPose(end, 2:4);
        x = currentPose(1);
        y = currentPose(2);
        theta = currentPose(3);
    else
        return;
    end
    
    % Clear previous particles and robot visualization
    if ~isempty(h_particles)
        delete(h_particles);
    end
    if ~isempty(h_robot)
        delete(h_robot);
    end
    if ~isempty(h_laser)
        delete(h_laser);
    end
    
    % Plot particles
    if isfield(dataStore, 'particles') && ~isempty(dataStore.particles)
        px = [dataStore.particles.x];
        py = [dataStore.particles.y];
        h_particles = plot(h_map, px, py, 'g.', 'MarkerSize', 5);
        
        % Scale particle alpha (opacity) by weight
        weights = [dataStore.particles.weight];
        weights = weights / max(weights);
        
        % Display high-weight particles larger
        highWeightIdx = weights > 0.5;
        if any(highWeightIdx)
            plot(h_map, px(highWeightIdx), py(highWeightIdx), 'g.', 'MarkerSize', 10);
        end
        
        % Plot optional walls state from highest weight particle
        [~, bestIdx] = max(weights);
        bestParticle = dataStore.particles(bestIdx);
        
        if isfield(bestParticle, 'optWallsState')
            for i = 1:length(bestParticle.optWallsState)
                wallState = bestParticle.optWallsState(i);
                if wallState == 1 % Wall exists
                    wall = optWalls(i, :);
                    plot(h_map, [wall(1), wall(3)], [wall(2), wall(4)], 'g-', 'LineWidth', 2);
                elseif wallState == -1 % Wall doesn't exist
                    wall = optWalls(i, :);
                    plot(h_map, [wall(1), wall(3)], [wall(2), wall(4)], 'r:', 'LineWidth', 1);
                end
            end
        end
    end
    
    % Plot robot position and orientation
    robotRadius = 0.15; % ~6 inches in meters
    robotHeadingLength = 0.2;
    
    % Robot circle
    h_robot(1) = rectangle(h_map, 'Position', [x-robotRadius, y-robotRadius, 2*robotRadius, 2*robotRadius], ...
                        'Curvature', [1 1], 'EdgeColor', 'b', 'LineWidth', 2);
    
    % Robot heading line
    h_robot(2) = plot(h_map, [x, x + robotHeadingLength * cos(theta)], ...
                                [y, y + robotHeadingLength * sin(theta)], 'b-', 'LineWidth', 2);
    
    % Plot depth sensor rays if available
    if ~isempty(dataStore.rsdepth)
        latestDepth = dataStore.rsdepth(end, 2:end); % Skip time column
        numMeasurements = length(latestDepth);
        
        % Calculate sensor angles
        angleStep = 6.75 * pi/180; % 6.75 degrees between measurements
        startAngle = -27 * pi/180; % Starting at -27 degrees from center
        
        h_laser = [];
        for i = 1:numMeasurements
            angle = wrapToPi(theta + startAngle + (i-1) * angleStep);
            dist = latestDepth(i);
            
            % Skip invalid measurements
            if dist < 0.175 || dist > 10
                continue;
            end
            
            % End point of laser ray
            xEnd = x + dist * cos(angle);
            yEnd = y + dist * sin(angle);
            
            % Plot ray
            h_laser(end+1) = plot(h_map, [x, xEnd], [y, yEnd], 'r-', 'LineWidth', 1);
        end
    end
    
    % Add title with position information
    title(h_map, sprintf('Robot Position: (%.2f, %.2f, %.1f°) | Particles: %d', ...
                     x, y, theta * 180/pi, length(dataStore.particles)));
                 
    % Add legend
    legend(h_map, 'Known Walls', 'Optional Walls', 'Beacons', 'Particles', 'Robot', 'Depth Rays', ...
           'Location', 'NorthEastOutside');
end