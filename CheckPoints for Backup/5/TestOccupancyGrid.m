function [lFinalBump,lFinalDepth]=TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% TESTOCCUPANCYGRID 
% Test function for MAE 4180/5180 CS 3758, Homework 5. 
% Returns and plots the final occupancy grids using the bump and depth sensors.
% Will create two (2) figures containing the occupancy grids.
%
%       TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
%
%       INPUTS:
%           dataStore   struct from running SimulatorGUI
%           l_0         initial log odds (scalar value)
%           NumCellsX   number of cells in the x direction (integer)
%           NumCellsY   number of cells in the y direction (integer)
%           boundaryX   boundary of the environment in the x direction.
%                       1 x 2 array [min_x max_x]
%           boundaryY   boundary of the environment in the Y direction.
%                       1 x 2 array [min_y max_y]
%       OUTPUTS:
%           lFinalBump  Final occupancy grid using bump sensor
%           lFinalDepth Final occupancy grid using depth sensor
%       Figures Created:
%           Figure 1    Occupancy grid using bump sensor
%           Figure 2    Occupancy grid from depth information

% Cornell University
% Autonomous Mobile Robots
% Homework #5
% NIRMAL, A J L A


%% Occupancy Grid Initialization
% Set the grid origin as the lower bound of the boundaries.
gridOrigin = [boundaryX(1), boundaryY(1)];
% Compute the cell size (meters per cell) assuming square cells.
mapResolution = (boundaryX(2) - boundaryX(1)) / NumCellsX;
% Initialize occupancy grid with the prior log-odds value.
initialLogOdds = l_0 * ones(NumCellsY, NumCellsX);  % (rows = NumCellsY, cols = NumCellsX)

%% Bump Sensor Mapping
% Bump sensor model parameters.
bumpSensorParams.bumpHit = 0.4;      % Log-odds increment for a bump detection.
bumpSensorParams.bumpRange = 0.16;     % Bump sensor contact range in meters.

% Initialize final bump occupancy grid.
lFinalBump = initialLogOdds;
truthPose = dataStore.truthPose;   % [time, x, y, theta]
bumpData = dataStore.bump;         % [time, bumpRight, bumpLeft, ..., bumpFront, ...]
numBump = size(bumpData, 1);

% Update grid for each bump measurement.
for i = 1:numBump
    currentTime = bumpData(i, 1);
    % Use columns 2 (BumpRight), 3 (BumpLeft), and 7 (BumpFront).
    bumpSensorData = [bumpData(i,2), bumpData(i,3), bumpData(i,7)];
    % Find the closest robot pose.
    [~, poseIdx] = min(abs(truthPose(:,1) - currentTime));
    robotPose = truthPose(poseIdx, 2:4);  % [x, y, theta]
    % Update occupancy grid using the bump sensor.
    lFinalBump = logOddsBump(lFinalBump, robotPose, bumpSensorData, mapResolution, gridOrigin, bumpSensorParams);
end

%% Depth Sensor Mapping
% Depth sensor model parameters.
depthSensorParams.depthHit = 0.85;     % Log-odds increment for an occupied (hit) cell.
depthSensorParams.depthMiss = -0.4;    % Log-odds decrement for free cells.
depthSensorParams.depthMaxRange = 3.0;   % Maximum reliable depth sensor range (meters).
sensorOrigin = [0, 0.08];              % Sensor origin in the robot frame.
% Lidar sensor: 9 beams evenly spaced from -27° to 27°.
sensorAngles = deg2rad(linspace(-27, 27, 9))';

% Initialize final depth occupancy grid.
lFinalDepth = initialLogOdds;
rsdepthData = dataStore.rsdepth;   % [time, d1, d2, ..., d9]
numDepth = size(rsdepthData, 1);

% Update grid for each depth measurement.
for i = 1:numDepth
    currentTime = rsdepthData(i, 1);
    % Find closest robot pose.
    [~, poseIdx] = min(abs(truthPose(:,1) - currentTime));
    robotPose = truthPose(poseIdx, 2:4);  % [x, y, theta]
    % Update occupancy grid using depth sensor data.
    % Since testRun does not have map data, we pass an empty array [].
    lFinalDepth = logOddsDepth(lFinalDepth, robotPose, [], sensorOrigin, sensorAngles, mapResolution, gridOrigin, depthSensorParams);
end

%% Plotting the Final Occupancy Grids

% Plot the final grid from bump sensor data.
figure;
probBump = exp(lFinalBump) ./ (1 + exp(lFinalBump)); % Convert log-odds to probability.
imagesc(boundaryX, boundaryY, probBump);
colormap(flipud(gray));
colorbar;
axis equal tight;
title('Final Occupancy Grid from Bump Sensor');
xlabel('X (m)');
ylabel('Y (m)');

% Plot the final grid from depth sensor data.
figure;
probDepth = exp(lFinalDepth) ./ (1 + exp(lFinalDepth));
imagesc(boundaryX, boundaryY, probDepth);
colormap(flipud(gray));
colorbar;
axis equal tight;
title('Final Occupancy Grid from Depth Sensor');
xlabel('X (m)');
ylabel('Y (m)');

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Function: logOddsBump
function newLogOdds = logOddsBump(prevLogOdds, robotPose, bumpSensorData, mapResolution, gridOrigin, sensorModelParams)
% logOddsBump updates the occupancy grid's log-odds based on bump sensor measurements.
%
%   newLogOdds = logOddsBump(prevLogOdds, robotPose, bumpSensorData, mapResolution, gridOrigin, sensorModelParams)
%
%   INPUTS:
%       prevLogOdds       - n-by-m matrix of current log-odds occupancy values.
%       robotPose         - 1x3 vector [x, y, theta] representing the robot's pose.
%       bumpSensorData    - 1x3 vector [BumpRight, BumpLeft, BumpFront] (1 if triggered, 0 otherwise).
%       mapResolution     - Scalar size (meters) of each grid cell.
%       gridOrigin        - 1x2 vector [x0, y0] corresponding to grid cell (1,1) in world coordinates.
%       sensorModelParams - Struct with:
%                             .bumpHit   : log-odds increment when a bump is detected.
%                             .bumpRange : Distance from the robot's center to the sensor contact point.
%
%   OUTPUT:
%       newLogOdds        - Updated occupancy grid (n-by-m matrix) in log-odds.

newLogOdds = prevLogOdds;
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% Sensor directions relative to the robot's heading:
% BumpRight -> theta - pi/2, BumpLeft -> theta + pi/2, BumpFront -> theta.
sensorAngles = [theta - pi/2, theta + pi/2, theta];

[n, m] = size(prevLogOdds);
for sensorIdx = 1:3
    if bumpSensorData(sensorIdx) == 1
        % Compute world coordinates of bump contact point.
        sensor_x = x + sensorModelParams.bumpRange * cos(sensorAngles(sensorIdx));
        sensor_y = y + sensorModelParams.bumpRange * sin(sensorAngles(sensorIdx));
        
        % Convert world coordinates to grid indices.
        col = floor((sensor_x - gridOrigin(1)) / mapResolution) + 1;
        row = floor((sensor_y - gridOrigin(2)) / mapResolution) + 1;
        
        if row >= 1 && row <= n && col >= 1 && col <= m
            newLogOdds(row, col) = newLogOdds(row, col) + sensorModelParams.bumpHit;
        else
            warning('Bump sensor %d contact point (%.2f, %.2f) is outside grid bounds.', sensorIdx, sensor_x, sensor_y);
        end
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Function: logOddsDepth
function newLogOdds = logOddsDepth(prevLogOdds, robotPose, map, sensorOrigin, sensorAngles, mapResolution, gridOrigin, sensorModelParams)
% logOddsDepth updates the occupancy grid's log-odds using depth sensor measurements.
%
%   newLogOdds = logOddsDepth(prevLogOdds, robotPose, map, sensorOrigin, sensorAngles, mapResolution, gridOrigin, sensorModelParams)
%
%   INPUTS:
%       prevLogOdds       - n-by-m matrix of current log-odds occupancy values.
%       robotPose         - 1x3 vector [x, y, theta] representing the robot's pose.
%       map               - N-by-4 matrix of wall endpoints [x1, y1, x2, y2]. (Not used if empty.)
%       sensorOrigin      - 1x2 vector [x, y] representing the sensor's origin in the robot frame.
%       sensorAngles      - Kx1 vector of beam angles (radians) relative to the sensor frame.
%       mapResolution     - Scalar size (meters) of each grid cell.
%       gridOrigin        - 1x2 vector [x0, y0] corresponding to grid cell (1,1) in world coordinates.
%       sensorModelParams - Struct with:
%                             .depthHit      : log-odds increment for an occupied cell.
%                             .depthMiss     : log-odds decrement for free cells.
%                             .depthMaxRange : Maximum reliable depth sensor range.
%
%   OUTPUT:
%       newLogOdds        - Updated occupancy grid (n-by-m matrix) in log-odds.
%
%   This function uses a ray-tracing approach: it leverages depthPredict to compute expected
%   depth measurements, then updates the hit cell (if the measured depth is less than depthMaxRange)
%   as occupied and cells along the beam as free.

% Get expected depth measurements using the provided depthPredict function.
depthSensorData = depthPredict(robotPose, map, sensorOrigin, sensorAngles);

newLogOdds = prevLogOdds;
[n, m] = size(prevLogOdds);
K = length(depthSensorData);
for beam = 1:K
    r = depthSensorData(beam);
    beamAngle = robotPose(3) + sensorAngles(beam);
    
    if r < sensorModelParams.depthMaxRange
        r_end = r;
        % Update hit cell.
        hitX = robotPose(1) + r * cos(beamAngle);
        hitY = robotPose(2) + r * sin(beamAngle);
        colHit = floor((hitX - gridOrigin(1)) / mapResolution) + 1;
        rowHit = floor((hitY - gridOrigin(2)) / mapResolution) + 1;
        if rowHit >= 1 && rowHit <= n && colHit >= 1 && colHit <= m
            newLogOdds(rowHit, colHit) = newLogOdds(rowHit, colHit) + sensorModelParams.depthHit;
        end
    else
        r_end = sensorModelParams.depthMaxRange;
    end
    
    % Update cells along the beam as free.
    stepSize = mapResolution / 2;
    for r_free = 0:stepSize:(r_end - mapResolution)
        freeX = robotPose(1) + r_free * cos(beamAngle);
        freeY = robotPose(2) + r_free * sin(beamAngle);
        colFree = floor((freeX - gridOrigin(1)) / mapResolution) + 1;
        rowFree = floor((freeY - gridOrigin(2)) / mapResolution) + 1;
        if rowFree >= 1 && rowFree <= n && colFree >= 1 && colFree <= m
            newLogOdds(rowFree, colFree) = newLogOdds(rowFree, colFree) + sensorModelParams.depthMiss;
        end
    end
end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Function: depthPredict
function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Antonyselvaraj Jhon Leonard Arputha , Nirmal

    K = length(angles);
    depth = inf(K, 1); % Initialize all depths to infinity

    % Extract robot pose
    x_r = robotPose(1);
    y_r = robotPose(2);
    theta_r = robotPose(3);

    % Compute sensor position in global frame
    x_s = x_r + sensorOrigin(1) * cos(theta_r) - sensorOrigin(2) * sin(theta_r);
    y_s = y_r + sensorOrigin(1) * sin(theta_r) + sensorOrigin(2) * cos(theta_r);

    % Loop through each sensor angle
    for k = 1:K
        % Compute ray direction in global frame
        theta_s = theta_r + angles(k);
        x_end = x_s + 100 * cos(theta_s); % Extend the ray for intersection checking
        y_end = y_s + 100 * sin(theta_s);

        % Check intersections with each wall
        for i = 1:size(map, 1)
            % Extract wall endpoints
            x1 = map(i, 1); y1 = map(i, 2);
            x2 = map(i, 3); y2 = map(i, 4);
            % disp(x1);disp(y1);disp(x2);disp(y2);

            % Compute intersection
            denom = (y2 - y1) * (x_end - x_s) - (x2 - x1) * (y_end - y_s);
            if denom == 0
                continue; % Parallel lines
            end

            ua = ((x2 - x1) * (y_s - y1) - (y2 - y1) * (x_s - x1)) / denom;
            ub = ((x_end - x_s) * (y_s - y1) - (y_end - y_s) * (x_s - x1)) / denom;

            if ua >= 0 && ub >= 0 && ub <= 1
                % Compute intersection point
                xi = x_s + ua * (x_end - x_s);
                yi = y_s + ua * (y_end - y_s);
                d = sqrt((xi - x_s)^2 + (yi - y_s)^2);
                d = d  * cos(angles(k));
                depth(k) = min(depth(k), d); % Keep the closest intersection
            end
        end
    end
end

