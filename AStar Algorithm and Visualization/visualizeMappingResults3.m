function visualizeMappingResults3(dataStore)
% VISUALIZEMAPPINGRESULTS Creates a visualization of the mapping results
% based on the robot's recorded depth sensor data and compares it with the
% map of optional walls.
%
% Input:
%   dataStore - The data structure containing recorded sensor data

% Load map data
load('practicemap2025update.mat');

% Map parameters
initLogOdds = 0;
gridCounts = [50, 50];
mapLimits = [-3.5, 3.5, -2.5, 2.5];  % Adjust based on actual map

% Initialize map grid coordinates
nX = gridCounts(1);
nY = gridCounts(2);
xMin = mapLimits(1); xMax = mapLimits(2);
yMin = mapLimits(3); yMax = mapLimits(4);
xVec = linspace(xMin, xMax, nX);
yVec = linspace(yMin, yMax, nY);

% Check if required data is available
if ~isfield(dataStore, 'truthPose') || isempty(dataStore.truthPose) || ...
   ~isfield(dataStore, 'rsdepth') || isempty(dataStore.rsdepth)
    error('Required data not found in dataStore.');
end

% Extract robot pose and depth data
robotPose = [dataStore.truthPose(:,2), dataStore.truthPose(:,3), dataStore.truthPose(:,4)];
depthData = dataStore.rsdepth(:,3:11);  % Depth sensor data

% Get the number of rows for each dataset
robotPoseRows = size(robotPose, 1);
depthDataRows = size(depthData, 1);

% Determine which dataset has fewer rows
minRows = min(robotPoseRows, depthDataRows);

% Adjust both datasets to have the same number of rows
robotPose = robotPose(1:minRows, :);
depthData = depthData(1:minRows, :);

fprintf('Building occupancy grid map from %d depth readings...\n', size(depthData, 1));

% Build occupancy grid map from depth data
logOddsGrid = logOddsDepth(robotPose, depthData, initLogOdds, gridCounts, mapLimits);

% Convert to probability grid
probGrid = 1 ./ (1 + exp(-logOddsGrid));

% Create binary grid (threshold at 0.5)
binaryGrid = probGrid >= 0.5;

% Create an explored area map (cells that have been observed)
exploredGrid = createExploredAreaMap(robotPose, depthData, gridCounts, mapLimits);

% Detect which optional walls exist with improved tolerance
fprintf('Detecting optional walls considering explored areas...\n');
optWallsStatus = detectOptionalWallsWithExploredAreas(binaryGrid, exploredGrid, optWalls, xVec, yVec);

% Count how many optional walls were detected as existing/non-existing
existingWalls = sum(optWallsStatus == 1);
nonExistingWalls = sum(optWallsStatus == -1);
unknownWalls = sum(optWallsStatus == 0);

fprintf('Detected %d existing walls, %d non-existing walls, %d unknown walls\n', ...
    existingWalls, nonExistingWalls, unknownWalls);

% Create figure
figure('Position', [100, 100, 1000, 800], 'Name', 'Mapping Results');

% Plot binary grid map
subplot(1, 2, 1);
imagesc(xVec, yVec, binaryGrid);
colormap(flipud(gray));
set(gca, 'YDir', 'normal');
axis equal;
title('Binary Occupancy Map (from Depth Data)');
xlabel('X (m)');
ylabel('Y (m)');
hold on;

% Plot robot trajectory
plot(robotPose(:, 1), robotPose(:, 2), 'g-', 'LineWidth', 2);
plot(robotPose(1, 1), robotPose(1, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');  % Start
plot(robotPose(end, 1), robotPose(end, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % End

% Draw fixed walls (black)
for i = 1:size(map, 1)
    line([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'Color', 'k', 'LineWidth', 2);
end

% Draw optional walls
for i = 1:size(optWalls, 1)
    if optWallsStatus(i) == 1  % Exists
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'b', 'LineWidth', 2);
    elseif optWallsStatus(i) == -1  % Does not exist
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    else  % Unknown
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', [0.5, 0.5, 0], 'LineStyle', ':', 'LineWidth', 2);
    end
end

% Add legend for first subplot
legend_handles = [];
legend_text = {};

h = plot(NaN, NaN, 'k-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Fixed Walls';

h = plot(NaN, NaN, 'b-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Existing Optional Walls';

h = plot(NaN, NaN, 'r--', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Non-existing Optional Walls';

h = plot(NaN, NaN, ':', 'Color', [0.5, 0.5, 0], 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Unknown Optional Walls';

h = plot(NaN, NaN, 'g-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Robot Trajectory';

h = plot(NaN, NaN, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Start Position';

h = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
legend_handles = [legend_handles, h];
legend_text{end+1} = 'End Position';

legend(legend_handles, legend_text, 'Location', 'SouthEast');

% Plot explored area map
subplot(1, 2, 2);
imagesc(xVec, yVec, exploredGrid);
colormap(flipud(gray));
set(gca, 'YDir', 'normal');
axis equal;
colorbar;
title('Explored Area Map');
xlabel('X (m)');
ylabel('Y (m)');
hold on;

% Plot robot trajectory
plot(robotPose(:, 1), robotPose(:, 2), 'g-', 'LineWidth', 2);
plot(robotPose(1, 1), robotPose(1, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');  % Start
plot(robotPose(end, 1), robotPose(end, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % End

% Draw fixed walls (black)
for i = 1:size(map, 1)
    line([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'Color', 'k', 'LineWidth', 2);
end

% Draw optional walls with the same colors
for i = 1:size(optWalls, 1)
    if optWallsStatus(i) == 1  % Exists
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'b', 'LineWidth', 2);
    elseif optWallsStatus(i) == -1  % Does not exist
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    else  % Unknown
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', [0.5, 0.5, 0], 'LineStyle', ':', 'LineWidth', 2);
    end
end

% Create a third figure for the final map
figure('Position', [100, 100, 800, 600], 'Name', 'Final Map with Detected Walls');
hold on;

% Draw fixed walls (black)
for i = 1:size(map, 1)
    line([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'Color', 'k', 'LineWidth', 2);
end

% Draw optional walls that exist (blue)
for i = 1:size(optWalls, 1)
    if optWallsStatus(i) == 1  % Exists
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'b', 'LineWidth', 2);
    elseif optWallsStatus(i) == -1  % Does not exist
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
    else  % Unknown (dotted brown/olive line)
        line([optWalls(i, 1), optWalls(i, 3)], [optWalls(i, 2), optWalls(i, 4)], ...
            'Color', [0.5, 0.5, 0], 'LineStyle', ':', 'LineWidth', 2);
    end
end

% Plot robot trajectory
plot(robotPose(:, 1), robotPose(:, 2), 'g-', 'LineWidth', 2);
plot(robotPose(1, 1), robotPose(1, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');  % Start
plot(robotPose(end, 1), robotPose(end, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % End

% Add waypoints and EC waypoints if available
if exist('waypoints', 'var')
    for i = 1:size(waypoints, 1)
        plot(waypoints(i, 1), waypoints(i, 2), 'bo', 'MarkerSize', 10);
        text(waypoints(i, 1) + 0.1, waypoints(i, 2) + 0.1, sprintf('W%d', i), 'Color', 'b');
    end
end

if exist('ECwaypoints', 'var')
    for i = 1:size(ECwaypoints, 1)
        plot(ECwaypoints(i, 1), ECwaypoints(i, 2), 'mo', 'MarkerSize', 10);
        text(ECwaypoints(i, 1) + 0.1, ECwaypoints(i, 2) + 0.1, sprintf('EC%d', i), 'Color', 'm');
    end
end

title('Final Map with Detected Walls and Robot Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
grid on;

% Add legend
legend_handles = [];
legend_text = {};

h = plot(NaN, NaN, 'k-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Fixed Walls';

h = plot(NaN, NaN, 'b-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Existing Optional Walls';

h = plot(NaN, NaN, 'r--', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Non-existing Optional Walls';

h = plot(NaN, NaN, ':', 'Color', [0.5, 0.5, 0], 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Unknown Optional Walls';

h = plot(NaN, NaN, 'g-', 'LineWidth', 2);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Robot Trajectory';

h = plot(NaN, NaN, 'bo', 'MarkerSize', 10);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'Waypoints';

h = plot(NaN, NaN, 'mo', 'MarkerSize', 10);
legend_handles = [legend_handles, h];
legend_text{end+1} = 'EC Waypoints';

legend(legend_handles, legend_text, 'Location', 'SouthEast');

% Set axis limits
axis([xMin xMax yMin yMax]);

fprintf('Visualization complete.\n');

% ---------------------------------------------------------
% - New content added, Chengle, you can check it out here -
% - New content added, Chengle, you can check it out here -
% - New content added, Chengle, you can check it out here -
% ---------------------------------------------------------
% Update：Update the map file
% Create the updated map file
fprintf('Updating map with detected walls...\n');

% Create new map data and copy the original data
updatedMap = map;

% Calculate the number of optional walls that need to be added to the map
wallsToAdd = sum(optWallsStatus == 1);
fprintf('Adding %d confirmed walls to the map...\n', wallsToAdd);

% Add the confirmed existing optional walls to the updated map
for i = 1:size(optWalls, 1)
    if optWallsStatus(i) == 1  % If the wall exists
        % Add this wall to the updated map
        updatedMap(end+1, :) = optWalls(i, :);
    end
end

% Create updated optWalls by only keeping unknown status walls
fprintf('Updating optional walls list...\n');
updatedOptWallsIndices = find(optWallsStatus == 0);  % Get indices of walls with unknown status
updatedOptWalls = optWalls(updatedOptWallsIndices, :);  % Keep only those walls

% Count how many optional walls were removed
wallsRemoved = sum(optWallsStatus ~= 0);
fprintf('Removed %d walls from optional walls list (%d existing, %d non-existing).\n', ...
    wallsRemoved, sum(optWallsStatus == 1), sum(optWallsStatus == -1));

% Save the updated map
if exist('waypoints', 'var')
    updatedWaypoints = waypoints;
end
if exist('ECwaypoints', 'var')
    updatedECwaypoints = ECwaypoints;
end

% Save as a new file
if exist('waypoints', 'var') && exist('ECwaypoints', 'var')
    save('updatepracticemap2025update.mat', 'updatedMap', 'updatedOptWalls', 'updatedWaypoints', 'updatedECwaypoints');
elseif exist('waypoints', 'var')
    save('updatepracticemap2025update.mat', 'updatedMap', 'updatedOptWalls', 'updatedWaypoints');
else
    save('updatepracticemap2025update.mat', 'updatedMap', 'updatedOptWalls');
end

fprintf('Map updated and saved as "updatepracticemap2025update.mat".\n');
end

% --------------------- Helper Functions ----------------------
function exploredGrid = createExploredAreaMap(robotPose, depth, gridCounts, boundary)
% Creates a binary map showing which areas have been explored by the robot's sensors
% 1 = Explored, 0 = Unexplored

    [nY, nX] = deal(gridCounts(2), gridCounts(1));
    exploredGrid = zeros(nY, nX);
    
    % Calculate cell dimensions
    cell_width = (boundary(2) - boundary(1)) / nX;
    cell_height = (boundary(4) - boundary(3)) / nY;
    
    % Calculate cell center coordinates
    cellMid_x = linspace(boundary(1), boundary(2), nX+1);
    cellMid_x = cellMid_x(1:end-1) + cell_width/2;
    cellMid_y = linspace(boundary(3), boundary(4), nY+1);
    cellMid_y = cellMid_y(1:end-1) + cell_height/2;
    
    % Define beam angles (9 beams from RealSense depth sensor)
    angles = linspace(27*pi/180, -27*pi/180, 9);
    
    % Minimum step size for ray tracing
    increment_diag = min(cell_width, cell_height);
    
    % Process each pose and depth reading
    for t = 1:size(robotPose, 1)
        % Current robot pose
        pose = robotPose(t, :);
        
        % Process each depth beam
        for i = 1:size(depth, 2)
            if depth(t, i) ~= 0  % Valid depth reading
                % Mark cells along the beam path as explored
                dist = depth(t, i);
                
                % Ray-tracing from robot to obstacle
                x = 0; y = 0; d = 0;
                
                while d <= dist
                    % Calculate global coordinates
                    pt = robot2global(pose, [0.13 + x, y + d*tan(angles(i))]);
                    
                    % Find corresponding grid cell
                    [~, ix] = min(abs(cellMid_x - pt(1)));
                    [~, iy] = min(abs(cellMid_y - pt(2)));
                    
                    % Mark cell as explored
                    exploredGrid(iy, ix) = 1;
                    
                    % Increment along beam
                    d = d + increment_diag;
                end
            end
        end
    end
end

function optWallsStatus = detectOptionalWallsWithExploredAreas(binaryGrid, exploredGrid, optWalls, xVec, yVec)
% Detect which optional walls exist in the environment,
% accounting for explored vs unexplored areas
    
    % Initialize wall status array
    optWallsStatus = zeros(size(optWalls, 1), 1);
    
    % Number of sampling points on wall
    numSamples = 10;
    
    % Tolerance parameters
    searchRadius = 1;  % Number of grid cells to search in each direction
    existenceThreshold = 0.6;  % Percentage of wall that needs to be detected to consider it existing
    nonExistenceThreshold = 0.6;  % Percentage of wall that needs to be empty to consider it non-existing
    minExploredPercentage = 0.3;  % Minimum percentage of wall that must be explored to make a determination
    
    % Size of the grid
    [gridRows, gridCols] = size(binaryGrid);
    
    % Check each optional wall
    for i = 1:size(optWalls, 1)
        % Get wall endpoints
        x1 = optWalls(i, 1);
        y1 = optWalls(i, 2);
        x2 = optWalls(i, 3);
        y2 = optWalls(i, 4);
        
        % Sample points along wall
        xSamples = linspace(x1, x2, numSamples);
        ySamples = linspace(y1, y2, numSamples);
        
        % Initialize counters
        wallDetectionCount = 0;
        exploredCount = 0;
        
        % For each sample point along the wall
        for j = 1:numSamples
            % Find grid indices for this sample point
            [~, xIdx] = min(abs(xVec - xSamples(j)));
            [~, yIdx] = min(abs(yVec - ySamples(j)));
            
            % Check if area around this point has been explored
            isExplored = false;
            
            % Search in a square neighborhood for explored cells
            for dy = -searchRadius:searchRadius
                for dx = -searchRadius:searchRadius
                    % Calculate neighboring cell indices
                    neighborY = yIdx + dy;
                    neighborX = xIdx + dx;
                    
                    % Check if indices are within grid bounds
                    if neighborY >= 1 && neighborY <= gridRows && ...
                       neighborX >= 1 && neighborX <= gridCols
                        % If cell is explored
                        if exploredGrid(neighborY, neighborX) == 1
                            isExplored = true;
                            break;
                        end
                    end
                end
                if isExplored
                    break;
                end
            end
            
            % If this part of the wall has been explored
            if isExplored
                exploredCount = exploredCount + 1;
                
                % Check if any point within the search radius is occupied
                foundWall = false;
                
                % Search in a square neighborhood around the sample point
                for dy = -searchRadius:searchRadius
                    for dx = -searchRadius:searchRadius
                        % Calculate neighboring cell indices
                        neighborY = yIdx + dy;
                        neighborX = xIdx + dx;
                        
                        % Check if indices are within grid bounds
                        if neighborY >= 1 && neighborY <= gridRows && ...
                           neighborX >= 1 && neighborX <= gridCols
                            % If cell is occupied (black in binary grid)
                            if binaryGrid(neighborY, neighborX) == 1
                                foundWall = true;
                                break;
                            end
                        end
                    end
                    if foundWall
                        break;
                    end
                end
                
                % Increment count if wall is detected at this sample point
                if foundWall
                    wallDetectionCount = wallDetectionCount + 1;
                end
            end
        end
        
        % Calculate percentage of explored wall
        exploredPercentage = exploredCount / numSamples;
        
        % If enough of the wall has been explored, make a determination
        if exploredPercentage >= minExploredPercentage
            % Calculate percentage of wall detected within explored areas
            if exploredCount > 0
                wallDetectionPercentage = wallDetectionCount / exploredCount;
            else
                wallDetectionPercentage = 0;
            end
            
            % Determine wall status based on detection percentage
            if wallDetectionPercentage >= existenceThreshold
                optWallsStatus(i) = 1;  % Wall exists
            elseif wallDetectionPercentage <= nonExistenceThreshold
                optWallsStatus(i) = -1;  % Wall does not exist
            else
                optWallsStatus(i) = 0;  % Unknown status (ambiguous)
            end
        else
            % Not enough of the wall has been explored
            optWallsStatus(i) = 0;  % Unknown status
        end
    end
end

function logOddsGrid = logOddsDepth(robotPose, depth, L0, numCells, boundary)
% Build occupancy grid map based on depth sensor data
    time_length = size(robotPose,1);
    cell_width = (boundary(2) - boundary(1)) / numCells(1);
    cell_height = (boundary(4) - boundary(3)) / numCells(2);
    cellMid_x = linspace(boundary(1),boundary(2),numCells(1)+1);
    cellMid_x = cellMid_x(1:end-1) + cell_width/2;
    cellMid_y = linspace(boundary(3),boundary(4),numCells(2)+1);
    cellMid_y = cellMid_y(1:end-1) + cell_height/2;
    
    logOddsGrid = L0 * ones(numCells(2), numCells(1));
    angles = linspace(27*pi/180,-27*pi/180, 9);
    increment_diag = min(cell_width, cell_height);
    
    for t = 1:time_length
        % Find nearest grid index
        [~,ix] = min(abs(cellMid_x - robotPose(t,1)));
        [~,iy] = min(abs(cellMid_y - robotPose(t,2)));
        
        % Mark robot position as free
        logOddsGrid(iy,ix) = logOddsGrid(iy,ix) - 0.368 - L0;
        
        % Process depth sensor data
        for i = 1:size(depth,2)
            % When valid depth detected
            if depth(t,i) ~= 0 && depth(t,i) < 10
                % Calculate obstacle global coordinates
                pt = robot2global(robotPose(t,:), [0.13+depth(t,i), depth(t,i)*tan(angles(i))]);
                
                % Find corresponding grid index
                [~,ix] = min(abs(cellMid_x - pt(1)));
                [~,iy] = min(abs(cellMid_y - pt(2)));
                
                % Mark obstacle position as occupied
                logOddsGrid(iy,ix) = logOddsGrid(iy,ix) + 5 - L0;
            end
            
            % Ray tracing, mark path between sensor and obstacle as free
            if depth(t,i) ~= 0
                dx = increment_diag * cos(angles(i));
                dy = increment_diag * sin(angles(i));
                dist = depth(t,i) / cos(angles(i));
                x = 0.13; y = 0; d = increment_diag;
                
                while d < dist
                    x = x + dx; y = y + dy;
                    pt = robot2global(robotPose(t,:), [x y]);
                    [~,ix] = min(abs(cellMid_x - pt(1)));
                    [~,iy] = min(abs(cellMid_y - pt(2)));
                    
                    % Mark path grid as free
                    logOddsGrid(iy,ix) = logOddsGrid(iy,ix) - 0.176 - L0;
                    d = d + increment_diag;
                end
            end
        end
    end
end

function globalPt = robot2global(pose, offset)
% Convert robot local coordinates to global coordinates
    th = pose(3);
    R = [cos(th), -sin(th); sin(th), cos(th)];
    globalOffset = R * offset(:);
    globalPt = [pose(1); pose(2)] + globalOffset;
    globalPt = globalPt(:).';
end