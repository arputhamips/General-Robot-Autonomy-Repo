%% Initialize occupancy grid parameters
gridSize = [50, 50];           % 25x25 grid
mapResolution = 5 / 50;        % (2.5 - (-2.5)) / 25 = 0.2 m per cell
gridOrigin = [-2.5, -2.5];       % World coordinates corresponding to grid cell (1,1)

% Prior occupancy probability p0(occ) = 0.5 => log odds = 0 for all cells
logOddsGrid = zeros(gridSize);

% Define sensor model parameters (example values)
sensorModelParams.bumpHit = 1;      % Increment to log-odds when a bump is detected
sensorModelParams.bumpRange = 0.2;    % Distance from robot center to contact point

% Define time steps (in seconds) at which to plot the grid
plotTimeSteps = [1, 10, 30];

%% Load datastore (assumed to contain truthPose and bump data)
% load('dataStore.mat');  
% dataStore.truthPose: [time, x, y, theta]
% dataStore.bump: [time, BumpRight, BumpLeft, DropRight, DropLeft, DropCaster, BumpFront]

bumpData = dataStore.bump;           
truthPoseData = dataStore.truthPose; 

currentPlotIdx = 1;
numPlots = length(plotTimeSteps);

%% Process bump measurements and update occupancy grid
for i = 1:size(bumpData,1)
    currentBumpTime = bumpData(i,1);
    % Use only BumpRight, BumpLeft, and BumpFront (columns 2,3,7)
    currentBumpSensors = bumpData(i, [2, 3, 7]); 
    
    % Find the closest truthPose entry (in time) for the current bump measurement
    [~, idx] = min(abs(truthPoseData(:,1) - currentBumpTime));
    robotPose = truthPoseData(idx, 2:4);  % [x, y, theta]
    
    % Update the occupancy grid using the bump sensor data
    logOddsGrid = logOddsBump(logOddsGrid, robotPose, currentBumpSensors, ...
                              mapResolution, gridOrigin, sensorModelParams);
    
    % When the current bump time exceeds a selected plot time, display the grid
    if currentBumpTime >= plotTimeSteps(currentPlotIdx)
        plotOccupancyGrid(logOddsGrid, currentBumpTime);
        currentPlotIdx = currentPlotIdx + 1;
        if currentPlotIdx > numPlots
            break;
        end
    end
end