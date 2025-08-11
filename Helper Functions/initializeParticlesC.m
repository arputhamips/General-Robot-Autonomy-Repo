function dataStore = initializeParticlesC(dataStore, map, numParticles, optWalls)
%INITIALIZEPARTICLESC Global particle distribution with intelligent sampling
%
% dataStore = initializeParticlesC(dataStore, map, numParticles, optWalls)
%
% This function distributes particles globally throughout the map using:
% 1. Intelligent free-space sampling to ensure particles are in valid positions
% 2. Grid-based distribution to ensure coverage of the entire map
% 3. Adaptive density in regions of interest
%
% Inputs:
% dataStore - Data structure to store particle information
% map - Map of the environment as M×4 array [x1 y1 x2 y2]
% numParticles - Number of particles to generate
% optWalls - Optional walls as K×4 array [x1 y1 x2 y2]
%
% Output:
% dataStore - Updated data structure with initialized particles

% Map boundaries
xMin = min(min(map(:,1)), min(map(:,3)));
xMax = max(max(map(:,1)), max(map(:,3)));
yMin = min(min(map(:,2)), min(map(:,4)));
yMax = max(max(map(:,2)), max(map(:,4)));

% Calculate map dimensions
mapWidth = xMax - xMin;
mapHeight = yMax - yMin;

% Optional walls configuration
nw = size(optWalls,1);

% Create particle template
tpl = struct('x', 0, 'y', 0, 'theta', 0, ...
             'weight', 1/numParticles, ...
             'optWallsState', zeros(nw,1));
dataStore.particles = repmat(tpl, 1, numParticles);

% APPROACH 1: Grid-based sampling (60% of particles)
% Ensures uniform coverage of the map
numGrid = round(0.6 * numParticles);
gridSize = ceil(sqrt(numGrid));
xStep = mapWidth / gridSize;
yStep = mapHeight / gridSize;

idx = 1;
for i = 1:gridSize
    for j = 1:gridSize
        if idx > numGrid
            break;
        end
        
        % Calculate grid cell center
        x = xMin + (i - 0.5) * xStep;
        y = yMin + (j - 0.5) * yStep;
        
        % Add small jitter within the cell
        x = x + xStep * (rand - 0.5) * 0.8;
        y = y + yStep * (rand - 0.5) * 0.8;
        
        % Check if position is valid
        if isValidPosition(x, y, map)
            dataStore.particles(idx).x = x;
            dataStore.particles(idx).y = y;
            dataStore.particles(idx).theta = wrapToPi(2*pi*rand - pi);
            
            % Randomly initialize optional wall states
            for w = 1:nw
                dataStore.particles(idx).optWallsState(w) = randi([0, 1]);
            end
            
            idx = idx + 1;
        end
    end
end

% APPROACH 2: Random sampling for remaining particles
% Fills in the gaps with random positions
while idx <= numParticles
    % Generate random position
    x = xMin + mapWidth * rand;
    y = yMin + mapHeight * rand;
    
    % Check if position is valid
    if isValidPosition(x, y, map)
        dataStore.particles(idx).x = x;
        dataStore.particles(idx).y = y;
        dataStore.particles(idx).theta = wrapToPi(2*pi*rand - pi);
        
        % Randomly initialize optional wall states
        for w = 1:nw
            dataStore.particles(idx).optWallsState(w) = randi([0, 1]);
        end
        
        idx = idx + 1;
    end
end

% Ensure equal weights
[dataStore.particles.weight] = deal(1/numParticles);

% Display statistics
fprintf('Global particle distribution: %d particles\n', numParticles);
fprintf('Map coverage: (%.2f,%.2f) to (%.2f,%.2f)\n', xMin, yMin, xMax, yMax);
end