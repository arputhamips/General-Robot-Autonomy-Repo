function dataStore = initializeWaypointParticles(dataStore, map, waypoint, numParticles, optWalls)
%INITIALIZEWAYPOINTPARTICLES Initialize particles at a single waypoint with all orientations
%
% dataStore = initializeWaypointParticles(dataStore, map, waypoint, numParticles, optWalls)
%
% Inputs:
%   dataStore - Data structure for storing particles
%   map - Map of the environment
%   waypoint - Single waypoint position [x, y]
%   numParticles - Number of particles (orientations)
%   optWalls - Optional walls in the environment
%
% Outputs:
%   dataStore - Updated data structure with initialized particles

% Number of optional walls
nw = size(optWalls, 1);

% Create template particle
tpl = struct('x', 0, 'y', 0, 'theta', 0, ...
           'weight', 1/numParticles, ...
           'optWallsState', zeros(nw, 1));
       
% Initialize particles array
dataStore.particles = repmat(tpl, 1, numParticles);

% Get waypoint position
wpX = waypoint(1);
wpY = waypoint(2);

% Angle step for uniform orientation coverage
angleStep = 2*pi / numParticles;

% Create particles at the waypoint with different orientations
for i = 1:numParticles
    % Calculate orientation uniformly around the circle
    theta = (i-1) * angleStep;
    
    % Set particle properties
    dataStore.particles(i).x = wpX;
    dataStore.particles(i).y = wpY;
    dataStore.particles(i).theta = theta;
    
    % Randomize optional wall states
    for w = 1:nw
        dataStore.particles(i).optWallsState(w) = randi([0, 1]);
    end
end

% Make sure weights are uniform
[dataStore.particles.weight] = deal(1/numParticles);

fprintf('Initialized %d particles at waypoint [%.2f, %.2f]\n', ...
        numParticles, wpX, wpY);
end