function dataStore = initializeFocusedParticles(dataStore, map, pose, optWalls, numParticles, radius)
%INITIALIZEFOCUSEDPARTICLES Initialize particles focused around a specific pose
%
% dataStore = initializeFocusedParticles(dataStore, map, pose, optWalls, numParticles, radius)
%
% Inputs:
%   dataStore - Data structure for storing particles
%   map - Map of the environment
%   pose - Pose to focus around [x; y; theta]
%   optWalls - Optional walls in the environment
%   numParticles - Number of particles to create
%   radius - Radius around the pose for position sampling
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

% Position and orientation of the focus pose
x0 = pose(1);
y0 = pose(2);
theta0 = pose(3);

% Initialize all particles
for i = 1:numParticles
    % Sample position with decreasing noise based on particle index
    % First 20% get very little noise, rest get increasingly more
    if i <= numParticles * 0.2
        % Low noise - within 20% of radius
        r = radius * 0.2 * sqrt(rand);
    else
        % Regular noise - within full radius
        r = radius * sqrt(rand);
    end
    
    % Random angle
    angle = 2*pi * rand;
    
    % Calculate position
    dataStore.particles(i).x = x0 + r * cos(angle);
    dataStore.particles(i).y = y0 + r * sin(angle);
    
    % Sample orientation with decreasing noise
    if i <= numParticles * 0.2
        % Low noise - within 15 degrees
        dataStore.particles(i).theta = wrapToPi(theta0 + 15*pi/180 * (2*rand-1));
    else
        % Full noise - within 45 degrees
        dataStore.particles(i).theta = wrapToPi(theta0 + 45*pi/180 * (2*rand-1));
    end
    
    % Randomize optional wall states
    for w = 1:nw
        dataStore.particles(i).optWallsState(w) = randi([0, 1]);
    end
end

% Make sure weights are uniform
[dataStore.particles.weight] = deal(1/numParticles);

fprintf('Initialized %d focused particles around [%.2f, %.2f, %.1f°]\n', ...
        numParticles, x0, y0, theta0*180/pi);
end