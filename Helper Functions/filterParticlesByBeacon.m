function dataStore = filterParticlesByBeacon(dataStore, beaconLoc, map)
%FILTERPARTICLESBYBEACON Filter particles based on beacon measurements
%
% dataStore = filterParticlesByBeacon(dataStore, beaconLoc)
%
% Inputs:
%   dataStore - Data structure with particles and beacon measurements
%   beaconLoc - Beacon locations in the environment
%
% Outputs:
%   dataStore - Updated data structure with reweighted particles

% Check if we have beacon data
if isempty(dataStore.beacon) || size(dataStore.beacon, 1) < 1
    fprintf('No beacon data available for filtering\n');
    return;
end

% Get the latest beacon measurement
latestBeacon = dataStore.beacon(end, :);

% Extract beacon information
beaconId = latestBeacon(2);
dPerp = latestBeacon(3);  % Forward distance (m)
offs = latestBeacon(4);   % Lateral offset (m)

% Find this beacon in the map
beaconIdx = find(beaconLoc(:,1) == beaconId, 1);

if isempty(beaconIdx)
    fprintf('Unknown beacon ID: %d\n', beaconId);
    return;
end

% Get beacon position in global frame
beaconX = beaconLoc(beaconIdx, 2);
beaconY = beaconLoc(beaconIdx, 3);

% Convert measurement to range and bearing
measRange = hypot(dPerp, offs);
measBearing = atan2(offs, dPerp);

% Noise parameters for Gaussian evaluation
sigmaRange = 0.20;     % Range error std dev (m)
sigmaBearing = 5*pi/180;  % Bearing error std dev (rad)

% Update weights for each particle
weights = zeros(1, length(dataStore.particles));

for i = 1:length(dataStore.particles)
    % Get particle pose
    px = dataStore.particles(i).x;
    py = dataStore.particles(i).y;
    ptheta = dataStore.particles(i).theta;
    
    % Calculate expected range and bearing to beacon
    dx = beaconX - px;
    dy = beaconY - py;
    expRange = hypot(dx, dy);
    expBearing = wrapToPi(atan2(dy, dx) - ptheta);
    
    % Calculate error
    rangeErr = measRange - expRange;
    bearingErr = wrapToPi(measBearing - expBearing);
    
    % Gaussian likelihood
    rangeProb = exp(-0.5 * (rangeErr/sigmaRange)^2) / (sqrt(2*pi) * sigmaRange);
    bearingProb = exp(-0.5 * (bearingErr/sigmaBearing)^2) / (sqrt(2*pi) * sigmaBearing);
    
    % Combined likelihood
    weights(i) = rangeProb * bearingProb;
end

% Update particle weights
for i = 1:length(dataStore.particles)
    dataStore.particles(i).weight = dataStore.particles(i).weight * weights(i);
end

% Normalize weights
totalWeight = sum([dataStore.particles.weight]);
if totalWeight > 0
    for i = 1:length(dataStore.particles)
        dataStore.particles(i).weight = dataStore.particles(i).weight / totalWeight;
    end
else
    % If all weights are zero, reinitialize with uniform weights
    for i = 1:length(dataStore.particles)
        dataStore.particles(i).weight = 1 / length(dataStore.particles);
    end
end

% Resample if effective particle number is too low
Neff = 1 / sum([dataStore.particles.weight].^2);
if Neff < length(dataStore.particles) / 2
    dataStore = resampleParticles(dataStore, map, 0.05);
end

fprintf('Particles filtered by beacon measurements (ID: %d)\n', beaconId);
end