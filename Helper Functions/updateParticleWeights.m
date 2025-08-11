function dataStore = updateParticleWeights(dataStore, map, sensorOrigin, angles, optWalls, beaconLoc)
%UPDATEPARTICLEWEIGHTS Update particle weights based on sensor measurements
%
% dataStore = updateParticleWeights(dataStore, map, sensorOrigin, angles, optWalls, beaconLoc)
%
% Inputs:
%   dataStore - Data structure with particles and sensor readings
%   map - Map of the environment
%   sensorOrigin - Origin of the depth sensor in robot frame
%   angles - Sensor angles
%   optWalls - Optional walls in the environment
%   beaconLoc - Beacon locations in the environment
%
% Outputs:
%   dataStore - Updated data structure with reweighted particles

% Check if we have depth data
if isempty(dataStore.rsdepth) || size(dataStore.rsdepth, 1) < 1
    fprintf('No depth data available for particle weight update\n');
    return;
end

% Get latest depth readings
latestDepth = dataStore.rsdepth(end, 3:end)';

% Update weights for each particle
for i = 1:length(dataStore.particles)
    p = dataStore.particles(i);
    
    % Create pose vector for this particle
    pose = [p.x; p.y; p.theta];
    
    % Calculate expected depth readings for this particle
    predDepth = depthPredict(pose, map, sensorOrigin, angles);
    
    % Calculate error between predicted and actual depth
    depthError = norm(predDepth - latestDepth);
    
    % Calculate depth likelihood (higher for smaller errors)
    depthLikelihood = exp(-0.5 * (depthError/0.05)^2);
    
    % Update weight based on depth measurements
    dataStore.particles(i).weight = dataStore.particles(i).weight * depthLikelihood;
end

% Check if we have beacon data
if ~isempty(dataStore.beacon) && size(dataStore.beacon, 1) >= 1
    % Get the latest beacon measurement
    latestBeacon = dataStore.beacon(end, :);
    beaconId = latestBeacon(2);
    beaconX = latestBeacon(3); % Forward distance
    beaconY = latestBeacon(4); % Lateral offset
    
    % Find this beacon in beacon locations
    beaconIdx = find(beaconLoc(:,1) == beaconId, 1);
    
    if ~isempty(beaconIdx)
        % Get beacon's global position
        beaconGlobalX = beaconLoc(beaconIdx, 2);
        beaconGlobalY = beaconLoc(beaconIdx, 3);
        
        % Measured range and bearing
        measRange = hypot(beaconX, beaconY);
        measBearing = atan2(beaconY, beaconX);
        
        % Update weights based on beacon measurements
        for i = 1:length(dataStore.particles)
            p = dataStore.particles(i);
            
            % Expected range and bearing to beacon
            dx = beaconGlobalX - p.x;
            dy = beaconGlobalY - p.y;
            expRange = hypot(dx, dy);
            expBearing = wrapToPi(atan2(dy, dx) - p.theta);
            
            % Range and bearing errors
            rangeErr = (measRange - expRange);
            bearingErr = wrapToPi(measBearing - expBearing);
            
            % Gaussian likelihoods
            rangeLikelihood = exp(-0.5 * (rangeErr/0.1)^2);
            bearingLikelihood = exp(-0.5 * (bearingErr/(5*pi/180))^2);
            
            % Combined beacon likelihood
            beaconLikelihood = rangeLikelihood * bearingLikelihood;
            
            % Update weight
            dataStore.particles(i).weight = dataStore.particles(i).weight * beaconLikelihood;
        end
    end
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

end