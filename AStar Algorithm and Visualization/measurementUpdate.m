function dataStore = measurementUpdate(dataStore, map, beaconLoc, optWalls)
    % Measurement update using depth and beacon information
    
    % Check if there are depth sensor readings
    if isempty(dataStore.rsdepth) || size(dataStore.rsdepth, 1) < 1
        return;
    end
    
    % Get latest depth and beacon readings
    latestDepth = dataStore.rsdepth(end, 3:end); % Skip time column

    
    % Fetch odometry and beacon times
    tOdo   = dataStore.odometry(end,1);
    tBcn   = [];   % default: no beacon
    if ~isempty(dataStore.beacon)
        tBcn = dataStore.beacon(end,1);
    end
    maxSkew = 0.30;                       % 100 ms tolerance
    beaconFresh = ~isempty(tBcn) && abs(tOdo - tBcn) < maxSkew;
    
    % For each particle, update weights based on measurements
    for i = 1:length(dataStore.particles)
        particle = dataStore.particles(i);
        particleMap = map;
        
        % Add optional walls that this particle believes exist
        for w = 1:size(optWalls, 1)
            if particle.optWallsState(w) == 1
                particleMap = [particleMap; optWalls(w, :)];
            end
        end
        
        % Inflate the map walls for collision checking (4 inches = 0.1016 meters)
        inflatedMap = inflateWalls(particleMap, 0.1);
        
        % Check if particle is in a valid position
        if ~isValidPosition(particle.x, particle.y, inflatedMap)
            dataStore.particles(i).weight = 0;
            continue;
        end
        
        % 1. Update based on depth sensor
        weight_depth = depthMeasurementLikelihood(particle, latestDepth, particleMap);
        
        weight_beacon = 1.0;
        if beaconFresh
            latestBeacons = dataStore.beacon(end, :);
            weight_beacon = beaconMeasurementLikelihood(particle, latestBeacons, beaconLoc);
        end
        
        % 3. Update with optional walls evidence
        [updatedParticle, weight_opt] = updateOptionalWalls(particle, latestDepth, optWalls);
        dataStore.particles(i) = updatedParticle;
        
        % Combine weights
        dataStore.particles(i).weight = dataStore.particles(i).weight * weight_depth * weight_beacon * weight_opt;
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