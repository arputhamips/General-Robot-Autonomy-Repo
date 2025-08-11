function particleFilter(map, optWalls, beaconLoc, waypoints, currentTime)
    % PARTICLEFILTER
    % Run one update step of particle filter and update dataStore.truthPose
    % Incorporates improved sensor models, resampling, and particle diversity
    global dataStore;
    
    % Parameters
    N = 10000;                         % Number of particles
    motionNoise = [0.02, 0.05];      % [transNoise, rotNoise]
    sensorOrigin = [0, 0.08];        % Location of depth sensor on robot
    angles = linspace(-0.471239, 0.471239, 9); % Depth sensor angles
    resampleThresh = 0.6 * N;        % Resampling threshold
    maxRange = 4;                  % Maximum sensor range
    
    % Initialize particles if needed
    if isempty(dataStore.particles)
        dataStore.particles = initializeParticles(N, waypoints);
        if ~isfield(dataStore, 'detectedWalls')
            dataStore.detectedWalls = false(size(optWalls,1), 1);
        end
    end
    
    % Skip if no odometry yet
    if isempty(dataStore.odometry) || size(dataStore.odometry, 1) < 1
        return;
    end
    
    particles = dataStore.particles;
    t = size(dataStore.odometry, 1);
    
    % MOTION UPDATE
    deltaD = dataStore.odometry(t, 2);
    deltaA = dataStore.odometry(t, 3);
    particles = motionUpdate(particles, deltaD, deltaA, motionNoise);
    
    % DEPTH UPDATE
    if ~isempty(dataStore.rsdepth) && ...
       size(dataStore.rsdepth, 1) >= t && ...
       size(dataStore.rsdepth, 2) >= 3 && ...
       ~all(isnan(dataStore.rsdepth(t, 3:end)))
        
        rsDepths = dataStore.rsdepth(t, 3:end);
        
        % Use robust depth update
        particles = robustDepthUpdate(particles, rsDepths, map, ...
            optWalls(dataStore.detectedWalls,:), sensorOrigin, angles);
        
        % Detect optional walls
        if any(~dataStore.detectedWalls)
            newDetections = detectOptionalWalls(particles, rsDepths, ...
                optWalls(~dataStore.detectedWalls,:), sensorOrigin, angles);
            dataStore.detectedWalls(~dataStore.detectedWalls) = newDetections;
        end
    end
    
    % BEACON UPDATE
    if ~isempty(dataStore.beacon) && size(dataStore.beacon, 1) > 0
        % Just get the most recent beacon reading
        latestBeacon = dataStore.beacon(end,:);
        
        % Only update if we have the necessary data
        if size(latestBeacon, 2) >= 5
            particles = beaconPositionUpdate(particles, latestBeacon, beaconLoc);
        end
    end
    
    % Add random particles if needed to maintain diversity
    particles = addRandomParticles(particles, waypoints);
    
    % NORMALIZE WEIGHTS
    weights = [particles.weight];
    sumWeights = sum(weights) + 1e-10;  % Avoid division by zero
    
    for i = 1:N
        particles(i).weight = weights(i) / sumWeights;
    end
    
    % RESAMPLE if effective sample size is too small
    Neff = 1 / sum(weights.^2 + 1e-10);
    if Neff < resampleThresh
        fprintf('Resampling particles (Neff = %.2f)\n', Neff);
        particles = improvedLowVarianceResample(particles);
    end
    
    % ESTIMATE AND STORE POSE
    [~, idx] = max([particles.weight]);
    bestPose = particles(idx).pose;
    dataStore.truthPose = [dataStore.truthPose; currentTime, bestPose];
    dataStore.particles = particles;
    
    % Print occasional debug info
    if mod(t, 10) == 0
        fprintf('Step %d: Max weight = %.6f, Effective N = %.2f\n', ...
            t, max(weights), Neff);
    end
end
