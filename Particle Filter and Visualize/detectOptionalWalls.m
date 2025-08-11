function detectedWalls = detectOptionalWalls(particles, rsDepths, optWalls, sensorOrigin, angles)
    % Get best particle
    [~, bestIdx] = max([particles.weight]);
    bestPose = particles(bestIdx).pose;
    
    % Check each optional wall
    numOptWalls = size(optWalls, 1);
    detectedWalls = false(numOptWalls, 1);
    
    for i = 1:numOptWalls
        wall = optWalls(i,:);
        
        % Calculate expected readings with and without this wall
        mapWithWall = [optWalls(i,:)];
        depthsWithWall = depthPredict(bestPose', mapWithWall, sensorOrigin, angles);
        depthsWithoutWall = inf(size(depthsWithWall));
        
        % Count evidence for wall
        evidence = 0;
        for k = 1:length(angles)
            if ~isnan(rsDepths(k)) && rsDepths(k) < 2.5
                % If actual reading is closer to prediction with wall
                errorWith = abs(rsDepths(k) - depthsWithWall(k));
                errorWithout = abs(rsDepths(k) - depthsWithoutWall(k));
                
                if errorWith < errorWithout && errorWith < 0.1
                    evidence = evidence + 1;
                end
            end
        end
        
        if evidence >= 3  % Threshold for detection
            detectedWalls(i) = true;
        end
    end
end