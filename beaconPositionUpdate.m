function particles = beaconPositionUpdate(particles, beaconData, beaconLoc)
    % Skip if not enough data
    if size(beaconData, 2) < 5
        return;
    end
    
    % Extract beacon info
    tagID = beaconData(3);
    observedX = beaconData(4);  % X position in robot frame
    observedY = beaconData(5);  % Y position in robot frame
    
    % Find matching beacon
    beaconIdx = find(beaconLoc(:,1) == tagID);
    
    if isempty(beaconIdx)
        return;  % Beacon not in map
    end
    
    % Global beacon position
    beaconGlobalX = beaconLoc(beaconIdx, 2);
    beaconGlobalY = beaconLoc(beaconIdx, 3);
    
    % Update weights based on position only (no angle)
    for i = 1:length(particles)
        % Get particle pose
        x = particles(i).pose(1);
        y = particles(i).pose(2);
        theta = particles(i).pose(3);
        
        % Calculate expected position of beacon in robot frame
        dx_global = beaconGlobalX - x;
        dy_global = beaconGlobalY - y;
        
        % Rotate to robot coordinates
        cos_theta = cos(theta);
        sin_theta = sin(theta);
        expected_x = cos_theta * dx_global + sin_theta * dy_global;
        expected_y = -sin_theta * dx_global + cos_theta * dy_global;
        
        % Calculate error
        position_error = sqrt((expected_x - observedX)^2 + (expected_y - observedY)^2);
        
        % Update weight using position error only
        sigma = 0.2;  % Position error standard deviation
        likelihood = exp(-0.5 * (position_error/sigma)^2);
        particles(i).weight = particles(i).weight * likelihood;
    end
    
    % Normalize weights to prevent numerical issues
    weights = [particles.weight];
    sumWeights = sum(weights);
    
    if sumWeights > 0
        for i = 1:length(particles)
            particles(i).weight = particles(i).weight / sumWeights;
        end
    end
end