function particles = beaconUpdate(particles, beaconRow, beaconLoc)
    % Get beacon data
    tagID = beaconRow(3);  % ArUco tag ID
    observedX = beaconRow(4);  % x position in robot frame
    observedY = beaconRow(5);  % y position in robot frame
    observedAngle = beaconRow(6);  % angle in robot frame
    
    % Find matching beacon in beaconLoc
    beaconIdx = find(beaconLoc(:,1) == tagID);
    
    if isempty(beaconIdx)
        return;  % No matching beacon found
    end
    
    % Get the global position of the beacon
    beaconGlobalX = beaconLoc(beaconIdx, 2);
    beaconGlobalY = beaconLoc(beaconIdx, 3);
    
    for i = 1:length(particles)
        % Current particle pose
        x = particles(i).pose(1);
        y = particles(i).pose(2);
        theta = particles(i).pose(3);
        
        % Expected relative position of beacon in global frame
        expectedRelX_global = beaconGlobalX - x;
        expectedRelY_global = beaconGlobalY - y;
        
        % Rotate to robot frame
        cosTheta = cos(theta);
        sinTheta = sin(theta);
        expectedRelX = cosTheta * expectedRelX_global + sinTheta * expectedRelY_global;
        expectedRelY = -sinTheta * expectedRelX_global + cosTheta * expectedRelY_global;
        
        % Calculate difference between expected and observed
        distDiff = sqrt((expectedRelX - observedX)^2 + (expectedRelY - observedY)^2);
        
        % Calculate angle difference
        expectedAngle = atan2(expectedRelY, expectedRelX);
        angleDiff = abs(wrapToPi(expectedAngle - observedAngle));
        
        % Update weight based on both position and angle errors
        distLikelihood = exp(-0.5 * (distDiff/0.2)^2);
        angleLikelihood = exp(-0.5 * (angleDiff/0.1)^2);
        
        particles(i).weight = particles(i).weight * distLikelihood * angleLikelihood;
    end
end