% Improved depth sensor model
function particles = depthUpdate(particles, rsDepths, map, optWalls, sensorOrigin, angles)
    N = length(particles);
    maxRange = 2.5; % Max sensor range
    
    for i = 1:N
        expectedDepths = depthPredict(particles(i).pose', [map; optWalls], sensorOrigin, angles);
        likelihood = 1.0;
        
        for k = 1:length(rsDepths)
            if isnan(rsDepths(k))
                continue;
            end
            
            % Handle max range readings
            if rsDepths(k) >= maxRange && expectedDepths(k) >= maxRange
                likelihood = likelihood * 0.9; % High probability for both at max range
            elseif rsDepths(k) >= maxRange || expectedDepths(k) >= maxRange
                likelihood = likelihood * 0.1; % Low probability if only one is max range
            else
                % Gaussian model for expected measurement
                sigma = 0.03 + 0.05 * rsDepths(k); % Error grows with distance
                error = rsDepths(k) - expectedDepths(k);
                likelihood = likelihood * (0.9 * exp(-0.5 * (error/sigma)^2) + 0.1/maxRange);
            end
        end
        
        particles(i).weight = particles(i).weight * likelihood;
    end
end