function particles = robustDepthUpdate(particles, rsDepths, map, optWalls, sensorOrigin, angles)
    N = length(particles);
    maxRange = 2.5;  % Max sensor range
    
    for i = 1:N
        expectedDepths = depthPredict(particles(i).pose', [map; optWalls], sensorOrigin, angles);
        likelihood = 1.0;
        
        for k = 1:length(rsDepths)
            if isnan(rsDepths(k))
                continue;  % Skip NaN readings
            end
            
            % Calculate measurement likelihood
            measuredDepth = rsDepths(k);
            predictedDepth = expectedDepths(k);
            
            % Handle max range readings specially
            if measuredDepth >= maxRange && predictedDepth >= maxRange
                likelihood = likelihood * 1.0;  % Both predict no obstacle
            elseif measuredDepth >= maxRange || predictedDepth >= maxRange
                likelihood = likelihood * 0.1;  % Disagree on obstacle presence
            else
                % For regular readings, use Gaussian model with increasing variance by distance
                sigma = 0.05 + 0.03 * measuredDepth;  % Error grows with distance
                error = abs(measuredDepth - predictedDepth);
                
                % Mixture model: Gaussian for correct matches, uniform for outliers
                pointLikelihood = 0.9 * exp(-0.5 * (error/sigma)^2) + 0.1/maxRange;
                likelihood = likelihood * pointLikelihood;
            end
        end
        
        % Update weight, avoid numerical issues
        particles(i).weight = particles(i).weight * likelihood + 1e-10;
    end
    
    % Normalize weights
    weights = [particles.weight];
    sumWeights = sum(weights);
    
    if sumWeights > 0
        for i = 1:N
            particles(i).weight = particles(i).weight / sumWeights;
        end
    else
        % If all weights are essentially zero, reset to uniform
        for i = 1:N
            particles(i).weight = 1/N;
        end
    end
end