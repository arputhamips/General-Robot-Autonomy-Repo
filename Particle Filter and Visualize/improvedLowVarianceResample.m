function particles = improvedLowVarianceResample(particles)
    N = length(particles);
    weights = [particles.weight];
    
    % Handle potential numerical issues
    weights = weights + 1e-10;  % Add small epsilon to avoid zero weights
    weights = weights / sum(weights);  % Normalize
    
    % Create CDF
    cdf = cumsum(weights);
    
    % Start at a random position
    r = rand() / N;
    newParticles = particles;  % Pre-allocate
    i = 1;
    
    for m = 1:N
        U = r + (m-1) / N;
        
        % Safety check to avoid infinite loops
        loopCount = 0;
        
        while U > cdf(i) && loopCount < N
            i = i + 1;
            loopCount = loopCount + 1;
            
            % Wrap around if we reach the end
            if i > N
                i = 1;
            end
        end
        
        % Copy the selected particle
        newParticles(m).pose = particles(i).pose;
        newParticles(m).weight = 1/N;  % Reset weights
    end
    
    particles = newParticles;
    
    % Print resampling statistics for debugging
    uniqueIndices = length(unique([particles.pose]'));
    fprintf('Resampled: %d unique particles out of %d\n', uniqueIndices, N);
end