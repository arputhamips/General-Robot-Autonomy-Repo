function particles = addRandomParticles(particles, waypoints)
    N = length(particles);
    weights = [particles.weight];
    
    % Calculate effective sample size
    Neff = 1 / sum(weights.^2);
    
    % If Neff is too small, inject some random particles
    if Neff < 0.1 * N
        fprintf('Low diversity detected, adding random particles\n');
        numToAdd = floor(0.1 * N);  % Add 10% new particles
        
        for i = 1:numToAdd
            idx = randi(N);  % Replace a random particle
            wp = waypoints(randi(size(waypoints, 1)), :);
            
            particles(idx).pose = [wp(1) + 0.2*randn(), wp(2) + 0.2*randn(), 2*pi*rand()];
            particles(idx).weight = 1/N;
        end
        
        % Renormalize weights
        weights = [particles.weight];
        weights = weights / sum(weights);
        
        for i = 1:N
            particles(i).weight = weights(i);
        end
    end
end