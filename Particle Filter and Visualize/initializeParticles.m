function particles = initializeParticles(N, waypoints)
    M = size(waypoints,1);
    particles(N) = struct('pose',[],'weight',[]);
    
    % Distribute particles evenly among waypoints
    particlesPerWaypoint = floor(N / M);
    
    particleIdx = 1;
    for i = 1:M
        for j = 1:particlesPerWaypoint
            wp = waypoints(i,:);
            x = wp(1) + 0.1*randn();
            y = wp(2) + 0.1*randn();
            theta = 2*pi*rand();
            
            particles(particleIdx).pose = [x y theta];
            particles(particleIdx).weight = 1/N;
            particleIdx = particleIdx + 1;
        end
    end
    
    % Distribute remaining particles randomly
    for i = particleIdx:N
        wp = waypoints(randi(M),:);
        x = wp(1) + 0.2*randn();
        y = wp(2) + 0.2*randn();
        theta = 2*pi*rand();
        
        particles(i).pose = [x y theta];
        particles(i).weight = 1/N;
    end
end