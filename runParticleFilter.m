function runParticleFilter(map, beaconLoc, optWalls, waypoints, toc)
    global dataStore;

    % Parameters
    N = 200;
    resampleThresh = 0.5 * N;
    motionNoise = [0.02, 0.05];
    depthHit = 5;
    depthMiss = -0.35;
    depthMaxRange = 2.5;

    % Initialize
    particles = initializeParticles(N, waypoints);
    dataStore.truthPose = zeros(size(dataStore.odometry,1),4);

    for t = 2:size(dataStore.odometry,1)
        deltaD = dataStore.odometry(t,2);
        deltaA = dataStore.odometry(t,3);

        % PREDICT
        particles = motionUpdate(particles, deltaD, deltaA, motionNoise);

        % SENSOR UPDATE - DEPTH
        if ~all(isnan(dataStore.rsdepth(t,3:end)))
            rsDepths = dataStore.rsdepth(t,3:end);
            for i = 1:N
                expectedDepths = depthPredict(particles(i).pose', [map; optWalls], [0 0], linspace(-0.471239, 0.471239, 9));
                score = 0;
                for k = 1:length(rsDepths)
                    if ~isnan(rsDepths(k))
                        if abs(expectedDepths(k) - rsDepths(k)) < 0.2
                            score = score + depthHit;
                        else
                            score = score + depthMiss;
                        end
                    end
                end
                particles(i).weight = particles(i).weight * exp(score);
            end
        end


        % % SENSOR UPDATE - BEACON
        % % Find latest beacon reading available before or at odometry time
        % odometryTime = dataStore.odometry(t,1);
        % validBeacons = dataStore.beacon(dataStore.beacon(:,1) <= odometryTime, :);
        % 
        % if ~isempty(validBeacons)
        %     beaconMatch = validBeacons(end,:); % use the most recent available
        % else
        %     beaconMatch = [];
        % end
        % if ~isempty(beaconMatch)
        %     particles = injectBeaconParticles(particles, beaconMatch, beaconLoc, N);
        % end

        % NORMALIZE & RESAMPLE
        weights = [particles.weight];
        weights = weights / sum(weights + 1e-12);
        for i = 1:N, particles(i).weight = weights(i); end

        Neff = 1 / sum(weights.^2);
        if Neff < resampleThresh
            particles = lowVarianceResample(particles);
        end

       % ESTIMATE POSE
        [~, idx] = max([particles.weight]);
        bestPose = particles(idx).pose; % [x y theta]
        dataStore.truthPose(t,:) = [toc, bestPose];

        % % Optional: Add bump handler here
    end
end

function particles = initializeParticles(N, waypoints)
    M = size(waypoints,1);
    particles(N) = struct('pose',[],'weight',[]);
    for i = 1:N
        wp = waypoints(randi(M),:);
        x = wp(1) + 0.2*randn();
        y = wp(2) + 0.2*randn();
        theta = 2*pi*rand();
        particles(i).pose = [x y theta];
        particles(i).weight = 1/N;
    end
end

function particles = motionUpdate(particles, deltaD, deltaA, noise)
    for i = 1:length(particles)
        x = particles(i).pose(1);
        y = particles(i).pose(2);
        theta = particles(i).pose(3);

        d = deltaD + noise(1)*randn();
        a = deltaA + noise(2)*randn();

        x = x + d*cos(theta);
        y = y + d*sin(theta);
        theta = wrapToPi(theta + a);

        particles(i).pose = [x y theta];
    end
end

function particles = sensorUpdateDepth(particles, rsdepthRow, map, optWalls, hitVal, missVal, maxRange)
    depths = rsdepthRow(3:end);
    angleSpan = linspace(-pi/6, pi/6, length(depths));
    for i = 1:length(particles)
        score = 0;
        for j = 1:length(depths)
            expected = raycastDepth(particles(i).pose, angleSpan(j), map, optWalls, maxRange);
            z = depths(j);
            if ~isnan(z)
                if abs(expected - z) < 0.2
                    score = score + hitVal;
                else
                    score = score + missVal;
                end
            end
        end
        particles(i).weight = particles(i).weight * exp(score);
    end
end

function d = raycastDepth(pose, angle, map, optWalls, maxRange)
    % Simulate depth by casting a ray from the pose at angle and finding first wall intersection
    % Combine map and active optWalls for now
    allWalls = [map; optWalls]; % Optionally filter for active optWalls
    origin = pose(1:2);
    theta = wrapToPi(pose(3) + angle);
    d = maxRange;
    for i = 1:size(allWalls,1)
        wall = allWalls(i,:);
        [intersects, dist] = raySegmentIntersect(origin, theta, wall(1:2), wall(3:4));
        if intersects && dist < d
            d = dist;
        end
    end
end

function [intersects, dist] = raySegmentIntersect(p0, theta, p1, p2)
    % Create a ray starting from p0 along direction theta
    rayLength = 10; % Large enough for depth range
    p3 = p0 + rayLength * [cos(theta) sin(theta)]; 

    % Ray points
    xRay = [p0(1) p3(1)];
    yRay = [p0(2) p3(2)];

    % Wall segment points
    xWall = [p1(1) p2(1)];
    yWall = [p1(2) p2(2)];

    % Use polyxpoly to check intersection
    [xi, yi] = polyxpoly(xRay, yRay, xWall, yWall);

    if ~isempty(xi)
        intersects = true;
        dist = norm([xi(1) yi(1)] - p0);
    else
        intersects = false;
        dist = inf;
    end
end

function particles = injectBeaconParticles(particles, beaconRow, beaconLoc, N)
    id = beaconRow(3);
    beaconPose = beaconLoc(beaconLoc(:,1)==id, 2:3);
    if isempty(beaconPose), return; end
    numNew = 20;
    for i = 1:numNew
        x = beaconPose(1) - beaconRow(4) + 0.1*randn();
        y = beaconPose(2) - beaconRow(5) + 0.1*randn();
        theta = 2*pi*rand();
        j = randi(N);
        particles(j).pose = [x y theta];
        particles(j).weight = 1/N;
    end
end

function particles = lowVarianceResample(particles)
    N = length(particles);
    weights = [particles.weight];
    weights = weights / sum(weights);
    cdf = cumsum(weights);
    r = rand()/N;
    newParticles = particles;
    i = 1;
    for m = 1:N
        U = r + (m-1)/N;
        while U > cdf(i)
            i = i + 1;
        end
        newParticles(m).pose = particles(i).pose;
        newParticles(m).weight = 1/N;
    end
    particles = newParticles;
end