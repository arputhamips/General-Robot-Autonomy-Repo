% =========================== initializeParticles.m ===========================
function dataStore = initializeParticles(dataStore, map, numParticles, optWalls, waypoints)
% 90 % of particles within 0.2 m of random waypoints, 10 % uniform free space

    if nargin < 5 || isempty(waypoints)
        error('initializeParticles: waypoints array required.');
    end
    if nargin < 4, optWalls = []; end

    % map bounds
    xMin = min(min(map(:,1)), min(map(:,3)));
    xMax = max(max(map(:,1)), max(map(:,3)));
    yMin = min(min(map(:,2)), min(map(:,4)));
    yMax = max(max(map(:,2)), max(map(:,4)));

    nw = size(optWalls,1);
    tpl = struct('x',0,'y',0,'theta',0,...
                 'weight',1/numParticles,...
                 'optWallsState',zeros(nw,1));
    dataStore.particles = repmat(tpl,1,numParticles);

    % -------- 95 % around waypoints ---------------------------------------
    numWP = round(0.95 * numParticles);
    for i = 1:numWP
        wp  = waypoints(randi(size(waypoints,1)),:);  % choose waypoint
        r   = 0.2 * sqrt(rand);                       % area‑uniform radius
        ang = 2*pi*rand;
        dataStore.particles(i).x     = wp(1) + r*cos(ang);
        dataStore.particles(i).y     = wp(2) + r*sin(ang);
        dataStore.particles(i).theta = wrapToPi(2*pi*rand - pi);
    end

    % -------- 5 % uniform free‑space particles ---------------------------
    idx = numWP + 1;
    while idx <= numParticles
        rx = xMin + (xMax - xMin) * rand;
        ry = yMin + (yMax - yMin) * rand;
        if isValidPosition(rx, ry, map)
            dataStore.particles(idx).x     = rx;
            dataStore.particles(idx).y     = ry;
            dataStore.particles(idx).theta = wrapToPi(2*pi*rand - pi);
            idx = idx + 1;
        end
    end

    % ensure equal weights
    [dataStore.particles.weight] = deal(1 / numParticles);
end