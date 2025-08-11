function dataStore = resampleParticles(dataStore, map, scatterFrac)
% resampleParticles  Systematic resampling with random‑particle injection.
%   scatterFrac – fraction (0–1) of total particles to spawn uniformly
%                 in free space after resampling (default 0.05).

    if nargin < 3, scatterFrac = 0.0; end

    N       = numel(dataStore.particles);
    weights = [dataStore.particles.weight];
    weights = weights / sum(weights);          % normalise

    % ----------- effective sample size check -----------------------------
    Neff = 1 / sum(weights.^2);
    if Neff > 0.5 * N
        return                                      % skip resampling
    end

    % ----------- systematic resampling (low‑variance) --------------------
    edges   = cumsum(weights);
    start   = rand / N;
    pts     = start + (0:N-1)/N;

    idx = zeros(1,N);
    j   = 1;
    for i = 1:N
        while pts(i) > edges(j), j = j + 1; end
        idx(i) = j;
    end
    newP = dataStore.particles(idx);          % cloned particles

    % ----------- inject uniformly random particles -----------------------
    numRand = round(scatterFrac * N);
    xMin = min(min(map(:,1)), min(map(:,3)));
    xMax = max(max(map(:,1)), max(map(:,3)));
    yMin = min(min(map(:,2)), min(map(:,4)));
    yMax = max(max(map(:,2)), max(map(:,4)));

    k = 1;
    for i = N-numRand+1 : N
        valid = false;
        while ~valid
            rx = xMin + (xMax-xMin)*rand;
            ry = yMin + (yMax-yMin)*rand;
            valid = isValidPosition(rx, ry, map);
        end
        newP(i).x     = rx;
        newP(i).y     = ry;
        newP(i).theta = 2*pi*rand - pi;
        k = k + 1;
    end

    % ----------- reset equal weights -------------------------------------
    for i = 1:N
        newP(i).weight = 1 / N;
    end

    dataStore.particles = newP;
end
