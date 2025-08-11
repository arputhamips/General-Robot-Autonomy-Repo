function dataStore = resampleParticles(dataStore, map, scatterFrac)
%RESAMPLEPARTICLES  Systematic resampling + random injection.
%   scatterFrac  – fraction (0–1) of particles to spawn uniformly
%                  after resampling (default 0.05).

    if nargin < 3, scatterFrac = 0.05; end

    N = numel(dataStore.particles);
    weights = [dataStore.particles.weight];

    % Guard against degenerate zero‑weights
    if all(weights == 0)
        weights = ones(1,N) / N;
    end
    weights = weights / sum(weights);

    % ---------- Effective sample size ----------
    Neff = 1 / sum(weights.^2);
    if Neff > 0.5 * N
        return   % high diversity – skip resampling & injection
    end

    % ---------- Systematic resampling ----------
    edges = cumsum(weights);
    u0    = rand / N;
    u     = u0 + (0:N-1)/N;

    idx = zeros(1,N);
    j   = 1;
    for i = 1:N
        while u(i) > edges(j), j = j + 1; end
        idx(i) = j;
    end
    newP = dataStore.particles(idx);     % cloned set

    % ---------- Random‑scatter injection -------
    numRand = round(scatterFrac * N);
    if numRand > 0
        xMin = min(min(map(:,1)), min(map(:,3)));
        xMax = max(max(map(:,1)), max(map(:,3)));
        yMin = min(min(map(:,2)), min(map(:,4)));
        yMax = max(max(map(:,2)), max(map(:,4)));

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
            % reset optional‑wall beliefs
            newP(i).optWallsState(:) = 0;
        end
    end

    % ---------- Reset equal weights ------------
    [newP.weight] = deal(1/N);
    dataStore.particles = newP;
end
