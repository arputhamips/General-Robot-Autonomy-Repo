function dataStore = initializeParticlesB(dataStore, map, numParticles, optWalls, centers, radius)
%INITIALIZEPARTICLESB  Cluster particles around given centers within radius.
%   centers: K×2 array of [x y]; particles drawn uniformly in disks of ‘radius’ around them.
%   If centers has more than one row, each particle picks one at random.
    if nargin < 6, radius = 0.05; end

    nw = size(optWalls,1);
    tpl = struct('x',0,'y',0,'theta',0,'weight',1/numParticles,'optWallsState',zeros(nw,1));
    dataStore.particles = repmat(tpl,1,numParticles);

    for i = 1:numParticles
        c = centers(randi(size(centers,1)),:);
        r = radius * sqrt(rand);
        ang = 2*pi*rand;
        dataStore.particles(i).x     = c(1) + r*cos(ang);
        dataStore.particles(i).y     = c(2) + r*sin(ang);
        dataStore.particles(i).theta = wrapToPi(2*pi*rand - pi);
    end

    [dataStore.particles.weight] = deal(1/numParticles);
end
