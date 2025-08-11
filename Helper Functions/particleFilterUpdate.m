% =========================== particleFilterUpdate.m ===========================
function dataStore = particleFilterUpdate(Robot, dataStore,numParticles, map, beaconLoc, optWalls, waypoints)
%   One full particle‑filter iteration
%   NOTE: waypoints is now an explicit input

    % -------- initialisation ----------------------------------------------
    if ~isfield(dataStore,'particles') || isempty(dataStore.particles)
        % numParticles = 600;
        dataStore = initializeParticles(dataStore, map, numParticles, optWalls, waypoints);
    end

    % -------- prediction ---------------------------------------------------
    dataStore = motionUpdate(dataStore);

    % -------- correction ---------------------------------------------------
    dataStore = measurementUpdate(dataStore, map, beaconLoc, optWalls);

    % ---------- log weighted‑mean pose -------------------------------------
    w  = [dataStore.particles.weight];          % 1×N
    x  = [dataStore.particles.x];
    y  = [dataStore.particles.y];
    th = [dataStore.particles.theta];
    
    xHat  = sum(w .* x);
    yHat  = sum(w .* y);
    thHat = atan2( sum(w .* sin(th)), ...
                   sum(w .* cos(th)) );        % circular mean
    
    tNow  = toc;
    dataStore.truthPose(end+1,:) = [tNow  xHat  yHat  thHat];

     % -------- resampling ---------------------------------------------------
    dataStore = resampleParticles(dataStore, map, 0.05);      % 5 % random

end