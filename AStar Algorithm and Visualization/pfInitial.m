function dataStore = pfInitial(dataStore, map, waypoints, optWalls, beaconLoc)
%PFINITIAL  Run a single update cycle of PF to estimate the initial pose.
% Assumes robot is exactly at one of the waypoints.
% Returns dataStore with initialized truthPose

    numParticles = 500;

    % --- Initialize particles tightly around waypoints
    dataStore = initializeParticlesB(dataStore, map, numParticles, optWalls, waypoints);

    % --- Perform one measurement update
    dataStore = measurementUpdate(dataStore, map, beaconLoc, optWalls);

    % --- Pick the most likely particle as truthPose
    [~, bestIdx] = max([dataStore.particles.weight]);
    bestP = dataStore.particles(bestIdx);

    dataStore.truthPose = [0, bestP.x, bestP.y, bestP.theta];
end
