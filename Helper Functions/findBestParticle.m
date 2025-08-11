function [bestPose, bestScore] = findBestParticle(dataStore, robotX, robotY)
%FINDBESTPARTICLE Find the best particle based on weights
%
% [bestPose, bestScore] = findBestParticle(dataStore, robotX, robotY)
%
% Inputs:
%   dataStore - Data structure with particles
%   robotX - Robot X position
%   robotY - Robot Y position
%
% Outputs:
%   bestPose - Best pose [x; y; theta]
%   bestScore - Score of the best particle

% Extract weights and thetas
weights = [dataStore.particles.weight];
thetas = [dataStore.particles.theta];

% Find the particle with maximum weight
[maxWeight, maxIdx] = max(weights);

% Get the orientation of the best particle
bestTheta = thetas(maxIdx);

% Calculate score (can incorporate additional metrics beyond just weight)
bestScore = maxWeight;

% Create final pose vector
bestPose = [robotX; robotY; bestTheta];

% Also calculate the weighted circular mean for comparison
circMeanTheta = atan2(sum(weights .* sin(thetas)), sum(weights .* cos(thetas)));

fprintf('Best particle theta: %.2f degrees (weight: %.4f)\n', bestTheta*180/pi, maxWeight);
fprintf('Weighted circular mean theta: %.2f degrees\n', circMeanTheta*180/pi);

end