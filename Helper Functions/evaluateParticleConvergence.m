function [bestPose, score, converged] = evaluateParticleConvergence(dataStore)
%EVALUATEPARTICLECONVERGENCE Evaluate if particles have converged to a single pose
%
% [bestPose, score, converged] = evaluateParticleConvergence(dataStore)
%
% Inputs:
%   dataStore - Data structure with particles
%
% Outputs:
%   bestPose - Best pose estimate [x; y; theta]
%   score - Convergence score (higher is better)
%   converged - Boolean indicating if particles have converged

% Extract particle data
weights = [dataStore.particles.weight];
xs = [dataStore.particles.x];
ys = [dataStore.particles.y];
thetas = [dataStore.particles.theta];

% Compute weighted mean
xMean = sum(weights .* xs);
yMean = sum(weights .* ys);
thetaMean = atan2(sum(weights .* sin(thetas)), sum(weights .* cos(thetas)));

% Wrap theta to [-pi, pi]
thetaMean = wrapToPi(thetaMean);

% Best pose estimate
bestPose = [xMean; yMean; thetaMean];

% Convergence metrics
maxWeight = max(weights);
Neff = 1 / sum(weights.^2);  % Effective particle count
positionVar = sum(weights .* ((xs - xMean).^2 + (ys - yMean).^2));

% Angular variance (accounting for circular wrapping)
dThetas = wrapToPi(thetas - thetaMean);
thetaVar = sum(weights .* (dThetas.^2));

% Combined convergence score - higher is better
score = maxWeight * (1 - positionVar) * (1 - min(thetaVar, pi/2)/(pi/2));

% Thresholds for convergence
converged = (maxWeight > 0.01) && (positionVar < 0.05) && (thetaVar < (20*pi/180)^2);

fprintf('Convergence metrics:\n');
fprintf('  Max weight: %.4f\n', maxWeight);
fprintf('  Position variance: %.4f\n', positionVar);
fprintf('  Orientation variance: %.1f degrees\n', sqrt(thetaVar)*180/pi);
fprintf('  Effective particle count: %.1f\n', Neff);
fprintf('  Convergence score: %.4f\n', score);
end