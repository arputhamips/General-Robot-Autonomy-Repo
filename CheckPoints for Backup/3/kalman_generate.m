clc; clear; close all;

% Load required data
load('HW3map.mat');       % Map containing walls
load('stationary.mat');   % Sensor measurements

% Initial state estimate
mu0 = [5; 5];              % Initial position guess (X, Y)
sigma0 = eye(2) * 0.5;     % Initial covariance (assumed uncertainty)

% Run the Kalman Filter function
[muFinal, sigmaFinal] = KFStationary(mu0, sigma0, stationary);

% Validate covariance matrices
if ~all(size(sigma0) == [2,2]) || ~all(size(sigmaFinal) == [2,2])
    error('Error: Covariance matrices must be 2x2.');
end
if any(isnan(sigma0(:))) || any(isnan(sigmaFinal(:)))
    error('Error: Covariance matrix contains NaN values.');
end
if any(eig(sigma0) <= 0) || any(eig(sigmaFinal) <= 0)
    error('Error: Covariance matrix is not positive semi-definite.');
end

% Plot the environment (walls)
figure; hold on; axis equal;
for i = 1:size(HW3map,1)
    plot([HW3map(i,1), HW3map(i,3)], [HW3map(i,2), HW3map(i,4)], 'k', 'LineWidth', 2);
end
xlabel('X Position'); ylabel('Y Position');
title('Kalman Filter Position and Covariance Estimates');

% Plot initial position estimate
plot(mu0(1), mu0(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plotCovEllipse(mu0(:), sigma0, 1, {'color', 'b', 'linewidth', 2}); % 1σ ellipse for initial estimate

% Plot final position estimate
plot(muFinal(1), muFinal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plotCovEllipse(muFinal(:), sigmaFinal, 1, {'color', 'r', 'linewidth', 2}); % 1σ ellipse for final estimate

legend('Walls', 'Initial Position', 'Initial Covariance', 'Final Position', 'Final Covariance');
grid on; hold off;