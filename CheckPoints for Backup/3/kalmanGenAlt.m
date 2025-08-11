clc; clear; close all;

% Load required data
load('HW3map.mat');       % Map containing walls
load('stationary.mat');   % Sensor measurements

% Initial state estimate
mu0 = [15; 10];              % Initial position guess (X, Y)
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

% Function to plot covariance ellipse manually
plot_covariance_ellipse = @(mu, sigma, color) ...
    ellipse_plot(mu, sigma, color);

% Plot the environment (walls)
figure; hold on; axis equal;
for i = 1:size(HW3map,1)
    plot([HW3map(i,1), HW3map(i,3)], [HW3map(i,2), HW3map(i,4)], 'k', 'LineWidth', 2);
end
xlabel('X Position'); ylabel('Y Position');
title('Kalman Filter Position and Covariance Estimates');

% Plot initial position estimate
plot(mu0(1), mu0(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot_covariance_ellipse(mu0, sigma0, 'b'); % Manually plotted ellipse

% Plot final position estimate
plot(muFinal(1), muFinal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot_covariance_ellipse(muFinal, sigmaFinal, 'r'); % Manually plotted ellipse

legend({'Walls', 'Initial Position', 'Final Position', 'Initial Covariance', 'Final Covariance'}, 'Location', 'best');
grid on; hold off;

% Function to manually plot covariance ellipse
function ellipse_plot(mu, sigma, color)
    theta = linspace(0, 2*pi, 100); % Generate angles
    [eigVec, eigVal] = eig(sigma);  % Eigenvalue decomposition
    a = sqrt(eigVal(1,1)); % Major axis
    b = sqrt(eigVal(2,2)); % Minor axis
    ellipse_points = [a*cos(theta); b*sin(theta)]; % Parametric ellipse

    % Rotate by eigenvectors
    rotated_ellipse = eigVec * ellipse_points;
    x_ellipse = rotated_ellipse(1, :) + mu(1);
    y_ellipse = rotated_ellipse(2, :) + mu(2);

    % Plot ellipse
    plot(x_ellipse, y_ellipse, color, 'LineWidth', 2);
end