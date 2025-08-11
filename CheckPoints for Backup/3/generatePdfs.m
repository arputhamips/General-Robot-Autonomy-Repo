% clc; clear; close all;

% Load data
load('HW3map.mat');
load('stationary.mat');

% Grid sizes
smallGridSize = [10, 10];
largeGridSize = [40, 22];

% Create grid coordinates (X and Y should be vectors)
[smallX, smallY] = meshgrid(1:smallGridSize(2), 1:smallGridSize(1));
[largeX, largeY] = meshgrid(1:largeGridSize(2), 1:largeGridSize(1));

% Convert to vectors (required for image function)
smallXVec = smallX(1, :);
smallYVec = smallY(:, 1);
largeXVec = largeX(1, :);
largeYVec = largeY(:, 1);

% Initialize uniform probability distributions
smallGridProb = ones(smallGridSize) / prod(smallGridSize);
largeGridProb = ones(largeGridSize) / prod(largeGridSize);

% Plot 10x10 grid initial and final beliefs
figure;
subplot(1,2,1);
plotGridBelief(smallXVec, smallYVec, smallGridProb);
title('Initial Grid Belief (10x10)');

% Update probabilities using grid localization function
smallGridProb = gridLocalizationStationary(smallGridProb, HW3map, stationary);

subplot(1,2,2);
plotGridBelief(smallXVec, smallYVec, smallGridProb);
title('Final Grid Belief (10x10)');

% Plot 40x22 grid initial and final beliefs
figure;
subplot(1,2,1);
plotGridBelief(largeXVec, largeYVec, largeGridProb);
title('Initial Grid Belief (40x22)');

% Update probabilities using grid localization function
largeGridProb = gridLocalizationStationary(largeGridProb, HW3map, stationary);

subplot(1,2,2);
plotGridBelief(largeXVec, largeYVec, largeGridProb);
title('Final Grid Belief (40x22)');
