function plotOccupancyGrid(logOdds, timeStep)
% plotOccupancyGrid Plots the occupancy grid using log-odds.
%
%   plotOccupancyGrid(logOdds, timeStep) displays an occupancy grid where
%   unoccupied cells appear white, occupied cells black, and uncertain cells
%   in shades of gray.
%
%   INPUTS:
%       logOdds  - An n-by-m matrix of log-odds occupancy values.
%       timeStep - A scalar indicating the time step (used for the plot title).
%
%   The function converts log-odds to probabilities using the logistic function,
%   and then plots the grid using imagesc with the colormap set to flipud(gray).
%
%   Example usage:
%       plotOccupancyGrid(currentLogOdds, t);
% 
% 
% AUTONOMOUS MOBILE ROBOTS HW5
%  CORNELL UNIVERSITY
%  NIRMAL , A J L A

% Convert log-odds to occupancy probability (range 0 to 1)
prob = exp(logOdds) ./ (1 + exp(logOdds));

% Create a new figure and display the occupancy grid.
figure;
imagesc(prob);
set(gca, 'YDir', 'normal');  % Ensure the Y-axis is in the correct direction
colormap(flipud(gray));      % Free space is white, occupied is black
colorbar;
axis equal tight;
title(['Occupancy Grid at Time Step ', num2str(timeStep)]);
xlabel('Grid X');
ylabel('Grid Y');
end