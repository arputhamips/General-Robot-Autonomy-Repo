function plotBinaryGrid(logOdds, timeStep, threshold)
% plotBinaryGrid Binarizes the occupancy grid based on a probability threshold
% and plots the result (0 or 1).
%
%   plotBinaryGrid(logOdds, timeStep, threshold) converts the log-odds grid 
%   to probabilities, thresholds them, and displays the result as a binary 
%   occupancy grid.
%
%   INPUTS:
%       logOdds   - n-by-m matrix of log-odds occupancy values
%       timeStep  - scalar for labeling the plot (e.g. final time)
%       threshold - probability threshold for deciding "occupied vs. free"
%
%   Cells with p > threshold are labeled 1 (occupied), otherwise 0 (free).

    % Convert log-odds to probabilities
    prob = exp(logOdds) ./ (1 + exp(logOdds));

    % Apply threshold
    binGrid = prob > threshold;

    % Plot the binary grid
    figure;
    imagesc(binGrid);  % 1's and 0's
    colormap(gray);    % black/white display
    caxis([0 1]);      % ensure 0 is black, 1 is white or vice versa
    axis equal tight;
    set(gca, 'YDir', 'normal');
    colorbar;
    title(sprintf('Binary Occupancy Grid at Time Step %.4f', timeStep));
    xlabel('Grid X');
    ylabel('Grid Y');
end