function navigationPlot(map, goal, k, lambda, x_points, y_points)
% navigationPlot - Evaluates and plots the navigation function for a sphere world.
%
%   INPUTS:
%       map      : m×3 array [x_center, y_center, radius] for the sphere world.
%                  Row 1 is the workspace boundary; rows 2…m are obstacle spheres.
%       goal     : 1×2 vector [x_goal, y_goal].
%       k        : Tuning exponent.
%       lambda   : Blending coefficient.
%       x_points : Vector of x–coordinates (e.g., linspace(0,100,100)).
%       y_points : Vector of y–coordinates.
%
%   This function evaluates spherePoint at each grid point and produces a filled contour plot.

% Autonomous Mobile Robots - HW6
% NIRMAL A J L A

[X, Y] = meshgrid(x_points, y_points);
phi = zeros(size(X));

for i = 1:size(X,1)
    for j = 1:size(X,2)
        q = [X(i,j), Y(i,j)];
        phi(i,j) = spherePoint(map, goal, k, lambda, q);
    end
end

figure;
contourf(X, Y, phi, 50, 'LineColor', 'none');
colorbar;
xlabel('X (m)');
ylabel('Y (m)');
title(sprintf('Navigation Function, k = %g, lambda = %g', k, lambda));
end