function potentialPlot(map, goal, c_att, c_rep, Q, x_points, y_points)
% potentialPlot generates a 2D contour plot and vector field (quiver)
%
% INPUTS:
%   map     : kx3 matrix defining the spheres [x_center y_center radius]
%   goal    : 1x2 array [x_goal y_goal]
%   c_att   : attractive potential scaling
%   c_rep   : repulsive potential scaling
%   Q       : influence range of obstacles
%   x_points: array of x coordinates (vector)
%   y_points: array of y coordinates (vector)

% Autonomous Mobile Robots - HW6
% NIRMAL A J L A


% Create mesh grid
[X, Y] = meshgrid(x_points, y_points);

% Initialize potential and gradient matrices
U = zeros(size(X));
Ux = zeros(size(X));
Uy = zeros(size(X));

% Evaluate potential and gradient at each grid point
for i = 1:size(X,1)
    for j = 1:size(X,2)
        pt = [X(i,j), Y(i,j)];
        [U(i,j), gradU] = potentialPoint(map, goal, c_att, c_rep, Q, pt);
        Ux(i,j) = -gradU(1);  % Negative gradient (descent direction)
        Uy(i,j) = -gradU(2);
    end
end

% Optional: Cap very high potential values for clean visualization
U(U > 1e4) = 1e4;

% Plotting
figure;

% Plot the potential field as a filled contour map
contourf(X, Y, U, 50, 'LineColor', 'none'); % 50 contour levels
hold on;
colormap('jet');  % Color scheme
colorbar;
xlabel('X (m)');
ylabel('Y (m)');
title('2D Potential Field and Vector Field');
axis equal tight;
grid on;

% Normalize vector arrows for consistent length
mag = sqrt(Ux.^2 + Uy.^2);
Ux_norm = Ux ./ (mag + eps);  % Add eps to avoid division by zero
Uy_norm = Uy ./ (mag + eps);

% Plot the vector field (quiver)
quiver(X, Y, Ux_norm, Uy_norm, 0.5, 'w'); % 0.5 scaling for better visibility

hold off;

end