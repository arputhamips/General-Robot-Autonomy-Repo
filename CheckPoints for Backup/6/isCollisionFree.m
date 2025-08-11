function free = isCollisionFree(q1, q2, obstacles)
% isCollisionFree: Checks if the line between q1 and q2 intersects any inflated obstacle.
% Toolbox-free version using inpolygon.
% 
% INPUTS:
%   q1, q2       - 1x2 vectors representing the endpoints of the path segment
%   obstacles    - Cell array of inflated polygon obstacles, each n×2 [x y]
%
% OUTPUT:
%   free         - True if path is collision-free, false otherwise

    free = true;

    % Sample N points along the straight line from q1 to q2
    nPoints = 20;  % Increase for more accuracy
    xSamples = linspace(q1(1), q2(1), nPoints);
    ySamples = linspace(q1(2), q2(2), nPoints);

    for i = 1:length(obstacles)
        poly = obstacles{i};
        polyX = poly(:,1);
        polyY = poly(:,2);

        % Check if any of the sampled points lie inside this polygon
        in = inpolygon(xSamples, ySamples, polyX, polyY);
        if any(in)
            free = false;
            return;
        end
    end
end