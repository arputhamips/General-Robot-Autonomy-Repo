function free = isCollisionFree(q1, q2, obstacles)
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A

    free = true;
    % Create a line from q1 to q2
    lineX = [q1(1), q2(1)];
    lineY = [q1(2), q2(2)];
    % For each obstacle polygon
    for ii = 1:length(obstacles)
        poly = obstacles{ii};
        % Ensure polygon is closed
        polyX = [poly(:,1); poly(1,1)];
        polyY = [poly(:,2); poly(1,2)];
        [xi, yi] = polyxpoly(lineX, lineY, polyX, polyY);
        if ~isempty(xi)
            free = false;
            return;
        end
    end
end
