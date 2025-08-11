function [intersects, x, y, distance] = lineIntersection(x1, y1, x2, y2, x3, y3, x4, y4)
    % Check if line segments (x1,y1)-(x2,y2) and (x3,y3)-(x4,y4) intersect
    
    % Calculate the denominator
    den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    
    % If denominator is zero, lines are parallel
    if abs(den) < 1e-10
        intersects = false;
        x = 0;
        y = 0;
        distance = inf;
        return;
    end
    
    % Calculate intersection parameters
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den;
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den;
    
    % Check if intersection lies on both segments
    if ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1
        intersects = true;
        x = x1 + ua * (x2 - x1);
        y = y1 + ua * (y2 - y1);
        distance = sqrt((x - x1)^2 + (y - y1)^2);
    else
        intersects = false;
        x = 0;
        y = 0;
        distance = inf;
    end
end