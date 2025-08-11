function distance = raycast(x, y, angle, map, maxRange)
    % Cast a ray from (x,y) in direction 'angle' until hitting a wall or maxRange
    
    % Ray end point
    xEnd = x + maxRange * cos(angle);
    yEnd = y + maxRange * sin(angle);
    
    % Initialize distance
    minDistance = maxRange;
    
    % Check intersection with each wall segment
    for i = 1:size(map, 1)
        [intersects, ix, iy, dist] = lineIntersection(x, y, xEnd, yEnd, map(i, 1), map(i, 2), map(i, 3), map(i, 4));
        
        if intersects && dist < minDistance
            minDistance = dist;
        end
    end
    
    distance = minDistance;
end