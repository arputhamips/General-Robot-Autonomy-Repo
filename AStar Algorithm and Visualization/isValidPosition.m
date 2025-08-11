function valid = isValidPosition(x, y, map)
    % Check if a position is valid (not inside any walls)
    
    % Buffer distance from walls (small value to prevent being exactly on the wall)
    buffer = 0.15; % 15 cm buffer
    
    % Let's use ray casting in four directions
    angles = [0, pi/2, pi, 3*pi/2];
    
    for angle = angles
        dist = raycast(x, y, angle, map, 1000);
        if dist < buffer
            valid = false;
            return;
        end
    end
    
    valid = true;
end