function poses = generateInitialPoses(n, arena_size, min_distance)
    % Generate non-colliding initial poses for n robots
    % Inputs:
    %   n - number of robots
    %   arena_size - size of the square arena [width, height]
    %   min_distance - minimum distance between robots
    % Output:
    %   poses - Nx3 matrix of poses [x, y, theta]
    
    if nargin < 1, n = 25; end
    if nargin < 2, arena_size = [100, 100]; end
    if nargin < 3, min_distance = 3.0; end
    
    poses = zeros(n, 3);
    count = 0;
    
    while count < n
        % Generate a random position and orientation
        x = arena_size(1) * rand();
        y = arena_size(2) * rand();
        theta = 2 * pi * rand(); % Random orientation
        
        % Check for collisions with existing robots
        collision = false;
        for i = 1:count
            dist = norm(poses(i, 1:2) - [x, y]);
            if dist < min_distance
                collision = true;
                break;
            end
        end
        
        % If no collision, add the robot
        if ~collision
            count = count + 1;
            poses(count, :) = [x, y, theta];
        end
    end
end

