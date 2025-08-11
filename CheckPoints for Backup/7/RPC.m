function cost = RPC(robot_location, targets)
    % Calculate minimum robot path cost given a robot location and targets
    % Inputs:
    %   robot_location - 1x2 vector [x, y]
    %   targets - mx2 matrix, each row is a target location [x, y]
    % Output:
    %   cost - minimum path cost (sum of Euclidean distances)
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    % If there are no targets, cost is 0
    if isempty(targets)
        cost = 0;
        return;
    end
    
    % Number of targets
    num_targets = size(targets, 1);
    
    % If there's only one target, calculate direct distance
    if num_targets == 1
        cost = norm(robot_location - targets);
        return;
    end
    
    % For multiple targets, use permutations to find the optimal ordering
    % Calculate all pairwise distances (including from robot to each target)
    all_locations = [robot_location; targets];
    num_locations = size(all_locations, 1);
    
    % Distance matrix
    dist_matrix = zeros(num_locations, num_locations);
    for i = 1:num_locations
        for j = 1:num_locations
            dist_matrix(i, j) = norm(all_locations(i, :) - all_locations(j, :));
        end
    end
    
    % Try all permutations for small number of targets
    if num_targets <= 8  % Limit for permutation calculation
        % Generate all possible target permutations
        target_perms = perms(2:num_locations);
        
        % Calculate cost for each permutation
        min_cost = inf;
        for i = 1:size(target_perms, 1)
            perm = [1, target_perms(i, :)];  % Add robot (index 1) at the beginning
            perm_cost = 0;
            
            % Calculate path cost for this permutation
            for j = 1:(num_locations-1)
                perm_cost = perm_cost + dist_matrix(perm(j), perm(j+1));
            end
            
            % Update minimum cost
            if perm_cost < min_cost
                min_cost = perm_cost;
            end
        end
        cost = min_cost;
    else
        % For larger problems, use a greedy approach (nearest neighbor)
        current_pos = 1;  % Start at robot position
        unvisited = 2:num_locations;
        total_cost = 0;
        
        while ~isempty(unvisited)
            % Find closest unvisited target
            [~, idx] = min(dist_matrix(current_pos, unvisited));
            next_pos = unvisited(idx);
            
            % Add distance to total cost
            total_cost = total_cost + dist_matrix(current_pos, next_pos);
            
            % Update current position and remove from unvisited
            current_pos = next_pos;
            unvisited(idx) = [];
        end
        cost = total_cost;
    end
end