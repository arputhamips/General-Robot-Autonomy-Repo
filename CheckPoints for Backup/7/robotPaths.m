function team_objective = robotPaths(robot_locations, assignments, target_locations)
    % Plot robot paths and calculate team objective
    % Inputs:
    %   robot_locations - nx2 matrix, each row is a robot location [x, y]
    %   assignments - n-element cell array, where assignments{i} contains
    %                 indices of targets assigned to robot i
    %   target_locations - mx2 matrix, each row is a target location [x, y]
    % Output:
    %   team_objective - value of the team objective (total path length)
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    n_robots = size(robot_locations, 1);
    
    % Plot setup
    figure;
    hold on;
    grid on;
    
    % Colors for each robot
    colors = hsv(n_robots);
    
    % Initialize path costs
    path_costs = zeros(n_robots, 1);
    
    % Plot for each robot
    for i = 1:n_robots
        % Get assigned targets for this robot
        assigned_target_indices = assignments{i};
        assigned_targets = target_locations(assigned_target_indices, :);
        
        % If no targets assigned, just plot the robot
        if isempty(assigned_targets)
            plot(robot_locations(i, 1), robot_locations(i, 2), 'o', ...
                'MarkerSize', 10, 'MarkerFaceColor', colors(i, :), ...
                'MarkerEdgeColor', 'k');
            path_costs(i) = 0;
            continue;
        end
        
        % Determine optimal path for the robot
        [optimal_path, path_cost] = findOptimalPath(robot_locations(i, :), assigned_targets);
        path_costs(i) = path_cost;
        
        % Extract coordinates for plotting
        path_x = [robot_locations(i, 1); optimal_path(:, 1)];
        path_y = [robot_locations(i, 2); optimal_path(:, 2)];
        
        % Plot robot starting position
        plot(robot_locations(i, 1), robot_locations(i, 2), 'o', ...
            'MarkerSize', 10, 'MarkerFaceColor', colors(i, :), ...
            'MarkerEdgeColor', 'k');
        
        % Plot targets
        plot(assigned_targets(:, 1), assigned_targets(:, 2), 's', ...
            'MarkerSize', 8, 'MarkerFaceColor', colors(i, :), ...
            'MarkerEdgeColor', 'k');
        
        % Plot path
        plot(path_x, path_y, '-', 'Color', colors(i, :), 'LineWidth', 2);
        
        % Add text labels for targets
        for j = 1:length(assigned_target_indices)
            text(assigned_targets(j, 1), assigned_targets(j, 2), ...
                ['T', num2str(assigned_target_indices(j))], ...
                'FontSize', 8, 'HorizontalAlignment', 'center');
        end
        
        % Add text label for robot
        text(robot_locations(i, 1), robot_locations(i, 2), ...
            ['R', num2str(i)], 'FontSize', 10, 'HorizontalAlignment', 'center');
    end
    
    % Plot unassigned targets, if any
    all_assigned = [assignments{:}];
    unassigned = setdiff(1:size(target_locations, 1), all_assigned);
    
    if ~isempty(unassigned)
        unassigned_targets = target_locations(unassigned, :);
        plot(unassigned_targets(:, 1), unassigned_targets(:, 2), 'x', ...
            'MarkerSize', 8, 'Color', 'k', 'LineWidth', 2);
        
        % Add text labels for unassigned targets
        for j = 1:length(unassigned)
            text(unassigned_targets(j, 1), unassigned_targets(j, 2), ...
                ['T', num2str(unassigned(j))], ...
                'FontSize', 8, 'HorizontalAlignment', 'center');
        end
    end
    
    % Calculate team objective (total path length)
    team_objective = sum(path_costs);
    
    % Finalize plot
    title(['Robot Paths - Total Path Length: ', num2str(team_objective, '%.2f')]);
    xlabel('X Position');
    ylabel('Y Position');
    axis equal;
    hold off;
end

function [optimal_path, path_cost] = findOptimalPath(robot_location, targets)
    % Find optimal path ordering for a robot to visit all targets
    % Inputs:
    %   robot_location - 1x2 vector [x, y]
    %   targets - kx2 matrix of target locations
    % Outputs:
    %   optimal_path - kx2 matrix of ordered target locations
    %   path_cost - total path cost
    
    if isempty(targets)
        optimal_path = [];
        path_cost = 0;
        return;
    end
    
    % If there's only one target, the path is trivial
    if size(targets, 1) == 1
        optimal_path = targets;
        path_cost = norm(robot_location - targets);
        return;
    end
    
    % For small number of targets, try all permutations
    num_targets = size(targets, 1);
    if num_targets <= 8  % Limit for permutation calculation
        % Generate all possible permutations
        perms_indices = perms(1:num_targets);
        
        % Calculate cost for each permutation
        min_cost = inf;
        best_perm = [];
        
        for i = 1:size(perms_indices, 1)
            perm = perms_indices(i, :);
            permuted_targets = targets(perm, :);
            
            % Calculate path cost for this permutation
            current_cost = norm(robot_location - permuted_targets(1, :));
            for j = 1:(num_targets-1)
                current_cost = current_cost + norm(permuted_targets(j, :) - permuted_targets(j+1, :));
            end
            
            % Update if better
            if current_cost < min_cost
                min_cost = current_cost;
                best_perm = perm;
            end
        end
        
        optimal_path = targets(best_perm, :);
        path_cost = min_cost;
    else
        % For larger problems, use a greedy approach (nearest neighbor)
        remaining_targets = targets;
        optimal_path = zeros(num_targets, 2);
        current_pos = robot_location;
        path_cost = 0;
        
        for i = 1:num_targets
            % Find nearest target
            distances = sqrt(sum((remaining_targets - repmat(current_pos, size(remaining_targets, 1), 1)).^2, 2));
            [min_dist, min_idx] = min(distances);
            
            % Add to path
            optimal_path(i, :) = remaining_targets(min_idx, :);
            path_cost = path_cost + min_dist;
            
            % Update current position and remove visited target
            current_pos = optimal_path(i, :);
            remaining_targets(min_idx, :) = [];
            
            if isempty(remaining_targets)
                break;
            end
        end
    end
end