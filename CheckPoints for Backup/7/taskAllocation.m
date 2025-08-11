function assignments = taskAllocation(robot_locations, target_locations)
    % Allocate targets to robots using market-based approach
    % Inputs:
    %   robot_locations - nx2 matrix, each row is a robot location [x, y]
    %   target_locations - mx2 matrix, each row is a target location [x, y]
    % Output:
    %   assignments - n-element cell array, where assignments{i} contains
    %                 indices of targets assigned to robot i
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    n_robots = size(robot_locations, 1);
    n_targets = size(target_locations, 1);
    
    % Initialize assignments
    assignments = cell(n_robots, 1);
    for i = 1:n_robots
        assignments{i} = [];
    end
    
    % Initialize unassigned targets
    unassigned_indices = 1:n_targets;
    
    % Continue until all targets are assigned or no more assignments can be made
    while ~isempty(unassigned_indices)
        best_robot = 0;
        best_target_idx = 0;
        best_bid = inf;
        
        % Find best bid across all robots
        for i = 1:n_robots
            % Get current assigned targets for this robot
            current_targets = target_locations(assignments{i}, :);
            
            % Get unassigned targets
            unassigned_targets = target_locations(unassigned_indices, :);
            
            % Calculate bid for this robot
            [local_target_idx, bid] = robotBid(robot_locations(i, :), ...
                                              current_targets, ...
                                              unassigned_targets);
            
            % If this is a better bid, update
            if ~isempty(local_target_idx) && bid < best_bid
                best_robot = i;
                best_target_idx = local_target_idx;
                best_bid = bid;
            elseif ~isempty(local_target_idx) && abs(bid - best_bid) < 1e-10
                % Breaking ties: prefer robot with fewer assigned targets
                if length(assignments{i}) < length(assignments{best_robot})
                    best_robot = i;
                    best_target_idx = local_target_idx;
                    best_bid = bid;
                elseif length(assignments{i}) == length(assignments{best_robot})
                    % If equal number of targets, prefer robot with lower index
                    if i < best_robot
                        best_robot = i;
                        best_target_idx = local_target_idx;
                        best_bid = bid;
                    end
                end
            end
        end
        
        % If no valid bid was found, break
        if best_robot == 0
            break;
        end
        
        % Assign the target to the winning robot
        global_target_idx = unassigned_indices(best_target_idx);
        assignments{best_robot} = [assignments{best_robot}, global_target_idx];
        
        % Remove the assigned target from unassigned list
        unassigned_indices(best_target_idx) = [];
    end
end