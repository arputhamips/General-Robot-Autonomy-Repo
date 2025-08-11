function [target_idx, bid] = robotBid(robot_location, assigned_targets, unassigned_targets)
    % Calculate bid for a robot based on marginal cost
    % Inputs:
    %   robot_location - 1x2 vector [x, y]
    %   assigned_targets - kx2 matrix of already assigned targets
    %   unassigned_targets - mx2 matrix of unassigned targets
    % Outputs:
    %   target_idx - index of the unassigned target with the lowest marginal cost
    %   bid - the marginal cost (bid value)
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    if isempty(unassigned_targets)
        target_idx = [];
        bid = inf;
        return;
    end
    
    % Calculate current path cost without any new targets
    current_cost = RPC(robot_location, assigned_targets);
    
    % Calculate marginal costs for each unassigned target
    num_unassigned = size(unassigned_targets, 1);
    marginal_costs = zeros(num_unassigned, 1);
    
    for i = 1:num_unassigned
        % Calculate cost with the additional target
        new_targets = [assigned_targets; unassigned_targets(i, :)];
        new_cost = RPC(robot_location, new_targets);
        
        % Marginal cost is the increase in path cost
        marginal_costs(i) = new_cost - current_cost;
    end
    
    % Find the target with minimum marginal cost
    [bid, target_idx] = min(marginal_costs);
end