function velocity = flockingVelocity(poses, params)
    % Inputs:
    %   poses - Nx3 matrix where each row is [x, y, theta] for a robot
    %   params - struct with flocking parameters
    % Output:
    %   velocity - Nx2 matrix of velocity vectors [vx, vy]
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    % Extract parameters or use defaults
    if nargin < 2
        params = struct();
    end
    if ~isfield(params, 'sep_radius'), params.sep_radius = 2.0; end
    if ~isfield(params, 'align_weight'), params.align_weight = 1.0; end
    if ~isfield(params, 'cohesion_weight'), params.cohesion_weight = 1.0; end
    if ~isfield(params, 'separation_weight'), params.separation_weight = 1.5; end
    if ~isfield(params, 'perception_radius'), params.perception_radius = 5.0; end
    
    n = size(poses, 1); % Number of robots
    velocity = zeros(n, 2);
    positions = poses(:, 1:2);
    headings = [cos(poses(:, 3)), sin(poses(:, 3))]; % Convert angles to unit vectors
    
    for i = 1:n
        % Initialize rule vectors
        separation = [0, 0];
        alignment = [0, 0];
        cohesion = [0, 0];
        
        % Find neighbors within perception radius
        diffs = positions - positions(i, :);
        distances = sqrt(sum(diffs.^2, 2));
        neighbors = find(distances > 0 & distances < params.perception_radius);
        
        if ~isempty(neighbors)
            % Separation: avoid crowding neighbors
            close_neighbors = neighbors(distances(neighbors) < params.sep_radius);
            if ~isempty(close_neighbors)
                sep_vectors = diffs(close_neighbors, :);
                sep_distances = distances(close_neighbors);
                % Normalize vectors
                for j = 1:length(close_neighbors)
                    if sep_distances(j) > 0
                        sep_vectors(j, :) = sep_vectors(j, :) / sep_distances(j);
                    end
                end
                separation = -sum(sep_vectors, 1) / length(close_neighbors);
            end
            
            % Alignment: steer towards average heading of neighbors
            alignment = sum(headings(neighbors, :), 1) / length(neighbors);
            
            % Cohesion: steer towards center of mass of neighbors
            cohesion = sum(positions(neighbors, :), 1) / length(neighbors) - positions(i, :);
            if norm(cohesion) > 0
                cohesion = cohesion / norm(cohesion);
            end
        end
        
        % Combine rules with weights
        combined = params.separation_weight * separation + ...
                   params.align_weight * alignment + ...
                   params.cohesion_weight * cohesion;
        
        % Normalize to unit magnitude
        if norm(combined) > 0
            velocity(i, :) = combined / norm(combined);
        else
            % If no neighbors, continue in current direction
            velocity(i, :) = headings(i, :);
        end
    end
end