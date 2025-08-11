function flocking(initial_poses, mode, sim_time, dt)
    % Simulate flocking behavior with different neighbor information modes
    % Inputs:
    %   initial_poses - Nx3 matrix of initial poses [x, y, theta]
    %   mode - 'all', 'nearest5', or 'mod4' for different information sharing
    %   sim_time - total simulation time
    %   dt - time step
    % 
    % 
    % AMR - MULTI-ROBOT-EXTRA CREDIT
    % NIRMAL A J L A
    
    if nargin < 3, sim_time = 100; end
    if nargin < 4, dt = 0.1; end
    
    n = size(initial_poses, 1);
    poses = initial_poses;
    params = struct('sep_radius', 3.0, 'align_weight', 1.0, ...
                    'cohesion_weight', 1.0, 'separation_weight', 1.5, ...
                    'perception_radius', 15.0);
    
    % Setup figure for animation
    figure;
    h_robots = gobjects(n, 1);  % Handles for robot markers
    h_tails = gobjects(n, 1);   % Handles for trajectory tails
    hold on;
    
    % Color map for different robots
    colors = hsv(n);
    
    % Initialize trajectories
    trajectories = cell(n, 1);
    for i = 1:n
        trajectories{i} = poses(i, 1:2);
        h_robots(i) = plot(poses(i, 1), poses(i, 2), 'o', 'MarkerSize', 8, ...
                          'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        h_tails(i) = plot(poses(i, 1), poses(i, 2), '-', 'Color', colors(i,:), 'LineWidth', 1);
    end
    
    axis equal;
    title(['Flocking Simulation - Mode: ', mode]);
    xlabel('X Position');
    ylabel('Y Position');
    grid on;
    
    % Time loop
    steps = sim_time / dt;
    for t = 1:steps
        % Determine visible neighbors based on mode
        visible_neighbors = cell(n, 1);
        
        switch lower(mode)
            case 'all'
                % All robots are visible to each other
                for i = 1:n
                    visible_neighbors{i} = setdiff(1:n, i);
                end
                
            case 'nearest5'
                % Only 5 nearest neighbors are visible
                for i = 1:n
                    dists = sqrt(sum((poses(:, 1:2) - poses(i, 1:2)).^2, 2));
                    [~, idx] = sort(dists);
                    visible_neighbors{i} = idx(2:min(6, n)); % Skip self
                end
                
            case 'mod4'
                % Only robots with same ID mod 4
                for i = 1:n
                    visible_neighbors{i} = find(mod(1:n, 4) == mod(i, 4) & (1:n) ~= i);
                end
                
            otherwise
                error('Unknown mode. Use ''all'', ''nearest5'', or ''mod4''');
        end
        
        % Calculate velocities with filtered information
        velocities = zeros(n, 2);
        for i = 1:n
            if isempty(visible_neighbors{i})
                % If no visible neighbors, continue in current direction
                heading = [cos(poses(i, 3)), sin(poses(i, 3))];
                velocities(i, :) = heading;
            else
                % For robot i and its visible neighbors
                neighbor_indices = visible_neighbors{i};
                
                % Initialize rule vectors
                separation = [0, 0];
                alignment = [0, 0];
                cohesion = [0, 0];
                
                % Current robot position and heading
                pos_i = poses(i, 1:2);
                heading_i = [cos(poses(i, 3)), sin(poses(i, 3))];
                
                % Process neighbors
                if ~isempty(neighbor_indices)
                    % Get neighbor positions and headings
                    pos_neighbors = poses(neighbor_indices, 1:2);
                    headings_neighbors = [cos(poses(neighbor_indices, 3)), sin(poses(neighbor_indices, 3))];
                    
                    % Separation: avoid crowding neighbors
                    diffs = pos_neighbors - repmat(pos_i, length(neighbor_indices), 1);
                    distances = sqrt(sum(diffs.^2, 2));
                    
                    % Find close neighbors for separation
                    close_neighbors = distances < params.sep_radius;
                    if any(close_neighbors)
                        sep_vectors = diffs(close_neighbors, :);
                        sep_distances = distances(close_neighbors);
                        
                        % Avoid division by zero
                        for j = 1:length(sep_distances)
                            if sep_distances(j) > 0
                                sep_vectors(j, :) = sep_vectors(j, :) / sep_distances(j);
                            end
                        end
                        
                        separation = -sum(sep_vectors, 1) / sum(close_neighbors);
                    end
                    
                    % Alignment: steer towards average heading of neighbors
                    alignment = sum(headings_neighbors, 1) / size(headings_neighbors, 1);
                    
                    % Cohesion: steer towards center of mass of neighbors
                    cohesion = sum(pos_neighbors, 1) / size(pos_neighbors, 1) - pos_i;
                    if norm(cohesion) > 0
                        cohesion = cohesion / norm(cohesion);
                    end
                    
                    % Combine rules with weights
                    combined = params.separation_weight * separation + ...
                               params.align_weight * alignment + ...
                               params.cohesion_weight * cohesion;
                    
                    % Normalize to unit magnitude
                    if norm(combined) > 0
                        velocities(i, :) = combined / norm(combined);
                    else
                        % If no effective force, continue in current direction
                        velocities(i, :) = heading_i;
                    end
                else
                    % If no visible neighbors, continue in current direction
                    velocities(i, :) = heading_i;
                end
            end
        end
        
        % Update poses
        for i = 1:n
            % Update position
            poses(i, 1:2) = poses(i, 1:2) + velocities(i, :) * dt;
            
            % Update orientation (heading)
            poses(i, 3) = atan2(velocities(i, 2), velocities(i, 1));
            
            % Update trajectory
            trajectories{i} = [trajectories{i}; poses(i, 1:2)];
            
            % Update visualization
            set(h_robots(i), 'XData', poses(i, 1), 'YData', poses(i, 2));
            set(h_tails(i), 'XData', trajectories{i}(:, 1), 'YData', trajectories{i}(:, 2));
        end
        
        % Adjust axis limits to keep all robots in view
        axis_margin = 10;
        xlim([min(poses(:, 1)) - axis_margin, max(poses(:, 1)) + axis_margin]);
        ylim([min(poses(:, 2)) - axis_margin, max(poses(:, 2)) + axis_margin]);
        
        drawnow;
        pause(0.01);
    end
    
    % Final plot with all trajectories
    figure;
    hold on;
    for i = 1:n
        plot(trajectories{i}(:, 1), trajectories{i}(:, 2), '-', 'Color', colors(i,:), 'LineWidth', 1);
        plot(trajectories{i}(1, 1), trajectories{i}(1, 2), 'o', 'MarkerSize', 8, ...
             'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        plot(trajectories{i}(end, 1), trajectories{i}(end, 2), 's', 'MarkerSize', 8, ...
             'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
    end
    axis equal;
    title(['Flocking Trajectories - Mode: ', mode]);
    xlabel('X Position');
    ylabel('Y Position');
    grid on;
    legend('Trajectories', 'Start Positions', 'End Positions');
    hold off;
end