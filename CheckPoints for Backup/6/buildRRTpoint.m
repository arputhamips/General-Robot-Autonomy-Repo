function [waypoints_startTOgoal] = buildRRTpoint(map,mapBoundary,start,goal)
% BUILDRRTPOINT
%
%       INPUTS:
%           map             file name for text file representing the obstacles in the workspace
%                           for example map = 'hw6b.txt'. Each row in this file contains the vertices
%                           of one polygonal obstacle: v1x, v1y, v2x, v2y, etc. The vertices are given in
%                           counterclockwise order. If an obstacle has fewer vertices, unused entries 
%                           in the line will contain the value zero.
%           mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                           and top right corner of the workspace respectively  
%           start           1 x 2 array [x y], for start point
%           goal            1 x 2 array [x y], for goal point
%       OUTPUTS:
%           waypoints       n x 2 array, for a series of waypoints (n waypoints) defining a 
%                           collision-free path from start to goal  
% 
%%  Helper Function isCollisionFree.m is required for this function to run
% 
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A

% Parameters for RRT
maxIterations = 2000;    % Maximum number of iterations
stepSize = 5;            % Step size for tree extension 
goalThreshold = 5;       % If the new node is within this distance of the goal, connect to goal

% Load obstacle data
obstacleData = load(map);
numObs = size(obstacleData, 1);
obstacles = cell(numObs, 1);
for i = 1:numObs
    obsRow = obstacleData(i,:);
    obsRow(obsRow==0) = [];  % Remove zeros
    obstacles{i} = reshape(obsRow, 2, [])';   % Each obstacle is an N x 2 matrix of vertices
end

% Define workspace boundaries
x_min = mapBoundary(1); y_min = mapBoundary(2);
x_max = mapBoundary(3); y_max = mapBoundary(4);

% Initialize the RRT tree as a struct array
% Each node has: pos (1x2) and parent
tree(1).pos = start;
tree(1).parent = 0;

% Optional: For visualization, store all edges (for plotting the tree)
treeEdges = []; % each row will be [parent_index, child_index]

% RRT Algorithm
goalReached = false;
goalIndex = -1;

for iter = 1:maxIterations
    % Sample a random point in the workspace
    randX = x_min + (x_max - x_min)*rand;
    randY = y_min + (y_max - y_min)*rand;
    q_rand = [randX, randY];
    
    % Find the nearest node in the tree to q_rand
    dists = arrayfun(@(node) norm(node.pos - q_rand), tree);
    [~, nearestIdx] = min(dists);
    q_near = tree(nearestIdx).pos;
    
    % Compute new point in direction from q_near to q_rand, stepping by stepSize
    direction = q_rand - q_near;
    if norm(direction) == 0
        continue; % avoid division by zero
    end
    q_new = q_near + stepSize * (direction / norm(direction));
    
    % Make sure q_new is within boundaries
    q_new(1) = min(max(q_new(1), x_min), x_max);
    q_new(2) = min(max(q_new(2), y_min), y_max);
    
    % Check if the path from q_near to q_new is collision free
    if isCollisionFree(q_near, q_new, obstacles)
        % Add q_new to the tree
        newIdx = length(tree) + 1;
        tree(newIdx).pos = q_new;
        tree(newIdx).parent = nearestIdx;
        treeEdges = [treeEdges; nearestIdx, newIdx]; %#ok<AGROW>
        
        % Check if goal is reachable from q_new (direct line collision-free) OR q_new is close to goal
        if (norm(q_new - goal) <= goalThreshold) || isCollisionFree(q_new, goal, obstacles)
            % Add goal to the tree and finish
            goalIdx = length(tree) + 1;
            tree(goalIdx).pos = goal;
            tree(goalIdx).parent = newIdx;
            treeEdges = [treeEdges; newIdx, goalIdx]; %#ok<AGROW>
            goalReached = true;
            break;
        end
    end
end

if ~goalReached
    warning('Goal not reached within maximum iterations.');
    waypoints_startTOgoal = [];
    return;
end

% Extract path from goal to start by backtracking
pathIdx = goalIdx;
pathPositions = [];
while pathIdx > 0
    pathPositions = [tree(pathIdx).pos; pathPositions];  %#ok<AGROW>
    pathIdx = tree(pathIdx).parent;
end
waypoints_startTOgoal = pathPositions;


if nargout == 0
    figure;
    hold on; axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); title('RRT: Tree and Final Path');
    % Plot workspace boundaries
    rectangle('Position', [x_min, y_min, (x_max-x_min), (y_max-y_min)], 'EdgeColor', 'k', 'LineWidth', 2);
    % Plot obstacles
    for i = 1:numObs
        obs = obstacles{i};
        obsPoly = [obs; obs(1,:)];  % close the polygon
        fill(obsPoly(:,1), obsPoly(:,2), 'r', 'FaceAlpha', 0.8, 'EdgeColor', 'r');
    end
    % Plot tree edges
    for i = 1:size(treeEdges,1)
        p1 = tree(treeEdges(i,1)).pos;
        p2 = tree(treeEdges(i,2)).pos;
        plot([p1(1) p2(1)], [p1(2) p2(2)], 'g-');
    end
    % Plot all nodes
    treePoints = cell2mat({tree.pos}');
    plot(treePoints(:,1), treePoints(:,2), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
    % Highlight start and goal
    plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    % Plot final path
    plot(waypoints_startTOgoal(:,1), waypoints_startTOgoal(:,2), 'b-', 'LineWidth', 3);
    legend('Workspace','Obstacles','Tree Edges','Tree Nodes','Start','Goal','Final Path');
    hold off;
end

end
