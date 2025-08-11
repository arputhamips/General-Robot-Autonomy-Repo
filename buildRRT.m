function [waypoints_startTOgoal] = buildRRT(map, mapBoundary, start, goal, robotRadius)
% BUILDRRT
%
%   INPUTS:
%       map         - An array representing the obstacles.
%                     Either:
%                        (a) an n x 4 matrix for walls [x1, y1, x2, y2] (each row a wall segment),
%                     or
%                        (b) an n x 16 matrix (each row with up to 8 vertices, zeros for unused entries)
%       mapBoundary - 1 x 4 vector [x_bl, y_bl, x_tr, y_tr] defining the workspace boundaries.
%       start       - 1 x 2 array, the starting position [x, y].
%       goal        - 1 x 2 array, the goal position [x, y].
%       robotRadius - Scalar representing the robot's radius (in m).
%
%   OUTPUTS:
%       waypoints_startTOgoal - An n x 2 matrix of waypoints (including start and goal)
%                               defining a collision-free path.
% 
% 
%%  Helper Function isCollisionFree.m is required for this function to run
% 
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A

global dataStore;

%% Parameters for RRT
maxIterations = 10000;
stepSize = .2;         % Step size for tree expansion (m)
goalThreshold = .1;    % Distance threshold (m) to connect directly to goal

%% Determine obstacle type and inflate obstacles
numCols = size(map,2);
inflatedObstacles = {};
if sum(map(1,:) ~= 0) == 4
    % Wall style: each row is [x1, y1, x2, y2]
    for i = 1:size(map,1)
        pt1 = map(i,1:2);
        pt2 = map(i,3:4);
        d = pt2 - pt1;
        len = norm(d);
        if len == 0
            % Degenerate wall => skip
            continue;
        end

        % Unit direction vector along the line
        d_norm = d / len;

        % Unit normal vector (perpendicular to the line)
        n_vec = [-d_norm(2), d_norm(1)];

        % 1) Extend the line segment by robotRadius along its length
        pt1_ext = pt1 - robotRadius * d_norm;  % Shift pt1 "backwards"
        pt2_ext = pt2 + robotRadius * d_norm;  % Shift pt2 "forwards"

        % 2) Offset the extended endpoints by ±robotRadius in the normal direction
        p1 = pt1_ext + robotRadius * n_vec;
        p2 = pt1_ext - robotRadius * n_vec;
        p3 = pt2_ext - robotRadius * n_vec;
        p4 = pt2_ext + robotRadius * n_vec;

        % Save the rectangle as a polygon
        inflatedObstacles{end+1} = [p1; p2; p3; p4];
    end
else
    % Polygon style: each row is a polygon with up to 8 vertices.
    for i = 1:size(map,1)
        obsRow = map(i,:);
        obsRow(obsRow==0) = []; % Remove zeros
        pts = reshape(obsRow, 2, [])';
        if size(pts,1) < 3
            continue;  % Not a valid polygon
        end
        polyObs = polyshape(pts(:,1), pts(:,2));
        inflatedPoly = polybuffer(polyObs, robotRadius);
        [vx, vy] = boundary(inflatedPoly);
        inflatedObstacles{end+1} = [vx, vy]; %#ok<AGROW>
    end
end

%% Workspace boundaries
x_min = mapBoundary(1); y_min = mapBoundary(2);
x_max = mapBoundary(3); y_max = mapBoundary(4);

%% Initialize RRT tree
% The tree is a struct array, each node with:
%   pos - 1 x 2 position
%   parent - index of the parent node (0 for root)
tree(1).pos = start;
tree(1).parent = 0;

goalReached = false;
goalIndex = -1;
treeEdges = [];  % For optional plotting

%% RRT Main Loop
for iter = 1:maxIterations
    % Sample random point in workspace
    q_rand = [ x_min + (x_max - x_min)*rand, y_min + (y_max - y_min)*rand ];
    
    % Find nearest node in the tree
    dists = arrayfun(@(node) norm(node.pos - q_rand), tree);
    [~, nearestIdx] = min(dists);
    q_near = tree(nearestIdx).pos;
    
    % Compute new point in direction from q_near to q_rand with stepSize
    direction = q_rand - q_near;
    if norm(direction) == 0, continue; end
    q_new = q_near + stepSize * (direction / norm(direction));
    % Clamp to workspace boundaries:
    q_new(1) = min(max(q_new(1), x_min), x_max);
    q_new(2) = min(max(q_new(2), y_min), y_max);
    
    % Collision check: test if segment from q_near to q_new is collision free.
    if isCollisionFree(q_near, q_new, inflatedObstacles)
        newIdx = length(tree) + 1;
        tree(newIdx).pos = q_new;
        tree(newIdx).parent = nearestIdx;
        treeEdges = [treeEdges; nearestIdx, newIdx]; %#ok<AGROW>
        
        % Check if we can connect directly to the goal.
        if (norm(q_new - goal) <= goalThreshold) || isCollisionFree(q_new, goal, inflatedObstacles)
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

%% Backtrack to get the path from start to goal
pathIdx = goalIdx;
pathPositions = [];
while pathIdx > 0
    pathPositions = [tree(pathIdx).pos; pathPositions]; %#ok<AGROW>
    pathIdx = tree(pathIdx).parent;
end
waypoints_startTOgoal = pathPositions;

dataStore.RRT.tree = tree;
dataStore.RRT.treeEdges = treeEdges;
dataStore.RRT.waypoints = waypoints_startTOgoal;

end
