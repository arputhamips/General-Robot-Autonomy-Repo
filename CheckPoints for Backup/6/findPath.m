function [fullPath, totalCost] = findPath(roadNodes, roadEdges, init, goal)
% findPath - Find the shortest path in a roadmap connecting the initial and goal points.
%
%   INPUTS:
%       roadNodes - n x 2 matrix; each row is a node's [x, y] coordinate.
%       roadEdges - m x 2 matrix; each row defines an edge as [node_i, node_j].
%       init      - 1 x 2 vector; initial point [x, y].
%       goal      - 1 x 2 vector; goal point [x, y].
%
%   OUTPUTS:
%       fullPath  - k x 2 matrix; the full path from init to goal, including the endpoints.
%       totalCost - scalar; total cost (sum of Euclidean distances) along the path.
%
%   AUTONOMOUS MOBILE ROBOTS - HW6
%   NIRMAL, A J L A

nNodes = size(roadNodes, 1);

% Find the nearest roadmap node for the initial point
dInit = sqrt(sum((roadNodes - init).^2, 2));
[~, initIdx] = min(dInit);

% Find the nearest roadmap node for the goal point
dGoal = sqrt(sum((roadNodes - goal).^2, 2));
[~, goalIdx] = min(dGoal);

% Build the weighted graph: weight for each edge is Euclidean distance between its nodes
numEdges = size(roadEdges, 1);
s = roadEdges(:,1);
t = roadEdges(:,2);
weights = zeros(numEdges, 1);
for i = 1:numEdges
    pt1 = roadNodes(roadEdges(i,1), :);
    pt2 = roadNodes(roadEdges(i,2), :);
    weights(i) = norm(pt1 - pt2);
end

G = graph(s, t, weights, nNodes);

% Compute the shortest path between the closest nodes for init and goal using Dijkstra's algorithm
[pathIdx, totalCost] = shortestpath(G, initIdx, goalIdx);

% Construct the full path: start with init, then the roadmap nodes, then goal
fullPath = [init; roadNodes(pathIdx, :); goal];

end