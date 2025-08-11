function [nodes,edges] = buildPRM(workspace,mapBoundary,n)
% BUILDPRM
% AMR Homework 6 
%
%       INPUTS:
%           workspace         file name for text file representing the obstacles in the workspace
%                             for example workspace = 'hw6b.txt'. Each row in this file contains the vertices
%                             of one polygonal obstacle: v1x, v1y, v2x, v2y, etc. The vertices are given in
%                             counterclockwise order. If an obstacle has fewer vertices, unused entries 
%                             in the line will contain the value zero
%           mapBoundary       1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                             and top right corner of the workspace respectively  
%           n                 number of vertices in the roadmap     
%       OUTPUTS:
%           nodes           m x 2, nodes of the PRM
%           edges           n x 4 array, edges of the PRM 
%                           (each row contains the coordinates of the vertices forming the edge)
% Autonomous Mobile Robots
% NIRMAL A J L A

obsData = load(workspace);
numObs = size(obsData,1);
obstacles = cell(numObs,1);
for i = 1:numObs
    % Remove zeros; each obstacle row may contain up to 8 vertices.
    obsRow = obsData(i,:);
    obsRow(obsRow==0) = [];
    pts = reshape(obsRow, 2, [])';
    if size(pts,1) < 3
        % Skip if not a valid polygon
        continue;
    end
    obstacles{i} = pts;
end

%% Sampling Collision-Free Nodes
% Workspace boundaries
x_min = mapBoundary(1);
y_min = mapBoundary(2);
x_max = mapBoundary(3);
y_max = mapBoundary(4);

nodes = [];
maxSamples = 10*n;  % limit the number of random candidates to avoid infinite loops

sampleCount = 0;
while size(nodes,1) < n && sampleCount < maxSamples
    q = [ x_min + (x_max - x_min)*rand,  y_min + (y_max - y_min)*rand ];
    if isFree(q, obstacles)
        nodes = [nodes; q]; %#ok<AGROW>
    end
    sampleCount = sampleCount + 1;
end

if size(nodes,1) < n
    warning('Only %d collision-free samples found; expected %d.', size(nodes,1), n);
end

%% Construct PRM Edges
% Connect every pair of nodes if the straight-line segment connecting them is collision-free.
edges = [];  % will store rows as [x1, y1, x2, y2]
numNodes = size(nodes,1);
for i = 1:numNodes-1
    for j = i+1:numNodes
        if isCollisionFreeEdge(nodes(i,:), nodes(j,:), obstacles)
            edges = [edges; nodes(i,:), nodes(j,:)]; %#ok<AGROW>
        end
    end
end

end

%% Helper Function: Check if a point is in free space (not in any obstacle)
function free = isFree(q, obstacles)
free = true;
for k = 1:length(obstacles)
    pts = obstacles{k};
    if inpolygon(q(1), q(2), pts(:,1), pts(:,2))
        free = false;
        return;
    end
end
end

%% Helper Function: Check if a line segment is collision-free
function free = isCollisionFreeEdge(q1, q2, obstacles)
free = true;
lineX = [q1(1), q2(1)];
lineY = [q1(2), q2(2)];
for k = 1:length(obstacles)
    pts = obstacles{k};
    % Ensure the polygon is closed
    if ~isequal(pts(1,:), pts(end,:))
        pts = [pts; pts(1,:)];
    end
    [xi, yi] = polyxpoly(lineX, lineY, pts(:,1), pts(:,2));
    if ~isempty(xi)
        free = false;
        return;
    end
end
end

