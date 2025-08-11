function [nodes, edges, cells] = createRoadmap(obstacles)
% createRoadmap - Generate a roadmap of free space via vertical (trapezoidal) decomposition.
%
%   INPUT:
%       obstacles - matrix from hw6b.txt (each row has 16 entries: 
%                   [v1x, v1y, v2x, v2y, ..., v8x, v8y]; unused entries are 0)
%
%   OUTPUTS:
%       nodes   - n x 2 matrix; each row is the center (x,y) of a free-space cell.
%       edges   - m x 2 matrix; each row is an edge between nodes (node indices).
%       cells   - k x 4 matrix; each row gives the boundaries of a free-space cell 
%                 [x_left, x_right, y_bottom, y_top].
%
% 
%
% Data Structure:
%   - cells: Each row is [x_left, x_right, y_bottom, y_top].
%   - nodes: Centers of these cells.
%   - edges: Connections between nodes in adjacent slabs.
%
%
% BASED ON LOGIC FROM, THANKS TO...
% @title: trapezoidal_decomposition.py
% @author: Terrance Williams
% @date: 22 November 2023
% https://github.com/tjdwill/TrapezoidalDecomposition/blob/main/trapezoidal_decomposition.py


% AMR - HW6
% NIRMAL, A J L A

% Workspace boundaries
MIN_X = 0; MAX_X = 100;
MIN_Y = 0; MAX_Y = 100;

% Number of obstacles
numObs = size(obstacles, 1);
polygons = cell(numObs, 1);
allX = [];

for i = 1:numObs
    obs = obstacles(i,:);
    obs(obs==0) = [];
    pts = reshape(obs, 2, [])';
    polygons{i} = pts;
    allX = [allX; pts(:,1)];
end

% Vertical boundaries for decomposition: add workspace borders
xEdges = unique([MIN_X; allX; MAX_X]);

cells = [];         % To store each cell as [x_left, x_right, y_bottom, y_top]
nodeCenters = [];   % To store centers of cells (nodes)
cell_node_indices = cell(length(xEdges)-1,1);  % To store node indices per vertical slab
node_count = 0;

% For each vertical slab
for i = 1:length(xEdges)-1
    x_left = xEdges(i);
    x_right = xEdges(i+1);
    % Work with a representative vertical line (e.g., at the slab's midpoint)
    x_mid = (x_left + x_right)/2;
    
    % Start with full vertical interval
    freeIntervals = [MIN_Y, MAX_Y];  % one interval: [0, 100]
    
    % For each obstacle, if it spans this vertical slab, subtract its y-range
    for j = 1:numObs
        pts = polygons{j};
        if (min(pts(:,1)) < x_right) && (max(pts(:,1)) > x_left)
            % Approximate the vertical blockage by the obstacle's y extent
            y_block = [min(pts(:,2)), max(pts(:,2))];
            freeIntervals = subtractInterval(freeIntervals, y_block);
        end
    end
    
    % freeIntervals is now an N x 2 matrix of free y-intervals in the slab
    for k = 1:size(freeIntervals,1)
        y_bottom = freeIntervals(k,1);
        y_top = freeIntervals(k,2);
        cells = [cells; x_left, x_right, y_bottom, y_top];  %#ok<AGROW>
        node_center = [(x_left+x_right)/2, (y_bottom+y_top)/2];
        node_count = node_count+1;
        nodeCenters(node_count,:) = node_center;  %#ok<AGROW>
        cell_node_indices{i}(end+1) = node_count; %#ok<AGROW>
    end
end

nodes = nodeCenters;

% Create edges between cells in adjacent slabs: if their vertical intervals overlap, connect nodes.
edges = [];
for i = 1:length(cell_node_indices)-1
    if isempty(cell_node_indices{i}) || isempty(cell_node_indices{i+1})
        continue;
    end
    % For each cell in slab i:
    for a = 1:length(cell_node_indices{i})
        idx_a = cell_node_indices{i}(a);
        cell_a = cells(idx_a,:);  % [x_left, x_right, y_bottom, y_top]
        % For each cell in slab i+1:
        for b = 1:length(cell_node_indices{i+1})
            idx_b = cell_node_indices{i+1}(b);
            cell_b = cells(idx_b,:);
            % They are adjacent if the vertical intervals overlap.
            overlap = min(cell_a(4), cell_b(4)) - max(cell_a(3), cell_b(3));
            if overlap > 0  % positive overlap
                edges = [edges; idx_a, idx_b]; %#ok<AGROW>
            end
        end
    end
end

end

%% Helper functions
function outIntervals = subtractInterval(inIntervals, subInterval)
% Subtract subInterval [s, e] from a set of intervals in inIntervals.
% inIntervals is an N x 2 matrix where each row is [a, b]
% Returns a new set of intervals that do not include the subInterval.
outIntervals = [];
for i = 1:size(inIntervals,1)
    a = inIntervals(i,1);
    b = inIntervals(i,2);
    s = subInterval(1);
    e = subInterval(2);
    % If no overlap, keep the original interval.
    if e <= a || s >= b
        outIntervals = [outIntervals; a, b]; %#ok<AGROW>
    else
        % There is overlap. Subtract the overlapping region.
        if s > a
            outIntervals = [outIntervals; a, s]; %#ok<AGROW>
        end
        if e < b
            outIntervals = [outIntervals; e, b]; %#ok<AGROW>
        end
    end
end
end