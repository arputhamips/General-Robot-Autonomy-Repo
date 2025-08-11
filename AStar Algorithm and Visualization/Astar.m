function [path, backPath, targetG] = Astar(V,coordMap,neighbors,start, goal)

% This function is to find the shortest path from start vertex to the goal
% vertex. (One-pair shorest path)

% INPUTS:
    % V: list of vertices (including waypoints and wall verteices)
    % neighbors: cells for vertices' neighbors
    % start: start vertex position [x,y]
    % goal: goal vertex position [x,y]

% OUTPUTS:
    % path: list of points we need to visit to reach from start to goal

% ***CoordMap section has been moved into the main program for reusing
% Create a map (dictionary) map vertex position to vertex index in V
% This structure help to easy reference from vertex postion to its
% g/f/parent list.
% coordMap = containers.Map();
% for i = 1:size(V,1)
%     key = sprintf('%.5f,%.5f', V(i,1), V(i,2)); % key is a string
%     coordMap(key) = i;
% end

% Find start/goal indices
startKey = sprintf('%.5f,%.5f', start(1), start(2));
goalKey  = sprintf('%.5f,%.5f', goal(1), goal(2));
startIdx = coordMap(startKey);
goalIdx  = coordMap(goalKey);

% Initialize all verteices' g(x) f(x) and parent node
numNodes = size(V, 1);
gScore = inf(numNodes,1);
fScore = inf(numNodes,1);
prev = zeros(numNodes,1); % since MATLAB index starts from 1, use 0 as n/a

% Set start vertex g and f; h(x) is the L2-norm
gScore(startIdx) = 0;
fScore(startIdx) = gScore(startIdx) + norm(start - goal); 

% Open set: store all non-inf vertex infomation
openSet = [startIdx, fScore(startIdx)];

while ~isempty(openSet)
    % Pop node with lowest fScore, use this vertex to update g/f/parent
    [~, minIdx] = min(openSet(:,2));
    current = openSet(minIdx,1); % current is vertex **idx**
    openSet(minIdx,:) = []; % remove current vertex from open set

    if current == goalIdx
        targetG = gScore(current);
        break;
    end

    currentCoord = V(current,:); % currentCoord is vertex **coordinate**

    % Update current neighbors
    neighbor_cell = neighbors{current}; 

    for i = 1:size(neighbor_cell,1)
        neighborCoord = neighbor_cell(i,:);
        neighborKey = sprintf('%.5f,%.5f', neighborCoord(1), neighborCoord(2));
%         if ~isKey(coordMap, neighborKey) % I don't think we need to check
%         this??
%             continue;
%         end
        neighborIdx = coordMap(neighborKey);

        tentativeG = gScore(current) + norm(currentCoord - neighborCoord); % the cost of start to current and current to this neighbor node
        
        % If new g is smaller than previous one, we need to update g and
        % put it into openSet if it's not.
        if tentativeG < gScore(neighborIdx)
            prev(neighborIdx) = current;
            gScore(neighborIdx) = tentativeG;
            fScore(neighborIdx) = tentativeG + norm(neighborCoord - goal);
            
            if ~any(openSet(:,1) == neighborIdx)
                openSet = [openSet; neighborIdx, fScore(neighborIdx)];
            end
        end % end tentativeG < gScore(neighborIdx)
    end % end for (updtae all neighbors of current)
end % end while (go through all openset)

% Reconstruct path
path = [];
if prev(goalIdx) ~= 0
    idx = goalIdx;
    while idx ~= 0
        path = [V(idx,:); path];
        idx = prev(idx);
    end
end

backPath = flip(path);
targetG = gScore(goalIdx);
end