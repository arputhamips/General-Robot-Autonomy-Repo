function schedulePath = runRoadmap2(start, walls, wayPt)
% This function is main motion plannig function, it integrates:
%   inflate walls, construct reduced visibility graph, A* for each
%   waypoints pair, TSP by total cost

% INPUT:
    % start: after PF localization for the inital position, the path starts
    % from here.
    % walls: map information
    % wayPt: target wayPt we want to visit

% load("practicemap2025update.mat");
% 
% walls = [map; optWalls(1,:)];            % combine real map and optWalls
% wayPt = [waypoints; ECwaypoints];   % combine two types of waypts

% inflate walls and create V, coordMap, E and neighbors
% Map plot is inside functions
[concex_vertices_list, union_obstacle] = createInflatedMap(walls, 0.1,0.16, 0.08);   % Inflate walls, 0.1-wall width, 0.16-robotR, 0.0-safe distance
[V,E] = createRoadMap2(concex_vertices_list, union_obstacle, wayPt);                % Construct visibility graph, including all verteices and waypoints
                                                                                    %   The E only contains wayPt - wayPt, wayPt- wall verteuces

neighbors = findNeighbors(V, E);    % Find the nerghbors for each vertices for A* path planning
coordMap = containers.Map();        % Construct a map to map vertex coordinate(key) to vertex index in V(value)
for i = 1:size(V,1)
    key = sprintf('%.5f,%.5f', V(i,1), V(i,2)); % key is a string
    coordMap(key) = i;
end

startKey = sprintf('%.5f,%.5f', start(1), start(2)); % key is a string
startIdx = coordMap(startKey);

% Plot vertices
scatter(V(:,1), V(:,2));

% Plot edges
for i=1:size(E,1)
    edge = E(i,:);
    h_edge = plotEdge([edge(1),edge(2)], [edge(3),edge(4)]);
end

% Plot waypoints
for i = 1:length(wayPt)
    text(wayPt(i,1), wayPt(i,2), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
end
scatter(wayPt(:,1), wayPt(:,2), 'yellow');

%% ===================== Signal Path: Go from one Pt to another PT using A* =============
% initial = waypoints(1,:);
% goal = ECwaypoints(1,:);
% [path, backPath, targetG] = Astar(V,neighbors,initial, goal);

% h_initial = scatter(initial(1), initial(2),200, 'o','Color','orange', "DisplayName","Initial Point");
% h_end = scatter(goal(1), goal(2),200,'go',"DisplayName","Goal Point");
% h_path = plot(path(:,1), path(:,2),'b--','LineWidth', 2,"DisplayName","Shortest Path");
% legend_handles = [h_initial, h_end, h_path];
% legend(legend_handles);
%% ======================================================================================

%% ======================= All paths to visit waypoints, cloest greedy PST ==============
[costMatrix, path_cells] = allPathAstar(V,coordMap, neighbors, wayPt); % cost matrix is N BY N (inf if not possible)
% startIdx = 1;
% tours = closestGreedyTSP(costMatrix, startIdx);                        % sub-optimal tours
tours = totalGreedyTSP(costMatrix, startIdx);
num_tourEle = size(tours,2);

schedulePath = [V(startIdx,:)];
for i = 1:num_tourEle-1
    startIdx = tours(i);
    goalIdx = tours(i+1);

    tempPath = path_cells{startIdx,goalIdx};
    tempPath(1, :) = [];   % remove the start pt coordinates

    schedulePath = [schedulePath; tempPath];

end

h_path = plot(schedulePath(:,1), schedulePath(:,2),'b--','LineWidth', 2,"DisplayName","Shortest Path");
legend_handles = [h_path];
legend(legend_handles);
%% ======================================================================================


end

