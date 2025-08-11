function [costMatrix, path_cells] = allPathAstar(V,coordMap, neighbors, wayPt)

num_wayPt = size(wayPt,1);
costMatrix = inf(num_wayPt,num_wayPt);
path_cells = cell(num_wayPt, num_wayPt);

% Get the shorest path from i to j, since we can get back path, we only run
% j = i+1:num_wayPt
for i = 1:num_wayPt
    for j = i+1:num_wayPt
        [path, backPath, targetG] = Astar(V,coordMap, neighbors,wayPt(i,:), wayPt(j,:));
        path_cells{i,j} = path;
        path_cells{j,i} = backPath;
        costMatrix(i,j)= targetG;
        costMatrix(j,i) = targetG;
    end
end


% to fit in the TSP function, let all inf to be 0
% costMatrix(isinf(costMatrix)) = 0;

end