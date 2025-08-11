function neighbors = findNeighbors(V, E)
% This function is to help find the neighbors of each vertices in V
% INPUTS:
    % V: graph V
    % E: graph E
% OUTPUTS:
    % neighbors: each vertex has one cell of neighbors, neighbors are
    % represented by their **coordinates**

% Construct a map(dictionary) to map vertex coordinate and vertex idx
numV = size(V,1);
coordMap = containers.Map();
for i = 1:size(V,1)
    key = sprintf('%.5f,%.5f', V(i,1), V(i,2)); % key is a string
    coordMap(key) = i;
end

% Eac vertex has a list of neighbors
neighbors = cell(numV, 1);

for i = 1:size(E,1)
    % get the edge endpt coordinate and idx in V
    p1 = E(i,1:2);
    p1_key = sprintf('%.5f,%.5f', p1(1), p1(2));    
    p1_idx = coordMap(p1_key);
    p2 = E(i,3:4);
    p2_key = sprintf('%.5f,%.5f', p2(1), p2(2));    
    p2_idx = coordMap(p2_key);


    neighbors{p1_idx} = [neighbors{p1_idx}; p2];
    neighbors{p2_idx} = [neighbors{p2_idx}; p1];
end

end