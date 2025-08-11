function inflatedMap = inflateInnerWalls(map, inflateAmount)
    % Compute lengths of all walls
    lengths = sqrt((map(:,3)-map(:,1)).^2 + (map(:,4)-map(:,2)).^2);
    
    % Identify 4 longest walls (outer boundaries)
    [~, sortedIdx] = sort(lengths, 'descend');
    boundaryIdx = sortedIdx(1:4);
    
    % Exclude boundary walls
    innerMap = map;
    innerMap(boundaryIdx, :) = [];
    
    % Initialize output
    inflatedMap = [];
    
    for i = 1:size(innerMap, 1)
        x1 = innerMap(i,1); y1 = innerMap(i,2);
        x2 = innerMap(i,3); y2 = innerMap(i,4);
        
        % Direction vector (tangential)
        dx = x2 - x1;
        dy = y2 - y1;
        len = sqrt(dx^2 + dy^2);
        tx = dx / len;
        ty = dy / len;
        
        % Normal vector
        nx = -ty;
        ny = tx;
        
        % Half offset
        offset = inflateAmount / 2;

        % Corner points with both tangential and normal offset
        p1 = [x1 - offset*tx + offset*nx, y1 - offset*ty + offset*ny];
        p2 = [x2 + offset*tx + offset*nx, y2 + offset*ty + offset*ny];
        p3 = [x2 + offset*tx - offset*nx, y2 + offset*ty - offset*ny];
        p4 = [x1 - offset*tx - offset*nx, y1 - offset*ty - offset*ny];

        % Represent rectangle as 4 segments
        inflatedMap = [inflatedMap;
            p1, p2;
            p2, p3;
            p3, p4;
            p4, p1];
    end
end