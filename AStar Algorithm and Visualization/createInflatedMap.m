function [convex_vertices_list, union_obstacle] = createInflatedMap(walls, wallWidth, robotRadius, safeDistance)
% wallWidth = 0.1m; robotRadius = 0.16m

    % INPUTS:
    %   walls: Nx4, [x1 y1 x2 y2]
    %   wall_width: scalar, the width of wall, when inclating, we need
    %   0.5*wallWidth;
    %   robotRaius: scalar, the radius of robot

    % OUTPUTS:
    %   convex_vertices_list: 
    %   union_obstacle: unioned inflated wall polyshape

    % inflate the wall line into a rectangle
    % --left-right need to expand for safe distance + robot radius
    % --up -down need to expand for 0.5*wallWidth + safe distance + robot radius
    
    poly_rects = [];

    min_x = min(min(walls(:,1)),min(walls(:,3)));
    max_x = max(max(walls(:,1)),max(walls(:,3)));

    min_y = min(min(walls(:,2)),min(walls(:,4)));
    max_y = max(max(walls(:,2)),max(walls(:,4)));
    
    % infalte each wall
    for i = 1:size(walls, 1)
        x1 = walls(i, 1); y1 = walls(i, 2);
        x2 = walls(i, 3); y2 = walls(i, 4);

        p1 = [x1,y1];  % wall line endpoint 1
        p2 = [x2,y2];  % wall line endpoint 2

        dir = p2-p1;  dir = dir/norm(dir); % normolize line direction

        
        p1 = p1+ (safeDistance+robotRadius)*(-dir);  
        p2 = p2 + (safeDistance+robotRadius)*(dir);
        perp = (0.5 * wallWidth +safeDistance+robotRadius) * [-dir(2), dir(1)];
      
        rect = polyshape([p1+perp; p2+perp; p2-perp; p1-perp]);
        poly_rects = [poly_rects; rect];

    end

    % union overlapped polygon
    if isempty(poly_rects)
        union_obstacle = polyshape();  % empty polygon
    else
        union_obstacle = poly_rects(1);
        for i = 2:length(poly_rects)
            union_obstacle = union(union_obstacle, poly_rects(i));
        end
    end


    % Get the vertices of each polygon, each cell of the list is vertices
    % for one polygon. The vertices are inside the boundary and except
    % unalignment vertices.
    convex_vertices_list= extractVerticesFromPolygons(union_obstacle,min_x, max_x, min_y, max_y);


    % Display inflated wall and original map
    figure; 
    hold on; axis equal;
    plot(union_obstacle, 'FaceColor', [0.4 0.7 0.4], 'FaceAlpha', 0.6);

    cornerMap = walls;
    for i = 1:size(cornerMap, 1)
        plot([cornerMap(i,1), cornerMap(i,3)], [cornerMap(i,2), cornerMap(i,4)], 'r-', 'LineWidth', 2,'HandleVisibility', 'off');
    end
    
    % Display vertices
   
    title('Unified Inflated Obstacles');

end

function vertices_list = extractVerticesFromPolygons(union_polyshape, min_x, max_x, min_y, max_y)
% This function will extract vertices from polygon list. vertices have been
% filtered by boundary and unalignment.

    % INPUTS:
    %   union_polyshape: after union, we only have one polyshap 
    % OUTPUTS:
    %   vertices_list: each cell is one obstacle's vertices, vertices are N
    %   by 2

    

    vertices_list = {};
    [x_all, y_all] = boundary(union_polyshape);
    coords = [x_all,y_all];

    % =========== split by nan =========
    nanRows = isnan(x_all);
    splitIdx = find(nanRows);
    splitIdx = [0; splitIdx; size(coords, 1)+1]; 
    
    vertices_list = {};  
    
    for i = 1:length(splitIdx)-1
        idxStart = splitIdx(i) + 1;
        idxEnd = splitIdx(i+1) - 1;
    
        if idxStart <= idxEnd
            vertices_list{end+1} = coords(idxStart:idxEnd, :);
        end
    end
    
    % ================== remove unnessaray vertices =====================
    for i =1 :size(vertices_list,2)
        x = vertices_list{i}(:,1); % ine one cell, get x, y
        y = vertices_list{i}(:,2);

        % remove the last vertices since polyShape will close
        if norm([x(1)-x(end), y(1)-y(end)]) < 1e-10
            x(end) = [];
            y(end) = [];
        end

        % find index of vetrices which are within the boundary
        inside_idx = (x >= min_x) & (x <= max_x) & (y >= min_y) & (y <= max_y);

        % filter vetices by boundary
        x_filtered = x(inside_idx);
        y_filtered = y(inside_idx);
        

        if size(x_filtered,1) == 0
            vertices_list{i} = [x_filtered,y_filtered];
        else
            % remove unalignment vertcie
            [x_reduced,y_reduced] = removeUnalign2(x_filtered, y_filtered);
            vertices_list{i} = [x_reduced, y_reduced]; % update vertices_list
        end
        
        
    end
    
end


function [x,y] = removeUnalign(x_filtered, y_filtered)
% This function is to remove extra points due to the unalignement of
% infalted walls. This function is naive and uses for loop --- slow.

% INPUTS:
    % x_filtered, y_filtered: vertices has been removed out of boundary and
    % all points are unique (no repeated)
% OUTPUTS:
    % x, y: x/y coordinate after removing unalignmnet vertices


% pad the vertices list so itertaion can check the first vertices
x_filtered = [x_filtered; x_filtered(1); x_filtered(2)];
y_filtered = [y_filtered; y_filtered(1); y_filtered(2)];

n = size(x_filtered);
x = [];
y = [];

for i = 1:n-2
    x_cur = x_filtered(i);
    x_nex = x_filtered(i+1);
    x_nexnex = x_filtered(i+2);

    y_cur = y_filtered(i);
    y_nex = y_filtered(i+1);
    y_nexnex = y_filtered(i+2);
    cur = [x_cur, y_cur];
    nex = [x_nex, y_nex];
    nexnex = [x_nexnex, y_nexnex];

    if norm(cur-nex) > 0.06 || norm(nex-nexnex) > 0.06
        x =  [x; x_nex]; 
        y =  [y; y_nex]; 

    end
end


end

function [x_out,y_out] = removeUnalign2(x_filtered, y_filtered)
% This function is to remove extra points due to the unalignement of
% infalted walls. This function is naive and uses vetcor --- quick.

% INPUTS:
    % x_filtered, y_filtered: vertices has been removed out of boundary and
    % all points are unique (no repeated)
% OUTPUTS:
    % x, y: x/y coordinate after removing unalignmnet vertices

% pad the vertices list so itertaion can check the first vertices
x_filtered = [x_filtered; x_filtered(1); x_filtered(2)];
y_filtered = [y_filtered; y_filtered(1); y_filtered(2)];

cur     = [x_filtered(1:end-2), y_filtered(1:end-2)];
nex     = [x_filtered(2:end-1), y_filtered(2:end-1)];
nexnex  = [x_filtered(3:end),   y_filtered(3:end)];

dist1 = vecnorm(cur - nex, 2, 2);     
dist2 = vecnorm(nex - nexnex, 2, 2);

valid_idx = (dist1 > 0.06) | (dist2 > 0.06);

x_out = nex(:,1);
y_out = nex(:,2);

x_out = x_out(valid_idx);
y_out = y_out(valid_idx);

end


