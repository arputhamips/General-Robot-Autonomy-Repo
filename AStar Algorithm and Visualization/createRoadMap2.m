% function [V, E] = createRoadMap2(obstacles)
function [V,E] = createRoadMap2(wall_vertices_list, union_obstacle, waypoints)
% Generates a visibility roadmap connecting wall vertices and waypoints.
% Inputs:
%   wall_vertices_list: each cell in the list is one polyshape(unioned
%   wall)'s vertices
%   union_obstacke: polyshapes of obstacles, obstacles are unioned
%   waypoints: W by 2

% Outputs:
%   V: Nx2 matrix of vertices [x, y]
%   E: Mx4 matrix of edges [x1, y1, x2, y2]

% Read obstacle data and initialize

ob_num = length(wall_vertices_list);
V = waypoints; % initialize V to be waypoints list
wall_V = []; % initialize wall_V for walls vertex
wall_E = []; % edge for walls edges 
E = [];


obstaclesVertices = cell(ob_num,1);
obstacleEdges = cell(ob_num, 1);

% ==========================================================
% ==== Concantenate all vertices and waypoints together ====
% ==========================================================
for i = 1:ob_num
    vertices = wall_vertices_list{i};
    
    if size(vertices,1) ~= 0

        wall_V = [wall_V;vertices];  
    
        wall_E_single = zeros(size(vertices,1), 4);
        for k = 1:size(vertices,1)-1
            wall_E_single(k,:) = [vertices(k,:), vertices(k+1,:)];
        end
        wall_E_single(end,:) = [vertices(end,:), vertices(1,:)];
        wall_E = [wall_E;wall_E_single];

    end

end

V = [V; wall_V]; % put waypoint and wall vertex together
E = [E;wall_E];

% ==========================================================
% ==== Connect Edges and check the validity ====
% ==========================================================
ver_num = size(V,1);
waypt_ver_num = size(waypoints,1);
wall_ver_num = ver_num - waypt_ver_num;

% Step 0: Wall edges stored in wall_E
% wall_E = zeros(wall_ver_num, 4);
% for i = 1:wal_ver_num-1
%     wall_E(i,:) = [wall_V(i,:), wall_V(i+1,:)];
% end
% wall_E(end,:) = [wall_V(end,:), wall_V(1,:)];
% E = [E;wall_E];

% Step 1: connect waypoints to all other points
for a = 1: waypt_ver_num 
    p1 = V(a,:);

    for b= a+1:ver_num 
        p2 = V(b,:);

        % ========= Valid checking block ============
        valid = true;
        for i = 1:wall_ver_num
            edge = wall_E(i,:);
            isIntersecting = intersectCheck(p1,p2,edge);
            if isIntersecting == true
                valid = false;
                break;
            end
        end       
        % ========= END: Valid checking block ============
        
        % if valid, put edge into E
        if(valid)
            E = [E; [p1,p2]];
        end

    end
end

% Step 2: connect wall edges and non-neighbor wall 
for a = waypt_ver_num+1: ver_num

    p1 = V(a,:);

    for b = a+2: ver_num

        p2 = V(b,:);

        if norm(p2-p1) > 0.5* (max(V(:,1)) - min(V(:,1)))
            continue;
        end

        mid = (p1+p2)/2;
        if isinterior(union_obstacle, mid(1), mid(2))
            continue;
        end

        % ========= Valid checking block ============
        valid = true;
        for i = 1:wall_ver_num
            isIntersecting = intersectCheck(p1,p2, wall_E(i,:));
            if isIntersecting == true
                valid = false;
                break;
            end
        end       
        % ========= END: Valid checking block ============

        % if valid, put edge into E
        if(valid)
            E = [E; [p1,p2]];
        end
    end

end

end % function end

function isIntersecting = intersectCheck(A, B, edge)
    C = [edge(1),edge(2)];
    D = [edge(3), edge(4)];

    cross1 = (A(1) - C(1)) * (D(2) - C(2)) - (A(2) - C(2)) * (D(1) - C(1)); % A -> C 和 D -> C 的叉积
    cross2 = (B(1) - C(1)) * (D(2) - C(2)) - (B(2) - C(2)) * (D(1) - C(1)); % B -> C 和 D -> C 的叉积
    cross3 = (C(1) - A(1)) * (B(2) - A(2)) - (C(2) - A(2)) * (B(1) - A(1)); % C -> A 和 B -> A 的叉积
    cross4 = (D(1) - A(1)) * (B(2) - A(2)) - (D(2) - A(2)) * (B(1) - A(1)); % D -> A 和 B -> A 的叉积
    
    
    isIntersecting = (cross1 * cross2 < 0) && (cross3 * cross4 < 0);
end