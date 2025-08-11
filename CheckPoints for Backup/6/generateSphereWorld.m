function sphereWorld = generateSphereWorld(walls)
% generateSphereWorld: Generate spheres from walls array
%
%   INPUT:
%       walls - (n x 4) numeric array [x1 y1 x2 y2]
%   OUTPUT:
%       sphereWorld - (m x 3) matrix [x_center y_center radius]
% 
% Autonomous Mobile Robots - HW6
% NIRMAL A J L A


% Group walls into boxes
nBoxes = size(walls,1) / 4;
sphereWorld = zeros(nBoxes, 3); % [x_center, y_center, radius]

for i = 1:nBoxes
    boxWalls = walls((i-1)*4+1:i*4, :); % Get 4 walls for this box
    xs = [boxWalls(:,1); boxWalls(:,3)];
    ys = [boxWalls(:,2); boxWalls(:,4)];
    
    xmin = min(xs);
    xmax = max(xs);
    ymin = min(ys);
    ymax = max(ys);
    
    % Center
    centerX = (xmin + xmax) / 2;
    centerY = (ymin + ymax) / 2;
    
    % Radius (half the diagonal)
    diagLength = sqrt((xmax - xmin)^2 + (ymax - ymin)^2);
    radius = diagLength / 2;
    
    sphereWorld(i,:) = [centerX, centerY, radius];
end

end