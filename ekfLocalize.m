function dataStore = ekfLocalize(dataStore, Robot, map, beaconLoc, optWalls, waypoints)

% ---------- parameters ----------
sigma_odometry = [0.01 0.01 0.02];  % [σx σy σθ]
sigma_beacon   = 0.2;               % beacon position std dev (m)
sigma_depth    = 0.03;              % depth noise (m)
depthFOV       = 54 * pi/180;       % RealSense FOV
depthBeams     = 9;
maxDepth       = 2.5;
sensorOffset   = 0.08;
optWallThresh  = 0.6;

persistent X P

% ---------- initialize ----------
if isempty(X)
    wp = waypoints(randi(size(waypoints,1)),:);
    X = [wp(1); wp(2); 2*pi*rand - pi];
    P = diag([0.2 0.2 pi/8].^2);
end

% ---------- odometry update ----------
if isempty(dataStore.odometry), return; end
odo = dataStore.odometry(end,2:3); d = odo(1); dth = odo(2);
th = X(3);
X(1) = X(1) + d*cos(th);
X(2) = X(2) + d*sin(th);
X(3) = wrapToPi(X(3) + dth);
G = [1 0 -d*sin(th); 0 1 d*cos(th); 0 0 1];
Q = diag(sigma_odometry.^2);
P = G*P*G' + Q;

% ---------- beacon update ----------
[Ntag, Z, Xr, ROT] = RealSenseTag(Robot);
if ~isempty(Ntag)
    id = Ntag(1); dxR = Z(1); dyR = Xr(1);
    idx = find(beaconLoc(:,1)==id,1);
    if ~isempty(idx)
        bx = beaconLoc(idx,2); by = beaconLoc(idx,3);
        c = cos(X(3)); s = sin(X(3));
        cx = X(1) + c*dxR - s*dyR;
        cy = X(2) + s*dxR + c*dyR;
        z = [bx; by];
        h = [cx; cy];
        H = [1 0 -s*dxR - c*dyR; 0 1 c*dxR - s*dyR];
        R = sigma_beacon^2 * eye(2);
        S = H*P*H' + R;
        K = P*H'/S;
        X = X + K*(z - h);
        P = (eye(3) - K*H)*P;
    end
end

% ---------- optional wall update ----------
if isfield(dataStore,'rsdepth') && ~isempty(dataStore.rsdepth)
    latestDepth = dataStore.rsdepth(end,3:end);
    numRays = length(latestDepth);
    angle0 = -depthFOV/2;
    angleStep = depthFOV / (depthBeams - 1);
    particleMap = map;
    for w = 1:size(optWalls,1)
        if isfield(dataStore,'optWallBelief') && dataStore.optWallBelief(w) == 1
            particleMap = [particleMap; optWalls(w,:)];
        end
    end
    inflatedMap = inflateWalls(particleMap, 0.1016);
    if ~isfield(dataStore,'optWallBelief')
        dataStore.optWallBelief = zeros(size(optWalls,1),1);
    end
    for w = 1:size(optWalls,1)
        if dataStore.optWallBelief(w) ~= 0, continue; end
        hits = 0; miss = 0;
        for i = 1:numRays
            angle = X(3) + angle0 + (i-1)*angleStep;
            sx = X(1) + sensorOffset*cos(X(3));
            sy = X(2) + sensorOffset*sin(X(3));
            [intersect,~,~,dist] = lineIntersection(...
                sx, sy, sx+maxDepth*cos(angle), sy+maxDepth*sin(angle), ...
                optWalls(w,1), optWalls(w,2), optWalls(w,3), optWalls(w,4));
            if intersect && abs(latestDepth(i)-dist)<3*sigma_depth
                hits = hits + 1;
            else
                miss = miss + 1;
            end
        end
        if hits > 2, dataStore.optWallBelief(w) = 1;
        elseif miss > 3, dataStore.optWallBelief(w) = -1;
        end
    end
end

% ---------- log pose ----------
tNow = toc;
dataStore.truthPose(end+1,:) = [tNow X.'];
end

function A = inflateWalls(map, inflateAmount)
A = [];
for i = 1:size(map,1)
    x1 = map(i,1); y1 = map(i,2);
    x2 = map(i,3); y2 = map(i,4);
    len = hypot(x2-x1, y2-y1);
    if len < 2.5
        dx = (y2 - y1)/len * (inflateAmount/2);
        dy = (x1 - x2)/len * (inflateAmount/2);
        A = [A;
            x1-dx y1-dy x2-dx y2-dy;
            x1+dx y1+dy x2+dx y2+dy;
            x1-dx y1-dy x1+dx y1+dy;
            x2-dx y2-dy x2+dx y2+dy];
    end
end
end

function [intersects, px, py, dist] = lineIntersection(x1,y1,x2,y2,x3,y3,x4,y4)
den = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
if den==0
    intersects = false; px=NaN; py=NaN; dist=Inf; return;
end
t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/den;
u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/den;
intersects = t>=0 && t<=1 && u>=0 && u<=1;
if intersects
    px = x1 + t*(x2-x1);
    py = y1 + t*(y2-y1);
    dist = hypot(px - x1, py - y1);
else
    px=NaN; py=NaN; dist=Inf;
end
end
