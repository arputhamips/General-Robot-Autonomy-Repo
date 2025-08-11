function likelihood = depthMeasurementLikelihood(particle, depthReadings, baseMap, optWalls, optState)
%DEPTHMEASUREMENTLIKELIHOOD  Compute likelihood of a depth frame given a particle.
%
%   likelihood = depthMeasurementLikelihood(particle, depthReadings, baseMap, optWalls, optState)
%
%   INPUTS
%     particle      : struct with fields
%                       x     – robot x position (m)
%                       y     – robot y position (m)
%                       theta – robot heading (rad)
%     depthReadings : 1×N vector of depth measurements (m)
%     baseMap       : M×4 array of known walls [x1 y1 x2 y2]
%     optWalls      : K×4 array of optional walls [x1 y1 x2 y2]
%     optState      : K×1 array of {–1,0,1} indicating absent/unknown/present
%
%   OUTPUT
%     likelihood    : scalar probability p(z | particle, map)

    % --- 1. Build the map hypothesis for this particle ---------------
    thisMap = baseMap;
    presentIdx = find(optState == 1);
    if ~isempty(presentIdx)
        thisMap = [ thisMap; optWalls(presentIdx, :) ];
    end

    % --- 2. Sensor & noise parameters --------------------------------
    sigmaRange = 0.03;          % depth noise standard deviation (m)
    minRange   = 0.175;         % minimum valid range (m)
    maxRange   = 10.0;          % maximum sensor range (m)
    FOV        = 54 * pi/180;   % field of view (rad)
    Nbeams     = numel(depthReadings);

    % --- 3. Precompute angles -----------------------------------------
    startAng = -FOV/2;
    if Nbeams > 1
        stepAng = FOV / (Nbeams - 1);
    else
        stepAng = 0;
    end

    % --- 4. Accumulate likelihood in linear space --------------------
    likelihood = 1.0;

    for i = 1:Nbeams
        meas = depthReadings(i);
        % skip out-of-range readings
        if meas < minRange || meas > maxRange
            continue;
        end

        % global beam angle
        angRel   = startAng + (i-1)*stepAng;
        beamAng  = wrapToPi(particle.theta + angRel);

        % expected radial distance along this beam
        expRadial = findDist( ... 
            particle.x, particle.y, thisMap, maxRange, beamAng);

        % project expected distance onto forward axis
        expDepth = expRadial * cos(angRel);

        % Gaussian probability of the measurement
        err = meas - expDepth;
        pz  = exp(-0.5 * (err/sigmaRange)^2) / (sqrt(2*pi)*sigmaRange);

        % update total likelihood
        likelihood = likelihood * pz;
    end
end

% --- Helper: findDist -----------------------------------------------
function d = findDist(x0, y0, mapSegments, maxRange, theta)
% Ray-cast from (x0,y0) at angle theta against each segment in mapSegments.
% Returns the smallest positive intersection distance ≤ maxRange.
    d = maxRange;
    for j = 1:size(mapSegments,1)
        x1 = mapSegments(j,1); y1 = mapSegments(j,2);
        x2 = mapSegments(j,3); y2 = mapSegments(j,4);
        [hit, ix, iy, dist] = lineIntersection(x0, y0, ...
            x0+maxRange*cos(theta), y0+maxRange*sin(theta), ...
            x1, y1, x2, y2);
        if hit && dist < d
            d = dist;
        end
    end
end

% --- Helper: lineIntersection ---------------------------------------
function [intersects, px, py, dist] = lineIntersection(x1,y1,x2,y2, x3,y3,x4,y4)
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if abs(den) < eps
        intersects=false; px=NaN; py=NaN; dist=Inf; return;
    end
    t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4)) / den;
    u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3)) / den;
    intersects = (t>=0 && t<=1 && u>=0 && u<=1);
    if intersects
        px = x1 + t*(x2-x1);
        py = y1 + t*(y2-y1);
        dist = hypot(px - x1, py - y1);
    else
        px=NaN; py=NaN; dist=Inf;
    end
end
