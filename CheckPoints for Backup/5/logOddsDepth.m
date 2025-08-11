function newLogOdds = logOddsDepth(prevLogOdds, robotPose, map, sensorOrigin, sensorAngles, mapResolution, gridOrigin, sensorModelParams)
% logOddsDepth updates the occupancy grid's log-odds using depth sensor measurements.
%
%   newLogOdds = logOddsDepth(prevLogOdds, robotPose, map, sensorOrigin, sensorAngles, mapResolution, gridOrigin, sensorModelParams)
%
% INPUTS:
%   prevLogOdds       - n-by-m matrix of current log-odds occupancy values.
%   robotPose         - 3x1 vector [x; y; theta] representing the robot's pose in global coordinates.
%   map               - N-by-4 matrix of wall endpoints [x1, y1, x2, y2].
%   sensorOrigin      - 1x2 vector [x y] representing the origin of the sensor in the robot frame.
%   sensorAngles      - Kx1 vector of beam angles (in radians) in the sensor-fixed frame (0 points forward).
%   mapResolution     - Scalar indicating the size (meters) of each grid cell.
%   gridOrigin        - 1x2 vector [x0, y0] of world coordinates corresponding to grid cell (1,1).
%   sensorModelParams - Struct with parameters for the depth sensor model, with fields:
%                         .depthHit      : log-odds increment for an occupied (hit) cell.
%                         .depthMiss     : log-odds decrement for a free (miss) cell.
%                         .depthMaxRange : Maximum reliable depth sensor range.
%
% OUTPUT:
%   newLogOdds        - Updated occupancy grid (n-by-m matrix) in log-odds.
%
%  AUTONOMOUS MOBILE ROBOTS HW5
%  CORNELL UNIVERSITY
%  NIRMAL , A J L A

    % Get predicted depth measurements using the provided depthPredict function
    depthSensorData = depthPredict(robotPose, map, sensorOrigin, sensorAngles);

    newLogOdds = prevLogOdds;
    [n, m] = size(prevLogOdds);
    
    % Number of beams
    K = length(depthSensorData);
    for beam = 1:K
        r = depthSensorData(beam);
        beamAngle = robotPose(3) + sensorAngles(beam);
        
        % Determine the endpoint for updating free space: if r is less than the max range,
        % use r; otherwise, use the sensor's maximum reliable range.
        if r < sensorModelParams.depthMaxRange
            r_end = r;
            % Mark the hit cell as occupied using depthHit increment
            hitX = robotPose(1) + r * cos(beamAngle);
            hitY = robotPose(2) + r * sin(beamAngle);
            colHit = floor((hitX - gridOrigin(1)) / mapResolution) + 1;
            rowHit = floor((hitY - gridOrigin(2)) / mapResolution) + 1;
            if rowHit >= 1 && rowHit <= n && colHit >= 1 && colHit <= m
                newLogOdds(rowHit, colHit) = newLogOdds(rowHit, colHit) + sensorModelParams.depthHit;
            end
        else
            r_end = sensorModelParams.depthMaxRange;
        end
        
        % Update cells along the beam as free using the depthMiss decrement
        stepSize = mapResolution / 2;  % Use a step smaller than a grid cell for smoother updates
        for r_free = 0:stepSize:(r_end - mapResolution)
            freeX = robotPose(1) + r_free * cos(beamAngle);
            freeY = robotPose(2) + r_free * sin(beamAngle);
            colFree = floor((freeX - gridOrigin(1)) / mapResolution) + 1;
            rowFree = floor((freeY - gridOrigin(2)) / mapResolution) + 1;
            if rowFree >= 1 && rowFree <= n && colFree >= 1 && colFree <= m
                newLogOdds(rowFree, colFree) = newLogOdds(rowFree, colFree) + sensorModelParams.depthMiss;
            end
        end
    end
end