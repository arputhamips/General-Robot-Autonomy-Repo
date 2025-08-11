function newLogOdds = logOddsBump(prevLogOdds, robotPose, bumpSensorData, ...
                                  mapResolution, gridOrigin, sensorModelParams)

%   newLogOdds = logOddsBump(prevLogOdds, robotPose, bumpSensorData, ...
%                            mapResolution, gridOrigin, sensorModelParams)
% INPUTS:
%   prevLogOdds       - n-by-m matrix of current log-odds occupancy values.
%   robotPose         - 1x3 vector [x, y, theta] (theta in radians) representing the
%                       robot's pose in world coordinates.
%   bumpSensorData    - 1x3 vector [BumpRight, BumpLeft, BumpFront] where each element is
%                       1 if the corresponding bump sensor is triggered, 0 otherwise.
%   mapResolution     - Scalar representing the map resolution (meters per cell).
%   gridOrigin        - 1x2 vector [x0, y0] of world coordinates corresponding to grid cell (1,1).
%   sensorModelParams - Struct with sensor model parameters, containing at least:
%                           .bumpHit   : Log-odds increment when a bump is detected.
%                           .bumpRange : Distance (in meters) from the robot's center to the
%                                        sensor contact point.
%
% OUTPUT:
%   newLogOdds        - Updated n-by-m log-odds occupancy grid.
%
%  AUTONOMOUS MOBILE ROBOTS HW5
%  CORNELL UNIVERSITY
%  NIRMAL , A J L A

% Copy the previous log-odds grid
newLogOdds = prevLogOdds;

% Unpack robot pose
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% Define sensor directions relative to the robot's heading:
%   BumpRight -> theta - pi/2
%   BumpLeft  -> theta + pi/2
%   BumpFront -> theta
sensorAngles = [theta - pi/2, ...  % BumpRight
                theta + pi/2, ...  % BumpLeft
                theta];           % BumpFront

% Process each sensor
for sensorIdx = 1:3
    if bumpSensorData(sensorIdx) == 1
        % Compute the world coordinates of the bump contact point for this sensor
        sensor_x = x + sensorModelParams.bumpRange * cos(sensorAngles(sensorIdx));
        sensor_y = y + sensorModelParams.bumpRange * sin(sensorAngles(sensorIdx));
        
        % Convert the world coordinates to grid indices
        % (Assuming grid cell (1,1) is at gridOrigin and indices are computed using floor)
        col = floor((sensor_x - gridOrigin(1)) / mapResolution) + 1;
        row = floor((sensor_y - gridOrigin(2)) / mapResolution) + 1;
        
        % Get grid dimensions
        [n, m] = size(prevLogOdds);
        
        % Update the grid cell if indices are within bounds
        if row >= 1 && row <= n && col >= 1 && col <= m
            newLogOdds(row, col) = newLogOdds(row, col) + sensorModelParams.bumpHit;
        else
            warning('Bump sensor %d contact point (%.2f, %.2f) is outside the grid bounds.', sensorIdx, sensor_x, sensor_y);
        end
    end
end

end