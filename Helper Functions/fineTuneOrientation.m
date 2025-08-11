function finalPose = fineTuneOrientation(Robot, dataStore, initPose, map, sensorOrigin, angles)
%FINETUNEORIENTATION Fine-tune orientation using depth measurements
%
% finalPose = fineTuneOrientation(Robot, dataStore, initPose, map, sensorOrigin, angles)
%
% This function fine-tunes the robot's orientation by:
% 1. Rotating slowly while collecting depth readings
% 2. Finding the orientation with best depth match
% 3. Returning to the best orientation
%
% Inputs:
%   Robot - Robot object
%   dataStore - Data structure
%   initPose - Initial pose estimate [x; y; theta]
%   map - Map of the environment
%   sensorOrigin - Sensor origin in robot frame
%   angles - Sensor angles
%
% Outputs:
%   finalPose - Fine-tuned pose [x; y; theta]

% Parameters
rotSpeed = 0.2;  % Slower rotation for precision
scanAngle = pi/2;  % Scan +/- 45 degrees from current orientation
numSteps = 20;  % Number of steps to scan
angStep = 2*scanAngle / numSteps;
pauseTime = 0.2;  % Pause at each step

fprintf('Starting orientation fine-tuning...\n');

% Initial position
x0 = initPose(1);
y0 = initPose(2);
theta0 = initPose(3);

% First, rotate to -scanAngle/2 position
targetRot = -scanAngle/2;
rotTime = abs(targetRot) / rotSpeed;
SetFwdVelAngVelCreate(Robot, 0, -rotSpeed * sign(targetRot));
pause(rotTime);
SetFwdVelAngVelCreate(Robot, 0, 0);
pause(pauseTime);

% Now scan from -scanAngle/2 to +scanAngle/2
bestScore = -inf;
bestTheta = theta0;
bestDepthErr = inf;

for step = 0:numSteps
    % Current angle relative to initial orientation
    currAngle = -scanAngle/2 + step * angStep;
    currTheta = wrapToPi(theta0 + currAngle);
    
    % Collect depth readings
    [~, dataStore] = readStoreSensorData(Robot, 0, dataStore);
    
    % Check if we have depth data
    if ~isempty(dataStore.rsdepth) && size(dataStore.rsdepth, 1) >= 1
        % Get latest depth readings
        latestDepth = dataStore.rsdepth(end, 3:end)';
        
        % Create test pose
        testPose = [x0; y0; currTheta];
        
        % Predict depth readings
        predDepth = depthPredict(testPose, map, sensorOrigin, angles);
        
        % Calculate error
        depthErr = norm(predDepth - latestDepth);
        
        % Check if this is better than previous best
        if depthErr < bestDepthErr
            bestDepthErr = depthErr;
            bestTheta = currTheta;
            bestScore = 1/depthErr;
            fprintf('  New best orientation: %.1f° (error: %.4f)\n', bestTheta*180/pi, depthErr);
        end
    end
    
    % Move to next angle (unless at last step)
    if step < numSteps
        SetFwdVelAngVelCreate(Robot, 0, rotSpeed);
        pause(angStep/rotSpeed);
        SetFwdVelAngVelCreate(Robot, 0, 0);
        pause(pauseTime);
    end
end

% Rotate back to best orientation
currTheta = wrapToPi(theta0 + scanAngle/2);  % Current orientation after scan
targetRot = wrapToPi(bestTheta - currTheta);  % Amount to rotate
rotTime = abs(targetRot) / rotSpeed;
SetFwdVelAngVelCreate(Robot, 0, rotSpeed * sign(targetRot));
pause(rotTime);
SetFwdVelAngVelCreate(Robot, 0, 0);

fprintf('Fine-tuning complete. Best orientation: %.1f°\n', bestTheta*180/pi);

% Return final pose
finalPose = [x0; y0; bestTheta];
end