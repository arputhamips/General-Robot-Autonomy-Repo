function isValid = validateDepthMeasurements(dataStore, pose, map, sensorOrigin, angles, threshold)
%VALIDATEDEPTHMEASUREMENTS Validate a pose using depth measurements
%
% isValid = validateDepthMeasurements(dataStore, pose, map, sensorOrigin, angles, threshold)
%
% Inputs:
%   dataStore - Data structure with depth measurements
%   pose - Pose to validate [x; y; theta]
%   map - Map of the environment
%   sensorOrigin - Origin of the depth sensor in robot frame
%   angles - Sensor angles
%   threshold - Validation threshold
%
% Outputs:
%   isValid - Boolean indicating if the pose is valid

% Check if we have depth data
if isempty(dataStore.rsdepth) || size(dataStore.rsdepth, 1) < 1
    fprintf('No depth data available for validation\n');
    isValid = false;
    return;
end

% Get latest depth readings
latestDepth = dataStore.rsdepth(end, 3:end)';

% Predict depth readings from the pose
predDepth = depthPredict(pose, map, sensorOrigin, angles);

% Calculate error
depthError = norm(predDepth - latestDepth);
relativeError = depthError / (mean(latestDepth) + 0.001);

% Individual sensor errors
individual_errors = abs(predDepth - latestDepth);
num_good_matches = sum(individual_errors < threshold);

fprintf('Depth validation:\n');
fprintf('  Overall error: %.4f m\n', depthError);
fprintf('  Relative error: %.2f%%\n', relativeError*100);
fprintf('  Matching sensors: %d of %d\n', num_good_matches, length(angles));

% Validation criteria: low overall error OR good individual sensor matches
isValid = (depthError < threshold) || (num_good_matches >= length(angles)*0.7);

if isValid
    fprintf('✅ Depth validation passed\n');
else
    fprintf('❌ Depth validation failed\n');
end
end