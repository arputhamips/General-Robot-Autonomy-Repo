function likelihood = depthMeasurementLikelihood(particle, depthReadings, map)
%DEPTHMEASUREMENTLIKELIHOOD  Compute particle likelihood from depth sensor.
%
%   likelihood = depthMeasurementLikelihood(particle, depthReadings, map)
%
%   INPUTS
%     particle      : struct with fields x, y, theta            (robot pose)
%     depthReadings : 1×N vector of RealSense ranges (metres)   (latest frame)
%     map           : M×4 wall‑segment matrix [x1 y1 x2 y2]
%
%   OUTPUT
%     likelihood    : scalar probability of the measurement given the pose
%
%   The model projects each ray onto the robot’s forward (z) axis to match
%   the RealSense depth interpretation.  A Gaussian error model is applied
%   and accumulated in log‑space for numerical stability.

% --- CONSTANT PARAMETERS -------------------------------------------------
sigmaRange   = 0.03;                % depth noise σ (m)
maxRange     = 10.0;                % sensor max (m)
minRange     = 0.175;               % sensor min (m)
angleStepDeg = 9;                   % spacing between rays (deg)
fovStartDeg  = -27;                 % first ray offset from heading (deg)
logFloor     = log(1e-12);          % floor to avoid log(0)

% --- PRE‑COMPUTED GEOMETRY ----------------------------------------------
deg2rad      = pi/180;
angleStepRad = angleStepDeg * deg2rad;
nRays        = numel(depthReadings);
startAngle   = fovStartDeg * deg2rad;

persistent cosProj
if isempty(cosProj) || numel(cosProj) ~= nRays
    rayIdx  = 0:nRays-1;
    cosProj = cos(startAngle + rayIdx * angleStepRad);  % z‑axis projection
end

% --- ACCUMULATE LOG‑LIKELIHOOD ------------------------------------------
logLikelihood = 0;
validCount    = 0;

for k = 1:nRays
    range = depthReadings(k);

    % skip invalid ranges
    if range < minRange || range > maxRange
        continue
    end

    % global bearing of this ray
    globalAngle = wrapToPi(particle.theta + startAngle + (k-1)*angleStepRad);

    % expected radial distance by ray‑casting
    expectedRadial = raycast(particle.x, particle.y, globalAngle, map, maxRange);

    % project expected distance onto forward axis
    expectedDepth = expectedRadial * cosProj(k);

    % Gaussian likelihood
    error      = range - expectedDepth;
    prob       = exp(-0.5*(error/sigmaRange)^2) / (sqrt(2*pi)*sigmaRange);
    logProb    = max(log(prob), logFloor);   % clamp to avoid -Inf

    logLikelihood = logLikelihood + logProb;
    validCount    = validCount + 1;
end

% --- FINAL OUTPUT --------------------------------------------------------
if validCount == 0
    likelihood = 1e-9;                       % no valid rays -> tiny weight
else
    likelihood = exp(logLikelihood);
end
end
