% ========================= updateOptionalWalls.m =========================
function [particle, weightOpt] = updateOptionalWalls(particle, depthMeas, optWalls)
%UPDATEOPTIONALWALLS  Infer existence of optional walls for one particle.
%
%   [particle, weightOpt] = updateOptionalWalls(particle, depthMeas, optWalls)
%
%   INPUTS
%     particle   : struct with fields x, y, theta, optWallsState (N×1 int8)
%     depthMeas  : 1×M vector of projected RealSense ranges (metres)
%     optWalls   : N×4 matrix [x1 y1 x2 y2] of optional wall segments
%
%   OUTPUTS
%     particle   : updated particle (optWallsState may be modified)
%     weightOpt  : multiplicative weight from optional‑wall evidence
%
%   NOTES
%   • optWallsState =  0  →  undecided
%                     +1  →  wall exists
%                     -1  →  wall absent
%   • A soft weight factor is returned (linear space) so that weights do
%     not collapse.  Evidence is accumulated per wall and mapped to a
%     logistic‑style confidence.

% ------------------------- tunable parameters ---------------------------
maxRange      = 10.0;          % Realsense max range (m)
sigmaDepth    = 0.10;          % depth noise σ (m)
evidenceStep  = 1;             % vote amount per supporting beam
evidenceThresh= 2;             % |evidence| > thresh ⇒ commit decision
betaConf      = 0.4;           % weight scaling: exp(±betaConf)
%   (betaConf ~0.4 ⇒ exp(±0.4) ≈ 1.5× / 0.67×)

% --------------------------- beam geometry ------------------------------
M           = numel(depthMeas);
fovDeg      = 54;                       % RealSense depth FOV
stepRad     = deg2rad(fovDeg/(M-1));    % ~6.75° spacing
startRad    = deg2rad(-fovDeg/2);       % -27° at i=1
beamAngles  = startRad + (0:M-1)*stepRad;
cDir        = cos(particle.theta + beamAngles);
sDir        = sin(particle.theta + beamAngles);

% initialise output
weightOpt   = 1.0;

% ------------------------ iterate optional walls ------------------------
for w = 1:size(optWalls,1)
    % skip already–decided walls
    if particle.optWallsState(w) ~= 0,  continue;  end

    % wall endpoints
    x1 = optWalls(w,1);  y1 = optWalls(w,2);
    x2 = optWalls(w,3);  y2 = optWalls(w,4);

    evidence = 0;

    % ---- iterate beams once per wall ----
    for i = 1:M
        % ray endpoint (maxRange away)
        rx = particle.x + maxRange * cDir(i);
        ry = particle.y + maxRange * sDir(i);

        [hit, ~, ~, rangeExp] = lineIntersection( ...
            particle.x, particle.y, rx, ry, x1, y1, x2, y2);

        if ~hit,  continue,  end          % beam misses this wall

        % compare expected vs measured
        rangeAct = depthMeas(i);
        err      = rangeAct - rangeExp;

        if abs(err) < 3*sigmaDepth
            evidence = evidence + evidenceStep;      % supports existence
        elseif err > 0
            evidence = evidence - evidenceStep;      % suggests absence
        end
    end

    % ---- commit decision if evidence strong enough ----
    if evidence >  evidenceThresh
        particle.optWallsState(w) =  1;              % wall exists
        weightOpt = weightOpt * exp( betaConf);
    elseif evidence < -evidenceThresh
        particle.optWallsState(w) = -1;              % wall absent
        weightOpt = weightOpt * exp(-betaConf);
    end
end
end
