function weightB = beaconMeasurementLikelihood(particle, beaconRow, beaconLoc)
%BEACONMEASUREMENTLIKELIHOOD  2D range–bearing update from a single tag.
%
%   weightB = beaconMeasurementLikelihood(particle, beaconRow, beaconLoc)
%
%   INPUTS
%     particle   : struct with fields .x, .y, .theta
%     beaconRow  : 1×5 [dt id x y rot]  (rot ignored)
%     beaconLoc  : M×3 [id  x  y]  for all tags in the map
%
%   OUTPUT
%     weightB    : scalar likelihood p(z_beacon | particle)

    % --- noise parameters ---
    sigmaRange   = 0.20;           % std dev of range (m)
    sigmaBearing =  5 * pi/180;    % std dev of bearing (rad)

    % --- parse measurement ---
    id   = beaconRow(2);
    dPerp = beaconRow(3);   % forward (perpendicular) distance (m)
    offs  = beaconRow(4);   % lateral offset (m)

    % --- true beacon position ---
    idx = find(beaconLoc(:,1)==id,1);
    if isempty(idx)
        weightB = 1;   % unknown tag → no info
        return;
    end
    bx = beaconLoc(idx,2);
    by = beaconLoc(idx,3);

    % --- expected measurement from particle ---
    dx = bx - particle.x;
    dy = by - particle.y;
    expRange   = hypot(dx, dy);
    expBearing = wrapToPi(atan2(dy, dx) - particle.theta);

    % --- actual measurement in range+bearing form ---
    measRange   = hypot(dPerp, offs);
    measBearing = atan2(offs, dPerp);

    % --- 2×2 covariance ---
    R = diag([sigmaRange^2, sigmaBearing^2]);

    % --- innovation ---
    err = [measRange - expRange;
           wrapToPi(measBearing - expBearing)];

    % --- Gaussian likelihood ---
    weightB = exp(-0.5 * (err.' / R) * err) / (2*pi*sqrt(det(R)));
end
