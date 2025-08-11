function likelihood = beaconMeasurementLikelihood(particle, beaconRow, beaconLoc)
% beaconRow format (per beacon, 5 fields): [id  dx  dy  dtheta  rssi]
% dx,dy are beacon coordinates in the *robot* frame (metres).
% beaconLoc  : [id  x  y]     – ground‑truth global beacon positions.
% Returns product of per‑beacon Gaussians (σ = 0.2 m).

    sigmaB   = 0.20;                       % position std‑dev (m)
    varB     = sigmaB^2;
    normCoef = 1/(2*pi*varB);

    likelihood = 1.0;

    % Strip any padding zeros at row end
    beaconRow = beaconRow(beaconRow ~= 0);
    if isempty(beaconRow), return; end

    % Re‑shape robustly
    colsPerBeacon = 5;
    numBeacons    = floor(numel(beaconRow) / colsPerBeacon);
    beaconMat     = reshape(beaconRow(1:colsPerBeacon*numBeacons), ...
                            colsPerBeacon, []).';     % Nx5

    for b = 1:numBeacons
        id  = beaconMat(b,1);
        if id == 0, continue; end                     % unused slot

        idx = find(beaconLoc(:,1) == id, 1);
        if isempty(idx), continue; end                % unknown beacon

        trueX = beaconLoc(idx,2);
        trueY = beaconLoc(idx,3);

        % measured position in robot frame
        dx = beaconMat(b,2);
        dy = beaconMat(b,3);

        % transform to global
        c = cos(particle.theta); s = sin(particle.theta);
        measX = particle.x + c*dx - s*dy;
        measY = particle.y + s*dx + c*dy;

        errSq = (measX - trueX)^2 + (measY - trueY)^2;
        likelihood = likelihood * normCoef * exp(-errSq/(2*varB));
    end
end