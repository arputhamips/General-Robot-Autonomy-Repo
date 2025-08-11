% ================================ motionUpdate.m ================================
function dataStore = motionUpdate(dataStore)
% Incremental‑odometry motion model (new row in dataStore.odometry is [t dΔ aΔ])

    if isempty(dataStore.odometry)
        return;                                % nothing to apply
    end

    % most‑recent incremental motion
    deltaDist  = dataStore.odometry(end,2);    % metres
    deltaTheta = dataStore.odometry(end,3);    % radians

    % noise coefficients
    a1 = 0.0;   % rot noise from rot
    a2 = 0.0;   % trans noise from trans
    a3 = 0.0;   % trans noise from rot
    a4 = 0.0;   % rot noise from trans

    halfRot = deltaTheta / 2;

    for k = 1:numel(dataStore.particles)
        p = dataStore.particles(k);

        % sample noisy increments
        dth1 = halfRot + a1*abs(deltaTheta)*randn + a3*abs(deltaDist)*randn;
        trans= deltaDist + a2*abs(deltaDist)*randn + a4*abs(deltaTheta)*randn;
        dth2 = halfRot + a1*abs(deltaTheta)*randn + a3*abs(trans)*randn;

        % update pose
        th  = wrapToPi(p.theta + dth1);
        p.x = p.x + trans*cos(th);
        p.y = p.y + trans*sin(th);
        p.theta = wrapToPi(th + dth2);

        dataStore.particles(k) = p;
    end
end