function particlesOut = particleFilterStep(map)
    global dataStore;
    persistent P W lastOdo

    % PARAMETERS
    N            = 5000;                       
    transNoise   = 0.02;                      
    rotNoise     = 0.05;                      
    sensorOrigin = [0, 0.08];                 
    angles       = linspace(0.4712,-0.4712,9);
    maxRange     = 4;                         

    % INITIALIZE on first call
    if isempty(P)
        xs = [map(:,1); map(:,3)];
        ys = [map(:,2); map(:,4)];
        xmin = min(xs); xmax = max(xs);
        ymin = min(ys); ymax = max(ys);
        P = [ ...
            rand(N,1)*(xmax-xmin)+xmin, ...
            rand(N,1)*(ymax-ymin)+ymin, ...
            rand(N,1)*2*pi-pi ];
        W = ones(N,1)/N;
        lastOdo = [];   % also reset lastOdo
        dataStore.truthPose = zeros(0,4);
    end

    % skip if no odometry
    if isempty(dataStore.odometry)
        particlesOut = P;
        return;
    end

    % initialize lastOdo if needed
    if isempty(lastOdo)
        lastOdo = dataStore.odometry(end,2);
    end

    % signed delta distance
    rawOdo = dataStore.odometry(end,2);
    d      = rawOdo - lastOdo;  
    lastOdo = rawOdo;

    % rotation
    a = dataStore.odometry(end,3);
    tstamp = dataStore.odometry(end,1);

    % FETCH DEPTH
    if isfield(dataStore,'rsdepth') && ~isempty(dataStore.rsdepth)
        rd = dataStore.rsdepth(end,:);
        z  = rd(3:end);
        tstamp = rd(1);
    else
        z = [];
    end

    % 1) MOTION UPDATE
    d_noisy   = d + transNoise * randn(N,1);
    a_noisy   = a + rotNoise   * randn(N,1);
    theta_new = P(:,3) + a_noisy;
    P(:,1)    = P(:,1) + d_noisy .* cos(P(:,3));
    P(:,2)    = P(:,2) + d_noisy .* sin(P(:,3));
    P(:,3)    = mod(theta_new + pi, 2*pi) - pi;

    % 2) MEASUREMENT UPDATE
    if ~isempty(z)
        likeVec = ones(N,1);
        for i = 1:N
            px = P(i,1); py = P(i,2); pt = P(i,3);
            sx = px + cos(pt)*sensorOrigin(1) - sin(pt)*sensorOrigin(2);
            sy = py + sin(pt)*sensorOrigin(1) + cos(pt)*sensorOrigin(2);
            for k = 1:numel(angles)
                zk = z(k);
                if isnan(zk), continue; end
                th = pt + angles(k);
                dx = cos(th); dy = sin(th);
                dmin = maxRange;
                for w = 1:size(map,1)
                    x1 = map(w,1); y1 = map(w,2);
                    x2 = map(w,3); y2 = map(w,4);
                    denom = (y2-y1)*dx - (x2-x1)*dy;
                    if denom==0, continue; end
                    ua = ((x2-x1)*(sy-y1)-(y2-y1)*(sx-x1)) / denom;
                    ub = (dx*(sy-y1)-dy*(sx-x1)) / denom;
                    if ua>0 && ub>=0 && ub<=1 && ua<dmin
                        dmin = ua;
                    end
                end
                if     zk>=maxRange && dmin>=maxRange
                    lk = 0.9;
                elseif zk>=maxRange || dmin>=maxRange
                    lk = 0.1;
                else
                    sigma = 0.05 + 0.03*zk;
                    e     = zk - dmin;
                    lk    = 0.9*exp(-0.5*(e/sigma)^2) + 0.1/maxRange;
                end
                likeVec(i) = likeVec(i)*lk;
            end
        end
        W = W .* likeVec;
    end

    % 3) NORMALIZE
    W = W + eps;
    W = W / sum(W);

    % 4) RESAMPLE IF NEEDED
    Neff = 1 / sum(W.^2);
    if Neff < N/3
        edges = [0; cumsum(W)];
        u     = rand(N,1)/N + (0:N-1)'/N;
        idx   = arrayfun(@(v)find(edges<=v,1,'last'), u);
        P     = P(idx,:);
        W     = ones(N,1)/N;
    end

    % 5) ESTIMATE & STORE MEAN POSE
    x_mean     = W' * P(:,1);
    y_mean     = W' * P(:,2);
    theta_mean = atan2(W' * sin(P(:,3)), W' * cos(P(:,3)));
    dataStore.truthPose(end+1,:) = [tstamp, x_mean, y_mean, theta_mean];

    % RETURN
    particlesOut = P;
end
