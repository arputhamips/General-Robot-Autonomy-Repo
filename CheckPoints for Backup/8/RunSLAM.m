function RunSLAM(data)
% RunSLAM Executes EKF-SLAM and FastSLAM on input data and plots final results
%   data: M×N array where columns=[t, v, omega, dz1a_x, dz1a_y, ..., dzmba_x, dzmba_y]

%% Setup
dt = 1;                            % time step (s)
n_steps = size(data,1);
n_landmarks = (size(data,2) - 3)/2; % number of markers
n_walls = n_landmarks/2;

% Allocate storage
poses_ekf = zeros(n_steps,3);

%% 1) EKF-SLAM
% State vector: [x; y; theta; m1_x; m1_y; ...; mN_x; mN_y]
mu = zeros(3 + 2*n_landmarks,1);
mu(1:3) = [0;0;0];                % initial robot pose
P  = diag([0.01, 0.01, 0.01, repmat(1000,1,2*n_landmarks)]);

% Noise covariances
Q = [0.01 0.001 0.002;
     0.001 0.01  0.002;
     0.002 0.002 0.015];
R = 0.1 * eye(2);

for t = 2:n_steps
    % Motion update
    v     = data(t,2);
    omega = data(t,3);
    theta = mu(3);
    mu(1:3) = mu(1:3) + [v*cos(theta)*dt;
                          v*sin(theta)*dt;
                          omega*dt];
    Fx = [eye(3), zeros(3,2*n_landmarks)];
    G  = eye(3 + 2*n_landmarks);
    G(1,3) = -v*dt*sin(theta);
    G(2,3) =  v*dt*cos(theta);
    P = G * P * G' + Fx' * Q * Fx;

    % Measurement update
    z = data(t,4:end);
    for j = 1:n_landmarks
        zx = z(2*j-1);
        zy = z(2*j);
        if zx==0 && zy==0, continue; end
        idx = 3 + 2*j - 1;   % index of this landmark in mu
        % initialize if first observation
        if all(mu(idx:idx+1)==0)
            Rwb = [cos(theta), -sin(theta);
                   sin(theta),  cos(theta)];
            mu(idx:idx+1) = mu(1:2) + Rwb * [zx; zy];
        end
        % predicted measurement (body frame)
        dx = mu(idx)   - mu(1);
        dy = mu(idx+1) - mu(2);
        h  = [cos(theta), sin(theta);
             -sin(theta), cos(theta)] * [dx; dy];
        % Jacobians
        Hr = [-cos(theta), -sin(theta), -sin(theta)*dx + cos(theta)*dy;
               sin(theta), -cos(theta), -cos(theta)*dx - sin(theta)*dy];
        Hl = [ cos(theta), sin(theta);
              -sin(theta), cos(theta)];
        H = zeros(2,3+2*n_landmarks);
        H(:,1:3)       = Hr;
        H(:,idx:idx+1) = Hl;
        S = H * P * H' + R;
        K = P * H' / S;
        y = [zx; zy] - h;
        mu = mu + K * y;
        P  = (eye(size(P)) - K * H) * P;
    end
    poses_ekf(t,:) = mu(1:3)';
end

% Plot EKF-SLAM final trajectory & map
figure; hold on; axis equal; grid on;
plot(poses_ekf(:,1), poses_ekf(:,2), 'b.-', 'LineWidth',1.5, 'DisplayName','EKF Trajectory');
plot(mu(1), mu(2), 'ro', 'MarkerSize',8, 'DisplayName','EKF Final Pose');
for i = 1:n_walls
    % wall endpoints are markers 2*i-1 & 2*i
    a = 2*i - 1;
    b = 2*i;
    idx_a = 3 + 2*a - 1;
    idx_b = idx_a + 2;
    pA = mu(idx_a:idx_a+1);
    pB = mu(idx_b:idx_b+1);
    plot(pA(1), pA(2), 'kx','MarkerSize',8);
    draw_ellipse(pA, P(idx_a:idx_a+1, idx_a:idx_a+1),'k');
    plot(pB(1), pB(2), 'kx','MarkerSize',8);
    draw_ellipse(pB, P(idx_b:idx_b+1, idx_b:idx_b+1),'k');
    plot([pA(1),pB(1)], [pA(2),pB(2)], 'k-', 'LineWidth',1.5, 'DisplayName','Wall');
end
legend('Location','best');
title('EKF-SLAM Final Map & Trajectory');

%% 2) FastSLAM
% initialize particles
N = 100;
particles(N) = struct();
best_traj = zeros(n_steps,3);
for i = 1:N
    particles(i).pose = [0;0;0];
    particles(i).weight = 1/N;
    lm_t = struct('mu',[0;0], 'Sigma',1000*eye(2), 'observed',false);
    particles(i).landmarks = repmat(lm_t, n_landmarks,1);
end

for t = 2:n_steps
    u = data(t,2:3)';
    z = data(t,4:end);
    % motion update
    for i = 1:N
        th = particles(i).pose(3);
        motion = [u(1)*cos(th)*dt; u(1)*sin(th)*dt; u(2)*dt];
        w_t = mvnrnd([0;0;0], Q)';
        particles(i).pose = particles(i).pose + motion + w_t;
    end
    % measurement update
    for i = 1:N
        th = particles(i).pose(3);
        for j = 1:n_landmarks
            zx = z(2*j-1); zy = z(2*j);
            if zx==0 && zy==0, continue; end
            lm = particles(i).landmarks(j);
            if ~lm.observed
                Rwb = [cos(th), -sin(th); sin(th), cos(th)];
                lm.mu = particles(i).pose(1:2) + Rwb*[zx; zy];
                lm.Sigma = 0.1*eye(2);
                lm.observed = true;
            end
            dx = lm.mu(1)-particles(i).pose(1);
            dy = lm.mu(2)-particles(i).pose(2);
            h  = [cos(th), sin(th); -sin(th), cos(th)]*[dx;dy];
            H  = [cos(th), sin(th); -sin(th), cos(th)];
            S  = H*lm.Sigma*H' + R;
            K  = lm.Sigma*H'/S;
            y  = [zx; zy] - h;
            lm.mu = lm.mu + K*y;
            lm.Sigma = (eye(2)-K*H)*lm.Sigma;
            particles(i).weight = particles(i).weight * mvnpdf(y,[0;0],S);
            particles(i).landmarks(j) = lm;
        end
    end
    % normalize & resample
    W = [particles.weight]; W = W / sum(W);
    for i=1:N, particles(i).weight = W(i); end
    newp = particles; r=rand/N; c=W(1); idx=1;
    for m = 1:N
        U = r + (m-1)/N;
        while U > c, idx = idx+1; c = c + W(idx); end
        newp(m) = particles(idx); newp(m).weight = 1/N;
    end
    particles = newp;
    [~, best_i] = max([particles.weight]);
    best_traj(t,:) = particles(best_i).pose';
end

% Plot FastSLAM
figure; hold on; axis equal; grid on;
plot(best_traj(:,1), best_traj(:,2), 'b.-','LineWidth',1.5, 'DisplayName','Fast Trajectory');
for i = 1:n_walls
    a = 2*i - 1; b = 2*i;
    pA = particles(best_i).landmarks(a).mu;
    pB = particles(best_i).landmarks(b).mu;
    plot(pA(1),pA(2),'kx','MarkerSize',8);
    draw_ellipse(pA, particles(best_i).landmarks(a).Sigma,'k');
    plot(pB(1),pB(2),'kx','MarkerSize',8);
    draw_ellipse(pB, particles(best_i).landmarks(b).Sigma,'k');
    plot([pA(1),pB(1)],[pA(2),pB(2)],'k-','LineWidth',1.5,'DisplayName','Wall');
end
legend('Location','best');
title('FastSLAM Final Map & Trajectory');
end

%% Helper: draw_ellipse
function h = draw_ellipse(mu, Sigma, color)
    [V,D] = eig(Sigma);
    t = linspace(0,2*pi,100);
    pts = (V*sqrt(D))*[cos(t); sin(t)];
    h = plot(mu(1)+pts(1,:), mu(2)+pts(2,:), color, 'LineWidth',1.2);
end
