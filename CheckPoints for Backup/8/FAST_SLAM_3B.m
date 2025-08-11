
clear; close all; clc;

%% 1) Load data and set parameters
load('SLAM.mat');               % loads SLAM (31×19)
data      = SLAM;
dt        = 1;                  % assume 1 s between steps
N         = 100;                % number of particles
n_steps   = size(data,1);
n_lm      = 8;                  % 8 markers (4 walls × 2)
Q         = [0.01 0.001 0.002;   % process‐noise covariance (x,y,θ)
             0.001 0.01 0.002;
             0.002 0.002 0.015];
R         = 0.1 * eye(2);       % measurement noise

%% 2) Initialize particles
particles(N) = struct();
for i = 1:N
    particles(i).pose       = [0;0;0];                  % x, y, θ
    particles(i).weight     = 1/N;
    lm_template            = struct('mu',[0;0], ...
                                     'Sigma',1000*eye(2), ...
                                     'observed',false);
    particles(i).landmarks = repmat(lm_template, n_lm, 1);
end

best_traj = zeros(n_steps,3);

%% 3) Fast SLAM loop
for t = 2:n_steps
    u = data(t,2:3)';             % [v; ω]
    z = data(t,4:end);            % 16 measurements

    % --- 3.1) Motion update (sampled) ---
    for i = 1:N
        theta = particles(i).pose(3);
        % deterministic motion increment
        motion = [ u(1)*cos(theta)*dt;
                   u(1)*sin(theta)*dt;
                   u(2)*dt ];
        % sample 3×1 process noise
        w_t = mvnrnd([0;0;0], Q)';  
        particles(i).pose = particles(i).pose + motion + w_t;
    end

    % --- 3.2) Measurement & weight update ---
    for i = 1:N
        theta_i = particles(i).pose(3);
        for l = 1:n_lm
            zx = z(2*l-1); zy = z(2*l);
            if zx==0 && zy==0, continue; end

            lm = particles(i).landmarks(l);
            % initialize if first seen
            if ~lm.observed
                Rwb = [cos(theta_i), -sin(theta_i);
                       sin(theta_i),  cos(theta_i)];
                lm.mu        = particles(i).pose(1:2) + Rwb*[zx; zy];
                lm.Sigma     = 0.1*eye(2);
                lm.observed  = true;
            end

            % predicted measurement in body frame
            dx = lm.mu(1) - particles(i).pose(1);
            dy = lm.mu(2) - particles(i).pose(2);
            h  = [ cos(theta_i),  sin(theta_i);
                  -sin(theta_i),  cos(theta_i)] * [dx; dy];

            H = [ cos(theta_i),  sin(theta_i);
                 -sin(theta_i),  cos(theta_i)];

            S   = H*lm.Sigma*H' + R;
            K   = lm.Sigma*H'/S;
            y   = [zx; zy] - h;
            lm.mu    = lm.mu + K*y;
            lm.Sigma = (eye(2)-K*H)*lm.Sigma;

            % update weight
            particles(i).weight = particles(i).weight * mvnpdf(y, [0;0], S);
            particles(i).landmarks(l) = lm;
        end
    end

    % --- 3.3) Normalize weights ---
    Wtot = sum([particles.weight]);
    for i = 1:N
        particles(i).weight = particles(i).weight / Wtot;
    end

    % --- 3.4) Low‐variance resampling ---
    newparts = particles;
    r        = rand/N;         c = particles(1).weight; idx = 1;
    for m = 1:N
        U = r + (m-1)/N;
        while U > c
            idx = idx + 1;
            c   = c + particles(idx).weight;
        end
        newparts(m) = particles(idx);
        newparts(m).weight = 1/N;
    end
    particles = newparts;

    % --- 3.5) Record best particle's pose for trajectory ---
    [~, best_i]      = max([particles.weight]);
    best_traj(t,:)   = particles(best_i).pose';
end

%% 4) Plot final map & best trajectory
best_particle = particles(best_i);

figure; hold on; axis equal; grid on;
title('Fast SLAM – Final Map & Trajectory');
xlabel('X (m)'); ylabel('Y (m)');

% Dummy handles for clean legend
h_traj    = plot(NaN,NaN,'b.-','LineWidth',1.5,'DisplayName','Trajectory');
h_pose    = plot(NaN,NaN,'ro','MarkerSize',8,'DisplayName','Final Pose');
h_lm      = plot(NaN,NaN,'kx','MarkerSize',8,'DisplayName','Landmarks');
h_ell     = plot(NaN,NaN,'k-','LineWidth',1.2,'DisplayName','Landmark Uncertainty');
h_wall    = plot(NaN,NaN,'k-','LineWidth',1.2,'DisplayName','Walls');
h_head    = quiver(NaN,NaN,NaN,NaN,'r','MaxHeadSize',1,'LineWidth',1.5,'DisplayName','Heading');

% 4.1) Trajectory & final pose
plot(best_traj(:,1), best_traj(:,2), 'b.-', 'LineWidth',1.5);
plot(best_particle.pose(1), best_particle.pose(2), 'ro', 'MarkerSize',8);

% 4.2) Landmarks + ellipses
for l = 1:n_lm
    lm = best_particle.landmarks(l);
    if lm.observed
        plot(lm.mu(1), lm.mu(2), 'kx', 'MarkerSize',8);
        draw_ellipse(lm.mu, lm.Sigma, 'k');
    end
end

% 4.3) Connect each wall's two markers
for w = 1:4
    p1 = best_particle.landmarks(2*w-1).mu;
    p2 = best_particle.landmarks(2*w  ).mu;
    plot([p1(1),p2(1)], [p1(2),p2(2)], 'k-', 'LineWidth',1.2);
end

% 4.4) Draw heading arrow
pose = best_particle.pose;
a    = 0.3;
quiver(pose(1), pose(2), a*cos(pose(3)), a*sin(pose(3)), ...
       'r','MaxHeadSize',1,'LineWidth',1.5);

legend('Location','best');

%% helper: draw_ellipse
function h = draw_ellipse(mu, Sigma, color)
    [V, D] = eig(Sigma);
    t      = linspace(0,2*pi,100);
    a_pts  = (V*sqrt(D))*[cos(t); sin(t)];
    h      = plot(mu(1)+a_pts(1,:), mu(2)+a_pts(2,:), color, 'LineWidth',1.2);
end
