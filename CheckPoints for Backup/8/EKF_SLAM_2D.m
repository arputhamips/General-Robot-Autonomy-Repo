load('SLAM.mat');      % Load data
data = SLAM;           % 31x19 matrix
dt = 1;                % Time step (assumed 1s)

% Constants
n_steps = size(data,1);
n_landmarks = 8;
mu = zeros(19,1);      % [x y theta m1ax m1ay ... m4bx m4by]
mu(1:3) = [0; 0; 0];   % Initial pose
P = diag([0.01 0.01 0.01 repmat(1000,1,16)]); % Initial covariance

% Process noise
Q = [0.01 0.001 0.002; 0.001 0.01 0.002; 0.002 0.002 0.015];

% Measurement noise
R = 0.1 * eye(2);

% Indices of 3 time steps to visualize
time_indices = [1, 15, 31];
pose_history = zeros(n_steps, 3);

for t = 2:n_steps
    %% 1. Prediction step
    v = data(t,2); w = data(t,3);
    theta = mu(3);
    
    % Motion model
    mu(1:3) = mu(1:3) + [v*cos(theta)*dt; v*sin(theta)*dt; w*dt];

    % Jacobian of motion w.r.t state (only top-left 3x3 changes)
    Fx = [eye(3), zeros(3,16)];
    Gt = eye(19);
    Gt(1,3) = -v*dt*sin(theta);
    Gt(2,3) =  v*dt*cos(theta);

    % Update covariance
    P = Gt * P * Gt' + Fx' * Q * Fx;

    %% 2. Measurement update
    z = data(t,4:end);
    for i = 1:n_landmarks
        zx = z(2*i-1); zy = z(2*i);
        if zx == 0 && zy == 0
            continue; % Not observed
        end
        
        lm_idx = 3 + 2*i - 1;
        landmark = mu(lm_idx:lm_idx+1);

        % If landmark never seen, initialize in global frame
        if all(landmark == 0)
            theta = mu(3);
            R_theta = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            mu(lm_idx:lm_idx+1) = mu(1:2) + R_theta * [zx; zy];
        end

        % Predict measurement
        dx = mu(lm_idx) - mu(1);
        dy = mu(lm_idx+1) - mu(2);
        theta = mu(3);
        h = [cos(theta), sin(theta); -sin(theta), cos(theta)] * [dx; dy];

        % Jacobians
        Hr = [ -cos(theta), -sin(theta), -sin(theta)*dx + cos(theta)*dy;
                sin(theta), -cos(theta), -cos(theta)*dx - sin(theta)*dy ];
        Hl = [  cos(theta), sin(theta);
               -sin(theta), cos(theta)];

        H = zeros(2,19);
        H(:,1:3) = Hr;
        H(:,lm_idx:lm_idx+1) = Hl;

        % Kalman gain
        S = H * P * H' + R;
        K = P * H' / S;

        % Measurement innovation
        z_meas = [zx; zy];
        y = z_meas - h;

        % Update state and covariance
        mu = mu + K * y;
        P = (eye(19) - K * H) * P;
    end
    
    pose_history(t,:) = mu(1:3)';

    %% Plot at selected steps
    if any(t == time_indices)
        figure;
        hold on; axis equal;
        title(sprintf('EKF-SLAM at Time Step %d', t));

        % Plot robot trajectory so far
        plot(pose_history(1:t,1), pose_history(1:t,2), 'b.-');
        plot(mu(1), mu(2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Robot');

        % Plot landmarks
        for i = 1:n_landmarks
            lm_idx = 3 + 2*i - 1;
            plot(mu(lm_idx), mu(lm_idx+1), 'kx', 'MarkerSize', 8);
            
            % Draw uncertainty ellipse
            Sigma_lm = P(lm_idx:lm_idx+1, lm_idx:lm_idx+1);
            draw_ellipse(mu(lm_idx:lm_idx+1), Sigma_lm, 'k');
        end

        % Draw robot pose uncertainty ellipse
        draw_ellipse(mu(1:2), P(1:2,1:2), 'r');
        legend;
    end
end

function draw_ellipse(mu, Sigma, color)
    [V, D] = eig(Sigma);
    t = linspace(0, 2*pi, 100);
    a = (V * sqrt(D)) * [cos(t); sin(t)];
    plot(mu(1) + a(1,:), mu(2) + a(2,:), color);
end