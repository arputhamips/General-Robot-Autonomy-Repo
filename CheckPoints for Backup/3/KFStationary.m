function [muFinal, sigmaFinal] = KFStationary(mu0, sigma0, measurements)
% KFStationary - Estimates the position of the robot using a Kalman Filter.
%
%   INPUTS:
%       mu0         - Initial mean position [x; y] (2x1 vector)
%       sigma0      - Initial covariance matrix (2x2)
%       measurements - M x 4 matrix of range measurements [North, East, South, West]
%
%   OUTPUTS:
%       muFinal     - Final estimated position (2x1 vector)
%       sigmaFinal  - Final covariance matrix (2x2)

% Define measurement noise covariance matrix R
R = diag([0.1, 0.3, 0.1, 0.3].^2); % Variance for [N, E, S, W]

% Identity transition matrix (robot is stationary)
F = eye(2);

% Initial state
mu = mu0;
sigma = sigma0;

% Process each measurement
for i = 1:size(measurements, 1)
    z = measurements(i, :)';  % Extract current measurement vector
    
    % Ignore NaN values (no obstacle detected within range)
    validIdx = ~isnan(z);
    if sum(validIdx) < 2  % Skip update if less than 2 valid measurements
        continue;
    end

    % Construct measurement matrix H dynamically based on valid measurements
    H = [];
    Z_valid = [];
    R_valid = [];
    
    if validIdx(1) % North
        H = [H; 0, 1];  
        Z_valid = [Z_valid; z(1)];
        R_valid = [R_valid, R(1,1)];
    end
    if validIdx(2) % East
        H = [H; 1, 0];  
        Z_valid = [Z_valid; z(2)];
        R_valid = [R_valid, R(2,2)];
    end
    if validIdx(3) % South
        H = [H; 0, 1];  
        Z_valid = [Z_valid; -z(3)];
        R_valid = [R_valid, R(3,3)];
    end
    if validIdx(4) % West
        H = [H; 1, 0];  
        Z_valid = [Z_valid; -z(4)];
        R_valid = [R_valid, R(4,4)];
    end

    % Convert R_valid to diagonal matrix
    R_valid = diag(R_valid);
    
    % Measurement update step
    S = H * sigma * H' + R_valid;   % Innovation covariance
    K = sigma * H' / S;             % Kalman gain
    mu = mu + K * (Z_valid - H * mu); % Update mean
    sigma = (eye(2) - K * H) * sigma; % Update covariance
end

% Return final estimates
muFinal = mu;
sigmaFinal = sigma;

end
