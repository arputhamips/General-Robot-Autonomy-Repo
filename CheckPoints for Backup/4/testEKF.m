function [mu_next_gps, sigma_next_gps, mu_next_depth, sigma_next_depth] = ...
    testEKF(mu, ut, sigma, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       mu           previous vector of pose state (mu(t-1))
%       ut           current command [d; phi]
%       sigma        previous covariance matrix
%       R            state model noise covariance matrix
%       z_gps        current gps measurement vector
%       Q_GPS        GPS measurement noise covariance matrix
%       z_depth      depth measurement vector
%       Q_depth      Realsense depth measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       n_rs_rays    number of evenly distributed realsense depth rays
%
%   OUTPUTS
%       mu_next_gps      current estimate of vector of pose state (gps)
%       sigma_next_gps   current covariance matrix (gps)
%       mu_next_depth    current estimate of vector of pose state (depth)
%       sigma_next_depth current covariance matrix (depth)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   NIRMAL A J L A

% ---- Define Motion and Measurement Models ----
motionModel = @(x, u) integrateOdom(x, u(1), u(2));
motionJac = @(x, u) GjacDiffDrive(x, u);

gpsMeasurement = @(x) hGPS(x);
gpsJac = @(x) HjacGPS(x);

depthMeasurement = @(x) depthPredict(x, map, sensor_pos, linspace(deg2rad(27), deg2rad(-27), n_rs_rays));
depthJac = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays);

% ---- EKF PREDICTION STEP ----
mu_pred = motionModel(mu, ut);          % Predicted state
G = motionJac(mu, ut);                   % Motion model Jacobian
sigma_pred = G * sigma * G' + R;         % Predicted covariance

% ---- EKF UPDATE STEP (GPS) ----
H_gps = gpsJac(mu_pred);                  % Measurement Jacobian for GPS
K_gps = sigma_pred * H_gps' / (H_gps * sigma_pred * H_gps' + Q_GPS); % Kalman gain
mu_next_gps = mu_pred + K_gps * (z_gps - gpsMeasurement(mu_pred));  % State update
sigma_next_gps = (eye(3) - K_gps * H_gps) * sigma_pred;             % Covariance update

% ---- EKF UPDATE STEP (Depth) ----
H_depth = depthJac(mu_pred);  % Compute Jacobian of depth measurement (Kx3)
% disp(size(Q_depth));
% disp(size(H_depth));
% disp(size(sigma_pred));
% disp(size(H_depth * sigma_pred * H_depth'))
K_depth = sigma_pred * H_depth' / ((H_depth * sigma_pred * H_depth') + Q_depth); % (3xK)

% Ensure `z_depth` is a (Kx1) column vector
if size(z_depth, 2) > 1
    z_depth = z_depth(:);
end

% Compute expected depth measurement
h_depth_pred = depthMeasurement(mu_pred);  % Should be (Kx1)

% Compute innovation term
innovation = z_depth - h_depth_pred;  % (Kx1)

% **Fix: Scale Down Y and Theta Kalman Gains**
K_depth(2, :) = K_depth(2, :) * 0.5;  % Reduce correction in Y
K_depth(3, :) = K_depth(3, :) * 0.5;  % Reduce correction in Theta

% **Fix: Take Mean Instead of Sum to Prevent Overcorrection**
mu_update = K_depth * innovation;  % (3xK) * (Kx1) = (3x1)
mu_next_depth = mu_pred + mean(mu_update, 2);  % Ensure (3x1) output

sigma_next_depth = (eye(3) - K_depth * H_depth) * sigma_pred;  % (3x3)

% Debugging (Optional)
% disp('H_depth = '); disp(H_depth);
% disp('K_depth = '); disp(K_depth);
% disp('Innovation = '); disp(innovation);

end