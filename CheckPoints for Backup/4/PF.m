function newParticles = PF(particles, u, depthMeasurements, predFunc, updateFunc, map, n_rs_rays)
% PF: Generic Particle Filter for one iteration using a weighted particle set.
%
%   INPUTS:
%       particles         - (N x 4) matrix of particles, where each row is
%                           [x, y, theta, weight]. (Weights should be initially uniform.)
%       u                 - (2 x 1) control input vector [d; phi]
%       depthMeasurements - (K x 1) vector of depth measurements from the sensor
%       predFunc          - Function handle for the prediction (motion) model.
%                           It accepts a particle state (3x1) and control input u,
%                           and returns a new state (3x1).
%       updateFunc        - Function handle for the measurement update.
%                           It accepts a particle state (3x1), the depth measurements,
%                           the map, and the number of sensor rays, and returns a scalar likelihood.
%       map               - Map of the environment (e.g., matrix of wall segments)
%       n_rs_rays         - Number of depth sensor rays
%
%   OUTPUT:
%       newParticles      - (N x 4) matrix of updated particles [x, y, theta, weight]
%
%   The function performs the following steps:
%     1. Prediction: Update each particle’s state using the motion model.
%     2. Update: Compute the measurement likelihood for each predicted particle,
%                then compute normalized weights.
%     3. Resampling: Resample particles based on the normalized weights.
%
%   Cornell University - Autonomous Mobile Robots
%   Homework 4
%   NIRMAL A J L A

    N = size(particles, 1);  % Number of particles
    predictedParticles = zeros(N, 4);
    weights = zeros(N, 1);
    epsilon = 1e-12;  % Small constant to avoid zero weights
    
    % ----- Prediction Step -----
    for i = 1:N
        % Extract current state (first 3 columns)
        state = particles(i, 1:3)';
        % Predict new state using the provided prediction function (e.g., integrateOdom)
        newState = predFunc(state, u);  % newState is 3x1
        predictedParticles(i, 1:3) = newState';
    end
    
    % ----- Update Step (Linear Domain) -----
    for i = 1:N
        % Compute the likelihood for the particle using updateFunc.
        % updateFunc returns a scalar likelihood.
        L = updateFunc(predictedParticles(i, 1:3)', depthMeasurements, map, n_rs_rays);
        % Add a small constant to avoid zero likelihood
        weights(i) = L + epsilon;
        % disp(weights(i));
    end
    
    % Normalize the weights so that they sum to one
    weights = weights / sum(weights);
    
    % Store the weights in the fourth column:
    predictedParticles(:, 4) = weights;
    
    % ----- Resampling Step -----
    % Resample particle indices based on the normalized weights (with replacement)
    indices = randsample(1:N, N, true, weights);
    newParticles = predictedParticles(indices, :);
    
    % Optionally, you can reset the weights to uniform values (e.g., 1/N)
    % newParticles(:, 4) = 1 / N;
end