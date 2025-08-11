function gridProb = gridLocalizationStationary(gridProb, map, measurements)
% gridLocalizationStationary - Updates the belief distribution over the grid
%
%   INPUTS:
%       gridProb       - n x m matrix representing prior probability distribution
%       map           - N x 4 matrix describing wall coordinates [x1, y1, x2, y2]
%       measurements  - M x 4 matrix (North, East, South, West distances)
%
%   OUTPUT:
%       gridProb      - Updated probability distribution over the grid

[n, m] = size(gridProb); % Grid dimensions

% Define noise standard deviations
sigma_N = 0.12; % North & South
sigma_E = 0.32; % East & West

% Loop through each grid cell and update based on sensor model
for x = 1:n
    for y = 1:m
        likelihood = 1; % Initialize likelihood

        % Loop through all sensor readings
        for k = 1:size(measurements, 1)
            meas = measurements(k, :);
            expected = expectedSensorReading([x, y], map); % Compute expected distances

            % Compute likelihood for each direction
            for dir = 1:4
                if ~isnan(meas(dir)) % Ignore NaN values
                    if dir == 1 || dir == 3  % North & South
                        likelihood = likelihood * normpdf(meas(dir), expected(dir), sigma_N);
                    else  % East & West
                        likelihood = likelihood * normpdf(meas(dir), expected(dir), sigma_E);
                    end
                end
            end
        end

        % Update grid probability
        gridProb(x, y) = gridProb(x, y) * likelihood;
    end
end

% Normalize to ensure it sums to 1
gridProb = gridProb / sum(gridProb(:));

end