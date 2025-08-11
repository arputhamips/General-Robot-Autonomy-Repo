function drawTrajectories()
% This function is to draw truthPose, and grid map infor

% ===================== load dataStore mat ==================
% dataStore = load('dataStore_forMapping.mat');
% dataStore = dataStore.dataStore;
global dataStore;



% ==================== load map information ===================
load('practicemap2025update.mat');
% cornerMap = loopMap;
cornerMap = map;
figure; hold on; grid on;
xlabel('X Position'); ylabel('Y Position');
title('Trajectory and Map Plot');

% Plot the Map
for i = 1:size(cornerMap, 1)
    plot([cornerMap(i,1), cornerMap(i,3)], [cornerMap(i,2), cornerMap(i,4)], 'r-', 'LineWidth', 2,'HandleVisibility', 'off');
end

% ======================== draw grid boundaries ========================== 
% grid_size = 25; % 25 by 25
% 
% x_min = -2.5;
% x_max = 2.5;
% y_min = -2.5;
% y_max = 2.5;
% 
% % Create the meshgrid
% [x, y] = meshgrid(linspace(x_min, x_max, grid_size), linspace(y_min, y_max, grid_size));

% Plot the grid boundaries
% figure;
hold on;

% Plot vertical lines
% for i = 1:grid_size
%     plot([x(1, i), x(end, i)], [y(1, i), y(end, i)], 'k','LineWidth', 1);  % Vertical lines
% end
% 
% % Plot horizontal lines
% for i = 1:grid_size
%     plot([x(i, 1), x(i, end)], [y(i, 1), y(i, end)], 'k','LineWidth', 1);  % Horizontal lines
% end


% ======================= Plot the trajectories ==========================
h1 = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
% hs = scatter(dataStore.truthPose(1,2), dataStore.truthPose(1,3), 100, 'b', 'filled','DisplayName', 'Start Point'); % mark start pint
% % he = scatter(dataStore.truthPose(end,2), dataStore.truthPose(end,3), 100, 'r', 'filled', 'DisplayName', 'End Point'); % mark end point
% h2 = plot(dataStore2.deadReck(:,1), dataStore2.deadReck(:,2), 'r-', 'LineWidth', 1, 'DisplayName', 'Dead Reckoning');

% h3 = plot(dataStore2.ekfMu(:,1), dataStore2.ekfMu(:,2), 'b-', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
% 
% for t = 1:length(dataStore2.ekfSigma)
%     Mu = dataStore2.ekfMu(t, 1:2);  % Extract [x, y] at time t
%     Sigma = dataStore2.ekfSigma{t}(1:2, 1:2);  % Extract 2x2 covariance matrix
%     
%     % Plot 1-Σ ellipse
%     plotCovEllipse(Mu, Sigma);
% end

% Plot the particles (scatter plot)
% ha = scatter(dataStore2.particles{1}(1,:), dataStore2.particles{1}(2,:), 'r', 'filled','DisplayName', 'Initial Particles');  % Red particles
% hb = scatter(dataStore2.particles{end}(1,:), dataStore2.particles{end}(2,:), 'b', 'filled', 'DisplayName', 'Final Particles');  % Blue particles

% x_vals = [];
% y_vals = [];
% for i=1:5
% %     point = highest_weight_point(dataStore2.particles{i+1});
%     point = weight_mean_point(dataStore2.particles{i+1});
%     scatter(point(1), point(2), 'cyan', 'filled'); 
%     x_vals = [x_vals; point(1)]; 
%     y_vals = [y_vals; point(2)];
% end
% Plot the first five highest point. 
% hold on;
% h4 = plot(NaN, NaN,'o', 'Color', 'cyan', 'DisplayName', '1-\Sigma Uncertainty');
% hc = plot(x_vals, y_vals,'LineWidth', 2, 'Color', 'cyan','DisplayName',  'PF Estimate');


% =================== mark bump locations ====================
% bump_detected = (dataStore.bump(:, 2) == 1 | dataStore.bump(:, 3) == 1 | dataStore.bump(:, 7) == 1);
% 
% Step 2: Extract the truthPose corresponding to detected bumps
% truthPose_bumps = dataStore.truthPose(bump_detected, :);  % Extract only rows where bumps are detected

% Step 3: Plot all bump positions at once
% h_bump = plot(truthPose_bumps(:, 2), truthPose_bumps(:, 3), 'o', 'MarkerFaceColor', 'cyan', 'MarkerSize', 8,'DisplayName', 'Bump Triggered');  % Plot all bump points at once


% ==================== legend ================================
legend_handles = [h1];
% hold off;
% Create the legend
legend(legend_handles);
legend('show');


end