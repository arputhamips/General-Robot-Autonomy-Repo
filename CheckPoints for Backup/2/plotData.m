no_config = load('Odo Data/dataStore_nc.mat');
config = load("Odo Data/dataStore_c.mat");

% noconfig data
phi_nc = no_config.dataStore.odometry(:,3);
d_nc = no_config.dataStore.odometry(:,2);
phi_nc = phi_nc';
d_nc = d_nc';

% config data
phi_c = config.dataStore.odometry(:,3);
d_c = config.dataStore.odometry(:,2);
phi_c = phi_c';
d_c = d_c';


% truthpose
tpose_nc = no_config.dataStore.truthPose(:,2:4);
tpose_nc = tpose_nc';
tpose_c = config.dataStore.truthPose(:,2:4);
tpose_c = tpose_c';




initpose = tpose_c(:,1);

trajectory_nc = integrateOdom(initpose, d_nc, phi_nc);
trajectory_c = integrateOdom(initpose, d_c, phi_c);


% plot the graph
figure;
hold on;
grid on;

% Plot each trajectory
plot(tpose_c(1,:), tpose_c(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Overhead Localization'); % Black line for ground truth
plot(trajectory_nc(1,:), trajectory_nc(2,:), 'r:', 'LineWidth', 2, 'DisplayName', 'Odometry with Noise'); % Red dashed line
plot(trajectory_c(1,:), trajectory_c(2,:), 'b:', 'LineWidth', 2, 'DisplayName', 'Odometry without Noise'); % Blue dash-dot line

% Labels and legend
xlabel('X Position');
ylabel('Y Position');
title('Trajectory Comparison');
legend('Location', 'best');
axis equal; % Ensures equal scaling
% view(3); % Set to 3D view

hold off;