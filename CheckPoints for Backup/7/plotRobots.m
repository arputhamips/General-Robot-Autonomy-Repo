% Usage example:
% poses = generateInitialPoses(25, [100, 100], 3.0);
% plotRobots(poses);  % We'll define this function below

function plotRobots(poses)
    % Plot robot positions and orientations
    % Input: poses - Nx3 matrix of [x, y, theta]
    
    figure;
    hold on;
    
    % Robot visualization parameters
    robot_size = 1.0;
    arrow_length = 1.5;
    
    for i = 1:size(poses, 1)
        x = poses(i, 1);
        y = poses(i, 2);
        theta = poses(i, 3);
        
        % Draw robot body
        viscircles([x, y], robot_size, 'Color', [0.7, 0.7, 0.7]);
        
        % Draw orientation arrow
        dx = arrow_length * cos(theta);
        dy = arrow_length * sin(theta);
        quiver(x, y, dx, dy, 0, 'LineWidth', 2, 'MaxHeadSize', 1);
        
        % Label with robot ID
        text(x, y, num2str(i), 'HorizontalAlignment', 'center');
    end
    
    axis equal;
    title('Robot Initial Poses');
    xlabel('X Position');
    ylabel('Y Position');
    grid on;
    hold off;
end