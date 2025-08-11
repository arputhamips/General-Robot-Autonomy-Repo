function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Antonyselvaraj Jhon Leonard Arputha , Nirmal

    K = length(angles);
    depth = inf(K, 1); % Initialize all depths to infinity

    % Extract robot pose
    x_r = robotPose(1);
    y_r = robotPose(2);
    theta_r = robotPose(3);

    % Compute sensor position in global frame
    x_s = x_r + sensorOrigin(1) * cos(theta_r) - sensorOrigin(2) * sin(theta_r);
    y_s = y_r + sensorOrigin(1) * sin(theta_r) + sensorOrigin(2) * cos(theta_r);

    % Loop through each sensor angle
    for k = 1:K
        % Compute ray direction in global frame
        theta_s = theta_r + angles(k);
        x_end = x_s + 100 * cos(theta_s); % Extend the ray for intersection checking
        y_end = y_s + 100 * sin(theta_s);

        % Check intersections with each wall
        for i = 1:size(map, 1)
            % Extract wall endpoints
            x1 = map(i, 1); y1 = map(i, 2);
            x2 = map(i, 3); y2 = map(i, 4);
            % disp(x1);disp(y1);disp(x2);disp(y2);

            % Compute intersection
            denom = (y2 - y1) * (x_end - x_s) - (x2 - x1) * (y_end - y_s);
            if denom == 0
                continue; % Parallel lines
            end

            ua = ((x2 - x1) * (y_s - y1) - (y2 - y1) * (x_s - x1)) / denom;
            ub = ((x_end - x_s) * (y_s - y1) - (y_end - y_s) * (x_s - x1)) / denom;

            if ua >= 0 && ub >= 0 && ub <= 1
                % Compute intersection point
                xi = x_s + ua * (x_end - x_s);
                yi = y_s + ua * (y_end - y_s);
                d = sqrt((xi - x_s)^2 + (yi - y_s)^2);
                d = d  * cos(angles(k));
                depth(k) = min(depth(k), d); % Keep the closest intersection
            end
        end
    end
end




