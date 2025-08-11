function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (cumulated pose)
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2

    cur_x = initPose(1);
    cur_y = initPose(2);
    cur_theta = initPose(3);

    N = size(d,2);
    
    finalPose = zeros(3,N);

    % compute one by one
    for i = 1:N
        if abs(phi(i)) < 1e-6
            finalPose(1,i) = cur_x + d(i) *cos(cur_theta);
            finalPose(2,i) = cur_y + d(i)*sin(cur_theta);
            finalPose(3,i) = cur_theta;
        else
            R = d(i) / phi(i);
            finalPose(1,i) = cur_x + R *sin(cur_theta +phi(i)) - R*sin(cur_theta);
            finalPose(2,i) = cur_y + R*cos(cur_theta) - R*cos(cur_theta + phi(i));
            finalPose(3,i) = cur_theta + phi(i);
        end
        
        cur_x = finalPose(1,i);
        cur_y = finalPose(2,i);
        cur_theta = finalPose(3,i);
        
    end
end
