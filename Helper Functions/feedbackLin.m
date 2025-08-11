function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques

%   Inputs:
%       cmdVx: input velocity in x direction wrt inertial frame
%       cmdVy: input velocity in y direction wrt inertial frame
%       theta: orientation of the robot
%       epsilon: turn radius

%   Outputs:
%       cmdV: fwd velocity
%       cmdW: angular velocity
%
% ANTONYSELVARAJ JHON LEONARD AR , NIRMAL

% Compute the linear velocity
cmdV = cos(theta) * cmdVx + sin(theta) * cmdVy;

% Compute the angular velocity
cmdW = (-sin(theta) * cmdVx + cos(theta) * cmdVy) / epsilon;

end
