function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   ANTONYSELVARAJ JHON LEONARD AR, NIRMAL


% Extract pose components
xR = pose(1);
yR = pose(2);
theta = pose(3);
    
% Rotation matrix
R = [cos(theta), -sin(theta); 
    sin(theta),  cos(theta)];
    
% Transform the point
xyR = R' * (xyG - [xR, yR])';
xyR = xyR';


end