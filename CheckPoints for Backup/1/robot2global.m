function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   ANTONYSELVARAJ JHON LEONARD AR , NIRMAL 

% Extract pose components
x = pose(1);
y = pose(2);
theta = pose(3);
    
% Rotation matrix
R = [cos(theta), -sin(theta); 
     sin(theta),  cos(theta)];
    
% Transform the point
xyG = (R * xyR')' + [x, y];

end
