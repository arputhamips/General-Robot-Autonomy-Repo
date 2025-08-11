function [gridProb, muFinal, sigmaFinal] = TestFunHW3(map, measurements, gridSize, mu0, sigma0)
% Test function for Homework 3.  
% This function checks the student's grid localization and stationary KF implementation.
% 
%   INPUTS
%       map       	    N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       measurements	M-by-4 matrix containing the range measurements 
%                       in the global North, East, South and West directions
%       gridSize 	    Number of cells in the X dimension x number of 
%                       cells in the Y dimension     1x2 [n m]
%       mu0             The initial expected position for Stationary KF 2x1
%       sigma0          initial variance of the position estimate
%
%   OUTPUTS
%       gridProb      	n x m matrix representing the probability of the
%                       robot being in a certain grid cell
%       muFinal      	final expected position for Stationary KF 2x1
%       sigmaFinal      final variance of the position estimate

%~~~~~~~~~~~~~~~~~~~~~
% Grid localization
%~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Create the grid based on the input gridSize


% STUDENTS: Call the function gridLocalizationStationary to get the updated pdf


%~~~~~~~~~~~~~~~~~~~~~
% Stationary KF
%~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Call the function KFStationary to get the updated pdf with the given initial position


end