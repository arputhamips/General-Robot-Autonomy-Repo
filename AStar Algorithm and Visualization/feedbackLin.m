function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques
% Inputs:
%  cmdVx: input velocity in x direction wrt inertial frame
%  cmdVy: input velocity in y direction wrt inertial frame
%  theta: orientation of the robot
%  epsilon: turn radius
% Outputs:
%  cmdV: fwd velocity
%  cmdW: angular velocity
%
% Zheng, Chengle
    cmdV = cmdVx*cos(theta) + cmdVy*sin(theta);
    if(theta == 0)
        cmdW = cmdVy/epsilon;
    else
        cmdW = (cmdV*cos(theta) - cmdVx)/ (epsilon*sin(theta));
    end
end