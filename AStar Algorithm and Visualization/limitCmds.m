function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Zheng, Chengle
    % compute the left/right wheel velocity
    V_left = fwdVel - wheel2Center*angVel;
    V_right = fwdVel + wheel2Center*angVel;
    
    % is the wheel velocity larger than maxV?
    desired_max = max(abs(V_left), abs(V_right));

    if (desired_max >maxV)
        scale = maxV/desired_max;
        cmdV = scale*fwdVel;
        cmdW = scale*angVel;
    else
        cmdV = fwdVel;
        cmdW = angVel;
    end



end
