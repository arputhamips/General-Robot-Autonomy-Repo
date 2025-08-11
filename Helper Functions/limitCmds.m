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
%   Antonyselvaraj Jhon Leonard Ar , Nirmal

% Compute individual wheel speeds
    V_left = fwdVel - angVel * wheel2Center;
    V_right = fwdVel + angVel * wheel2Center;

    % Find the maximum absolute wheel speed
    maxWheelSpeed = max(abs([V_left, V_right]));

    % Scale velocities only if they exceed maxV
    if maxWheelSpeed > maxV
        scale = maxV / maxWheelSpeed;
        cmdV = fwdVel * scale;
        cmdW = angVel * scale;
    else
        cmdV = fwdVel;
        cmdW = angVel;
    end

end 
