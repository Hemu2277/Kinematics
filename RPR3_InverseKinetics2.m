function [d, theta] = RPR3_InverseKinetics2(x, y, rO)
% This should be called RP4_InverseKinetics2. Never got around to change it
% Calculates the iverse kinetics for one leg.
% 
% [d, theta] = RPR3_InverseKinetics(x, y, rO)
% Input: x = Translation in the x direction with regards to the basefram
%        y = Translation in the y direction with regards to the basefram
%        rO = The translation of the revolute joint with regards to the base frame
% 
% Output: [d,theta] = Joint positions of the manipulators joints

if x-rO(1) == 0
    if y > rO(2)
        theta = 90;
    else 
        theta = -90;
    end
    d = y;
else
theta = atan2d((y-rO(2)),(x-rO(1)));
d = (x-rO(1))/cosd(theta);

end
