function [x,y] = RPR3_ForwardKinetics2(theta, d, rO)
% This should be called RP4_ForwardKinetics2. Never got around to change it
% Calculates the forward kinetics for one leg.
% 
% [x,y] = RPR3_ForwardKinetics(theta, d, rO)
% Input: theta = Angle of the revolute joint
%        d = The displacement of the prismatic joint
%        rO = The translation of the revolute joint with regards to the base frame
% 
% Output: [x,y] = Position of the end affector with regards to the base frame

x = rO(1,1) + d*cosd(theta);
y = rO(1,2) + d*sind(theta);
end
