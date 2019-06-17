function [theta1,theta2] = map_IC_2d(u1,u2)

% This function takes in the shape space representation of a 2d arm
% configuration, and returns the two joint variables.

theta1 = atan2(u1(1), u1(2));
theta2 = atan2(u2(1), u2(2))-theta1;
end

