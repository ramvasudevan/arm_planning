
function [u1,u2] = map_FC_2d(theta1,theta2)

% This function takes in joint variables for a 2d planar arm, and returns
% the representation of each link in shape space


u1 = [cos(theta1) sin(theta1)];
u2 = [cos(theta1 + theta2) sin(theta1 + theta2)];

end

