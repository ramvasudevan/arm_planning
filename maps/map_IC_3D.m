function [theta1, theta2, theta3, theta4] = map_IC_3D(u1, u2)
% This function takes in the shape space representation of a two link 4DOF
% arm in 3D, and returns the joint variables.

% We solve for theta4 as follows:
%   cos(theta4) = <u1, u2>
%   sin(theta4) = ||u1 x u2||
%   theta4 = atan2(sin(theta4)/cos(theta4))
%   
%   Note that in this case, theta4 is the angle between u1 and u2 as we 
%   have restricted our kinematics to one rotation about the current x
%   - axis.
%
% We solve for theta1, theta2, and theta3 as follows:
%
% Let h denote a unit vector representing link 1 in home position.
%   h = [0 1 0]'
% We seek R,k,theta s.t. Rh = u1, where R = Rot(k, theta)
% Let k = h x u1
% Let cos(theta) = <h, u1>, sin(theta) = ||k||
% By Rodrigues' formula:
%   R = I + sin(theta)*K + (1- cos(theta))*K^2,
%   where K = [k]x
% Letting R = Rot(z,theta1) * Rot(y, theta2) * Rot(x, theta3),
% We have:
%   theta2 = arcsin(R(3,1))
%   theta3 = atan2(R(3,2)/R(3,3))
%   theta1 = atan2(R(2,1)/R(1,1))

% TODO singularities if cos(theta4) = R(3,3) = R(1,1) = zero. Might want
% to switch to a quaternion based method.

%% Solve for theta4

cos_theta4 = dot(u1,u2);
sin_theta4 = norm(cross(u1,u2));
theta4 = atan2(sin_theta4/cos_theta4);

%% Solve for theta1, theta2, and theta3

h = [0 1 0]';
k = cross(h, u1);
cos_theta = dot(h, u1);
sin_theta = norm(k);

K = [0 -k(3) k(2);
    k(3) 0 -k(1);
    -k(2) k(1) 0];

R = eye(3) + sin_theta * K + (1 - cos_theta) * K * K;
R_33  = R(3,3);
R_11 = R(1,1);
if (R_33 ~= 0) and (R_11 ~= 0)
    theta2 = asin(R(3,1));
    theta3 = atan2(R(3,2)/R_33);
    theta1 = atan2(R(2,1)/R_11);
    
else 
    theta2 = NaN;
    theta3 = NaN;
    theta1 = NaN;
end

end

