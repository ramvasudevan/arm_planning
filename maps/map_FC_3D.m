function [u1,u2] = map_FC_3D(theta1,theta2, theta3, theta4, L1, L2)

%   Function that maps from configuration space to shape space for a 2-arm
%   4DOF arm. 
%   
%   In home position, link one and two are parallel and
%   coincide with the global y - axis. We attach a frame to the
%   base joint, with the y - axis coiniciding with link 1, and a a frame to 
%   the second joint, with the y - axis coinciding with the link 2  and a 
%   rotation about the x - axis. 

%   This function takes in four joint angle variables and returns two unit
%   vectors that represent the arm links in shape space.

%   Let theta1 = rotation about the current z - axis
%       theta2 = rotation about the current y - axis
%       theta3 = rotation about the current x - axis
%       theta4 = rotation about the current x axis

%   For the end position of link one, a roll-pitch-yaw rotation order about 
%   the current axis is assumed, followed by a translation along the 
%   current y axis, as follows:
%
%       H1 = Rot(z,theta1) * Rot(y, theta2) * Rot(x, theta3) * Trans(y, L1)

%   For the end position of link two, we additionally rotate by theta4
%   about the current x - axis and translate by L2 along the current Y axis
%   as follows:
%       
%       H2 = Rot(x, theta4) * Trans(y, L2)
%   
%   u1 is therefore the normalized vector representing the position of the
%   end of link 1.
%
%   u2 is the normalized direction vector between the end of link one and
%   the end of link 2.

h = [0 0 0 1]';

Rot_z_theta1 = [cos(theta1) -sin(theta1) 0 0; 
                sin(theta1) cos(theta1) 0 0;
                0 0 1 0;
                0 0 0 1];
            
Rot_y_theta2 = [cos(theta2) 0 sin(theta2) 0;
                0 1 0 0;
                -sin(theta2) 0 cos(theta2) 0;
                0 0 0 1];
                
Rot_x_theta3 = [1 0 0 0
                0 cos(theta3) -sin(theta3) 0; 
                0 sin(theta3) cos(theta3)  0;
                0 0 0 1];
            
Rot_x_theta4 = [1 0 0 0
                0 cos(theta4) -sin(theta4) 0; 
                0 sin(theta4) cos(theta4)  0;
                0 0 0 1];

Trans_ID = [eye(3); 0 0 0]

Trans_y_L1 = [Trans_ID [0 L1 0 1]'] 

Trans_y_L2 = [Trans_ID [0 L2 0 1]'];
H1 = Rot_z_theta1 * Rot_y_theta2 * Rot_x_theta3 * Trans_y_L1;
e1 = H1 * h;
u1 = e1(1:3)/norm(e1(1:3));

e2 = H1 * Rot_x_theta4 * Trans_y_L2 * h;
d = e2 - e1;
u2 = d(1:3)/norm(d(1:3));

end

