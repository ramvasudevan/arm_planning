% script for generating IK functions

% basically, creating a bunch of functions that return the joint locations
% x_i, y_i, z_i of the i-th joint as a function of the fetch generalized
% coordinates.

% fetch rotation order...
% q1 about 3
% q2 about 2
% q3 about 1
% q4 about 2
% q5 about 1
% q6 about 2

syms x1 x2 x3 y1 y2 y3 z1 z2 z3 q1 q2 q3 q4 q5 q6 real;
L = 0.33;

Owa = make_orientation(q1, 3);
Oab = make_orientation(q2, 2);
Obc = make_orientation(q3, 1);
Ocd = make_orientation(q4, 2);
Ode = make_orientation(q5, 1);
Oef = make_orientation(q6, 2);

origin = [0;0;0];

X1 = origin + Owa*Oab*[L;0;0] - [x1;y1;z1];
X2 = origin + Owa*Oab*[L;0;0] + Owa*Oab*Obc*Ocd*[L;0;0] - [x2;y2;z2];
X3 = origin + Owa*Oab*[L;0;0] + Owa*Oab*Obc*Ocd*[L;0;0] + Owa*Oab*Obc*Ocd*Ode*Oef*[L;0;0] - [x3;y3;z3];

J1 = jacobian(X1, [q1;q2]);
J2 = jacobian(X2, [q3;q4]);
J3 = jacobian(X3, [q5;q6]);

matlabFunction(X1, J1, 'File', 'joint1_IK', 'Outputs', {'E', 'J'}, 'Vars', {[x1;y1;z1], [q1;q2]});
matlabFunction(X2, J1, 'File', 'joint2_IK', 'Outputs', {'E', 'J'}, 'Vars', {[x2;y2;z2], [q1;q2], [q3;q4]});
matlabFunction(X3, J1, 'File', 'joint3_IK', 'Outputs', {'E', 'J'}, 'Vars', {[x3;y3;z3], [q1;q2;q3;q4], [q5;q6]});


