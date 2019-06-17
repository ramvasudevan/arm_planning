function [ ] = generate_arm_vol_dynamics_absolute( )
% generate some simple 2D arm dynamics for the volume computation problem

% syms x y c1 c12 c123 s1 s12 s123 s real
syms x y c1 c2 s1 s2 s real;

% z = [x;y;c1;c12;c123;s1;s12;s123;s];
% dz1 = [c1;s1;0;0;0;0;0;0;1];
% dz2 = [c12;s12;0;0;0;0;0;0;1];
% dz3 = [c123;s123;0;0;0;0;0;0;1];
z = [x;y;c1;s1;c2;s2;s];
dz1 = [c1;s1;0;0;0;0;0];
dz2 = [c2;s2;0;0;0;0;0];

syms tdummy udummy real

matlabFunction(dz1, 'File', 'arm_vol_dyn_abs_l1', 'vars', {tdummy z udummy});
matlabFunction(dz2, 'File', 'arm_vol_dyn_abs_l2', 'vars', {tdummy z udummy});
% matlabFunction(dz3, 'File', 'arm_vol_dyn_rel_l3', 'vars', {tdummy z udummy});

end

