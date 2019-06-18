% Script for testing FC and IC maps in 2D and 3D

% tests for map_FC_3D

L1 = 5;
L2 = 3;

% Check each rotation

theta1 = 0;
theta2 = 0;
theta3 = pi/2;
theta4 = 0;

[u1, u2] = map_FC_3D(theta1, theta2, theta3, theta4, L1, L2)

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = pi/2;

[u1, u2] = map_FC_3D(theta1, theta2, theta3, theta4, L1, L2)