% going to try stacking two 3D rotatotopes together...

clear all; clc;

% q_0 = zeros(4, 1);
% q_0 = [0; 0; 0; 0; 0; 0];
q_0 = pi/6*ones(6, 1);
q_dot_0 = ones(6, 1)*0.1;
figure(2); clf; hold on; view(3); axis equal;

slice_pt = pi/6-0.001;
% slice_pt = 0;

tic
FRS = robot_arm_FRS_rotatotope_fetch(q_0, q_dot_0);
toc
% FRS.plot(10)
FRS.plot_slice(ones(6,1)*slice_pt, 10)

pause(0.05);

% xlim([-0.66, 0.66]);
% ylim([-0.66, 0.66]);
% zlim([-0.66, 0.66]);

xlim([-1, 1]);
ylim([-1, 1]);
zlim([-1, 1]);

box on;
xlabel('X');