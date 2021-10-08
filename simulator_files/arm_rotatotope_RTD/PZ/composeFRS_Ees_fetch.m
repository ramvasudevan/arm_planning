function [joint_pos_error, duration_t] = composeFRS_Ees_fetch(timeids, PZ_reachset, T, axes)
% compose slicable joint FRS given JRS as polynomial zonotopes

num_joints = size(axes,2);

%% compose FRS through kinematics chain
joint_pos_error = cell(length(PZ_reachset),1);

start_t = tic;
for i = timeids
    disp(i)
    joint_pos_error{i} = RobotForwardKinematics_Ees_fetch(num_joints, PZ_reachset{i}, T, axes);
end
duration_t = toc(start_t);
