function [joint_pos, duration_t] = composeFRS_PZmatrix_fetch(timeids, PZ_reachset, T, axes)
% compose slicable joint FRS given JRS as polynomial zonotopes

num_joints = size(axes,2);

%% compose FRS through kinematics chain
joint_reachable_set = cell(length(PZ_reachset),1);

start_t = tic;
% for i = 1:length(PZ_reachset.Rcont)
for i = timeids
    disp(i)
    joint_reachable_set{i} = RobotForwardKinematics_PZmatrix_fetch(num_joints,PZ_reachset{i},T,axes);
end
duration_t = toc(start_t);

%% simplify FRS
joint_pos = cell(length(PZ_reachset),1);

for i = timeids
    joint_pos{i} = cell(num_joints,1);
    for j = 2:num_joints
        joint_pos{i}{j} = joint_reachable_set{i}{j}{1}.toPolyZonotope;
        
        for k = 2:12
            % convert a PZmatrix class back to CORA polyZonotope for slicing
            if isa(joint_reachable_set{i}{j}{k}, 'PZmatrix')
                joint_pos{i}{j} = exactCartProd(joint_pos{i}{j}, joint_reachable_set{i}{j}{k}.toPolyZonotope);
            else
                joint_pos{i}{j} = exactCartProd(joint_pos{i}{j}, joint_reachable_set{i}{j}{k});
            end
        end
    end
end

