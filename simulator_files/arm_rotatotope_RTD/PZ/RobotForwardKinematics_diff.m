function [joint_reachable_set, d_joint_reachable_set] = RobotForwardKinematics_diff(num_joints,c,s,dc,ds,BaseTM,T,axes)
% modified from Jon's forward kinematics code
% perform forward kinematics according to motor parameterized trajectories 
% num_joints is the number of joints you want to configure
% motors is a polynomial zonotope that contain parameterization of
% reference trajectories of motors
% T, axes, are the robot forward kinematics configuration information

FK = BaseTM;

joint_reachable_set = cell(num_joints,1);

d_joint_reachable_set = cell(num_joints,1);

% For Digit, we don't have to worry about the last joints (swing toe pitch and
% roll) since we assume that the swing foot is always flat over the ground
for i = 1:num_joints-1
    dim = find(axes(:,i) ~=0);
    
    if dim == 1
        [R, dR] = rx(c(i), axes(dim,i) * s(i), axes(dim,i) * dc(i,:), axes(dim,i) * ds(i,:));
    elseif dim == 2
        [R, dR] = ry(c(i), axes(dim,i) * s(i), axes(dim,i) * dc(i,:), axes(dim,i) * ds(i,:));
    else
        [R, dR] = rz(c(i), axes(dim,i) * s(i), axes(dim,i) * dc(i,:), axes(dim,i) * ds(i,:));
    end
    
    if i == 1
        joint_reachable_set{i} = FK * T(:,:,i) * R;
        d_joint_reachable_set{i} = dR;
    else
        joint_reachable_set{i} = joint_reachable_set{i-1} * T(:,:,i) * R;
        d_joint_reachable_set{i} = einsum(d_joint_reachable_set{i-1}, T(:,:,i) * R, 'ijk,jl->ilk') + einsum(joint_reachable_set{i-1} * T(:,:,i), dR, 'ij,jlk->ilk');
    end
end

joint_reachable_set{num_joints} = joint_reachable_set{num_joints-1} * T(:,:,num_joints);
d_joint_reachable_set{num_joints} = einsum(d_joint_reachable_set{num_joints-1}, T(:,:,num_joints), 'ijk,jl->ilk');

end

%% helper functions
function [R, dR] = rx(c, s, dc, ds)
    R = [1  0  0  0;
         0  c -s  0;
         0  s  c  0; 
         0  0  0  1];
     
    numVar = length(dc);
    
    dR = zeros(4,4,numVar);
    dR(2,2,:) = dc;
    dR(2,3,:) = -ds;
    dR(3,2,:) = ds;
    dR(3,3,:) = dc;
end

function [R, dR] = ry(c, s, dc, ds)
    R = [c  0  s  0;
         0  1  0  0;
        -s  0  c  0; 
         0  0  0  1];
     
    numVar = length(dc);
    
    dR = zeros(4,4,numVar);
    dR(1,1,:) = dc;
    dR(1,3,:) = ds;
    dR(3,1,:) = -ds;
    dR(3,3,:) = dc;
end

function [R, dR] = rz(c, s, dc, ds)
    R = [c -s 0  0;
         s  c 0  0; 
         0  0 1  0;
         0  0 0  1];
     
    numVar = length(dc);
    
    dR = zeros(4,4,numVar);
    dR(1,1,:) = dc;
    dR(1,2,:) = -ds;
    dR(2,1,:) = ds;
    dR(2,2,:) = dc;
end