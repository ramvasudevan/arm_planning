function [c,ceq,gc,gceq] = linkFRSconstraints_Ees(factor, motors, joint_pos_error, timeids, num_joints, T, axes, obstacles)
% evaluate the distance between link FRS and obstacles as constraints
% link FRS are computed implicitly from joint FRS
% FRS are sliced over variable factor

ceq = [];
gceq = [];
c = [];
gc = [];

for timeid = timeids
    c_inp = zeros(length(factor),1);
    s_inp = zeros(length(factor),1);
    dc_inp = zeros(length(factor),length(factor));
    ds_inp = zeros(length(factor),length(factor));
    for j = 1:length(factor)
        motors{timeid}.id = j;
        [temp_cs, temp_dcs] = sliceFRS(getDim(motors{timeid}, 1:2), factor);
        temp_cs = center(temp_cs);
        c_inp(j) = temp_cs(1);
        s_inp(j) = temp_cs(2);
        dc_inp(j,:) = temp_dcs(1,:);
        ds_inp(j,:) = temp_dcs(2,:);
    end
    
    [joint_reachable_set, d_joint_reachable_set] = RobotForwardKinematics_diff(num_joints,c_inp,s_inp,dc_inp,ds_inp,eye(4),T,axes);
    
    for j = 1:(num_joints-1)
        if j == 1
            joint_pos1 = zeros(3,1);
            joint_pos1_gradient = zeros(3,length(factor));
        else
            joint_pos1 = joint_reachable_set{j}(1:3,4);
            joint_pos1_gradient = squeeze(d_joint_reachable_set{j}(1:3,4,:));
        end
        
        joint_pos2 = joint_reachable_set{j+1}(1:3,4);
        joint_pos2_gradient = squeeze(d_joint_reachable_set{j+1}(1:3,4,:));
        
        % each link FRS is composed of the convex hull of joint_pos1 and joint_pos2
        % approximate joint_pos1 with the radius of joint_pos2
        % so that the convex hull of them can be easily represented as a zonotope
        
        for obs = 1:length(obstacles)
            obstacle_center = obstacles{obs}.Z(:,1);

            % the constant generators in link FRS zonotope are buffered with obstacle generators
            linkFRS_center = (joint_pos1 + joint_pos2) / 2;
            linkFRS_generator = (joint_pos2 - joint_pos1) / 2;
            buffered_generators = [diag([joint_pos_error{timeid}{j+1}{10}.e.rad, joint_pos_error{timeid}{j+1}{11}.e.rad, joint_pos_error{timeid}{j+1}{12}.e.rad]), obstacles{obs}.Z(:,2:end)];
            
            % delete all zero generators
            buffered_generators(:,sum(abs(buffered_generators),1) == 0) = [];
            
            if nargout > 2
                [Apoly, Bpoly, dApoly, dBpoly] = differentiable_polytope_PH(linkFRS_center, ...
                                                                            linkFRS_generator, ...
                                                                            buffered_generators, ...
                                                                            (joint_pos1_gradient + joint_pos2_gradient) / 2, ...
                                                                            (joint_pos2_gradient - joint_pos1_gradient) / 2, ...
                                                                            6);
            else
                [Apoly, Bpoly] = differentiable_polytope_PH(linkFRS_center, ...
                                                            linkFRS_generator, ...
                                                            buffered_generators, ...
                                                            (joint_pos1_gradient + joint_pos2_gradient) / 2, ...
                                                            (joint_pos2_gradient - joint_pos1_gradient) / 2, ...
                                                            6);
            end
            
            cv = Apoly * obstacle_center - Bpoly;
            [cv_max, cv_max_id] = max(cv);
                                         
            c = [c; -cv_max];
            
            if nargout > 2
                gcv = einsum(dApoly, obstacle_center, 'ijk,jl->ikl')' - dBpoly';
                gc = [gc, -gcv(:,cv_max_id)];
            end
        end
    end
end

end

