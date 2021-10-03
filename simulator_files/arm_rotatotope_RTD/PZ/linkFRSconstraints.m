function [c,ceq,gc,gceq] = linkFRSconstraints(factor, timeids, joint_pos, num_joints, obstacles)
% evaluate the distance between link FRS and obstacles as constraints
% link FRS are computed implicitly from joint FRS
% FRS are sliced over variable factor

ceq = [];
gceq = [];
c = [];
gc = [];

for timeid = timeids
    for j = 1:(num_joints-1)
        if j == 1
            joint_pos_zono1 = interval(zeros(3,1));
            joint_pos_zono1_gradient = zeros(3,length(factor));
        else
            [joint_pos_zono1, joint_pos_zono1_gradient] = sliceFRS(getDim(joint_pos{timeid}{j}, 10:12), factor);
        end
        
        [joint_pos_zono2, joint_pos_zono2_gradient] = sliceFRS(getDim(joint_pos{timeid}{j+1}, 10:12), factor);
        
        % each link FRS is composed of the convex hull of joint_pos_zono1 and joint_pos_zono2
        % approximate joint_pos_zono1 with the radius of joint_pos_zono2
        % so that the convex hull of them can be easily represented as a zonotope
        
        for obs = 1:length(obstacles)
            obstacle_center = obstacles{obs}.Z(:,1);

            % the constant generators in link FRS zonotope are buffered with obstacle generators
            linkFRS_center = (center(joint_pos_zono1) + center(joint_pos_zono2)) / 2;
            linkFRS_generator = (center(joint_pos_zono2) - center(joint_pos_zono1)) / 2;
            buffered_generators = [diag(interval(joint_pos_zono2).rad), obstacles{obs}.Z(:,2:end)];
            
            % delete all zero generators
            buffered_generators(:,sum(abs(buffered_generators),1) == 0) = [];
            
            [Apoly, Bpoly, dApoly, dBpoly] = differentiable_polytope_PH(linkFRS_center, ...
                                                                        linkFRS_generator, ...
                                                                        buffered_generators, ...
                                                                        (joint_pos_zono1_gradient + joint_pos_zono2_gradient) / 2, ...
                                                                        (joint_pos_zono2_gradient - joint_pos_zono1_gradient) / 2, ...
                                                                        6);
                                         
            c = [c; Apoly * obstacle_center - Bpoly];
            gc = [gc, einsum(dApoly, obstacle_center, 'ijk,jl->ikl')' - dBpoly'];
        end
    end
end

end

