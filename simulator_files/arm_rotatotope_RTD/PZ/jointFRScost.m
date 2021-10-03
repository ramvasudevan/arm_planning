function [f,df] = jointFRScost(factor, timeid, joint_pos, goal)
% evaluate the distance between center of FRS of end effector and the goal
% as the cost function

[joint_pos_zono, joint_pos_zono_gradient] = sliceFRS(getDim(joint_pos{timeid}{end}, 10:12), factor);

diffd = center(joint_pos_zono) - goal;

f = 0.5*norm(diffd)^2;

df = sum(diffd.*joint_pos_zono_gradient,1);

end

