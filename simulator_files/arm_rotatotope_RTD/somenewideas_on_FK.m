% run somenewideas.m first.

num_joints = 14;
load('digit_FK_info_left_pinned.mat');

cosq = interval(zeros(num_joints,1));
sinq = interval(zeros(num_joints,1));
cose = interval(zeros(num_joints,1));
sine = interval(zeros(num_joints,1));
for i = 1:num_joints
    p0_v = pi - 2*pi*rand;
    v0_v = pi - 2*pi*rand;
    k_range = interval(-abs(v0_v)/3,abs(v0_v)/3);
    k_v = randPoint(k_range);
    tmid_v = (i - 0.5) * dt;
    t_range = tmid_v + interval(-dt/2,dt/2);
    
    cos_error_bound = cos_traj_2_d_k_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * k_range^k_order + cos_traj_2_d_t_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * interval(-dt/2,dt/2)^t_order;
    sin_error_bound = sin_traj_2_d_k_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * k_range^k_order + sin_traj_2_d_t_LagrangeRemainder(k_range, p0_v, t_range, v0_v) * interval(-dt/2,dt/2)^t_order;   
    cos_center = double(subs(cos_traj_2_poly, tmid, tmid_v));
    sin_center = double(subs(sin_traj_2_poly, tmid, tmid_v));
    
    cose(i) = cos_error_bound;
    sine(i) = sin_error_bound;
    cosq(i) = cos_center + cos_error_bound;
    sinq(i) = sin_center + sin_error_bound;
end

joint_id = 14;

% bound only using gradient
errorbound1 = interval(zeros(4,4));
grad_FK1 = get_gradient(cosq, sinq, T, axes, joint_id);

for mi = 1:4
    for mj = 1:4
        for i = 1:num_joints
            errorbound1(mi,mj) = errorbound1(mi,mj) + grad_FK1{mi,mj}(i) * cose(i) + grad_FK1{mi,mj}(i + num_joints) * sine(i);
        end
    end
end

% bound only using gradient and hessian
errorbound2 = interval(zeros(4,4));
grad_FK2 = get_gradient(cosq.center, sinq.center, T, axes, joint_id);
hess_FK = get_hessian(cosq, sinq, T, axes, num_joints);

for mi = 1:4
    for mj = 1:4
        for i = 1:num_joints
            errorbound2(mi,mj) = errorbound2(mi,mj) + grad_FK2{mi,mj}(i) * cose(i) + grad_FK2{mi,mj}(i + num_joints) * sine(i);
        end
        for i = 1:num_joints
            for j = 1:num_joints
                errorbound2(mi,mj) = errorbound2(mi,mj) + (1/2) * hess_FK{mi,mj}(i,j) * cose(i) * cose(j) ...
                                                        + (1/2) * hess_FK{mi,mj}(i + num_joints,j) * sine(i) * cose(j) ...
                                                        + (1/2) * hess_FK{mi,mj}(i,j + num_joints) * cose(i) * sine(j) ...
                                                        + (1/2) * hess_FK{mi,mj}(i + num_joints,j + num_joints) * sine(i) * sine(j);
            end
        end
    end
end

function FK = get_FK(c, s, T, axes, num_joints) 
    % id implies cos(x_id) or sin(x_id)
    % csflag implies this is a cos or sin
    FK = eye(4);
    
    for i = 1:num_joints
        dim = find(axes(:,i) ~= 0);
        
        if dim == 1
            [R, ~, ~] = rx(c(i), s(i));
        elseif dim == 2
            [R, ~, ~] = ry(c(i), s(i));
        else
            [R, ~, ~] = rz(c(i), s(i));
        end
        
        FK = FK * T(:,:,i) * R;
    end
end

function grad_FK = get_gradient(cosq, sinq, T, axes, num_joints)
    grad_FK = cell(4,4);
    
    if isa('cosq', 'double')
        for i = 1:4
            for j = 1:4
                grad_FK{i,j} = zeros(1,num_joints);
            end
        end
    elseif isa('cosq', 'interval')
        for i = 1:4
            for j = 1:4
                grad_FK{i,j} = interval(zeros(1,num_joints));
            end
        end
    end

    for i = 1:num_joints
        grad_c = get_grad_FK(cosq, sinq, T, axes, num_joints, i, 'c');
        grad_s = get_grad_FK(cosq, sinq, T, axes, num_joints, i, 's');
        
        for mi = 1:4
            for mj = 1:4
                grad_FK{mi,mj}(i) = grad_c(mi,mj);
                grad_FK{mi,mj}(i + num_joints) = grad_s(mi,mj);
            end
        end
    end
end

function grad = get_grad_FK(c, s, T, axes, num_joints, id, csflag) 
    % id implies cos(x_id) or sin(x_id)
    % csflag implies this is a cos or sin
    FK = eye(4);
    
    for i = 1:num_joints
        dim = find(axes(:,i) ~= 0);
        
        if dim == 1
            [R, dR_c, dR_s] = rx(c(i), s(i));
        elseif dim == 2
            [R, dR_c, dR_s] = ry(c(i), s(i));
        else
            [R, dR_c, dR_s] = rz(c(i), s(i));
        end
        
        if i ~= id
            FK = FK * T(:,:,i) * R;
        else
            if csflag == 'c'
                FK = FK * T(:,:,i) * dR_c;
            else
                FK = FK * T(:,:,i) * dR_s;
            end
        end
    end
    
    grad = FK;
end

function hess_FK = get_hessian(cosq, sinq, T, axes, num_joints)
    hess_FK = cell(4,4);
    
    if isa('cosq', 'double')
        for i = 1:4
            for j = 1:4
                hess_FK{i,j} = zeros(1,num_joints,num_joints);
            end
        end
    elseif isa('cosq', 'interval')
        for i = 1:4
            for j = 1:4
                hess_FK{i,j} = interval(zeros(1,num_joints,num_joints));
            end
        end
    end

    for i = 2:num_joints
        for j = 1:(i-1)
            hess_cc = get_hess_FK(cosq, sinq, T, axes, num_joints, i, 'c', j, 'c');
            hess_sc = get_hess_FK(cosq, sinq, T, axes, num_joints, i, 's', j, 'c');
            hess_cs = get_hess_FK(cosq, sinq, T, axes, num_joints, i, 'c', j, 's');
            hess_ss = get_hess_FK(cosq, sinq, T, axes, num_joints, i, 's', j, 's');
            
            for mi = 1:4
                for mj = 1:4
                    hess_FK{mi,mj}(i,j) = hess_cc(mi,mj);
                    hess_FK{mi,mj}(i + num_joints,j) = hess_sc(mi,mj);
                    hess_FK{mi,mj}(i,j + num_joints) = hess_cs(mi,mj);
                    hess_FK{mi,mj}(i + num_joints,j + num_joints) = hess_ss(mi,mj);
                    hess_FK{mi,mj}(j,i) = hess_cc(mi,mj);
                    hess_FK{mi,mj}(j,i + num_joints) = hess_sc(mi,mj);
                    hess_FK{mi,mj}(j + num_joints,i) = hess_cs(mi,mj);
                    hess_FK{mi,mj}(j + num_joints,i + num_joints) = hess_ss(mi,mj);
                end
            end
        end
    end
end

function hess = get_hess_FK(c, s, T, axes, num_joints, id1, csflag1, id2, csflag2) 
    % id implies cos(x_id) or sin(x_id)
    % csflag implies this is a cos or sin
    FK = eye(4);
    
    for i = 1:num_joints
        dim = find(axes(:,i) ~= 0);
        
        if dim == 1
            [R, dR_c, dR_s] = rx(c(i), s(i));
        elseif dim == 2
            [R, dR_c, dR_s] = ry(c(i), s(i));
        else
            [R, dR_c, dR_s] = rz(c(i), s(i));
        end
        
        if i ~= id1 && i ~= id2
            FK = FK * T(:,:,i) * R;
        else
            if i == id1
                csflag = csflag1;
            else
                csflag = csflag2;
            end

            if csflag == 'c'
                FK = FK * T(:,:,i) * dR_c;
            else
                FK = FK * T(:,:,i) * dR_s;
            end
        end
    end
    
    hess = FK;
end

function [R, dR_c, dR_s] = rx(c,s)
    if isa(c, 'double')
        R = [1  0  0  0;
             0  c -s  0;
             0  s  c  0; 
             0  0  0  1];
    elseif isa(c,'interval')
        R = interval(eye(4));
        R(2,2) = c;
        R(2,3) = -s;
        R(3,2) = s;
        R(3,3) = c;
    end
    
    dR_c = [0 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 0];

    dR_s = [0 0 0 0;
            0 0 -1 0;
            0 1 0 0;
            0 0 0 0];
end

function [R, dR_c, dR_s] = ry(c,s)
    R = interval(eye(4));
    R(1,1) = c;
    R(1,3) = s;
    R(3,1) = -s;
    R(3,3) = c;

    if isa(c, 'double')
        R = [c  0  s  0;
             0  1  0  0;
            -s  0  c  0; 
             0  0  0  1];
    elseif isa(c,'interval')
        R = interval(eye(4));
        R(1,1) = c;
        R(1,3) = s;
        R(3,1) = -s;
        R(3,3) = c;
    end

    dR_c = [1 0 0 0;
            0 0 0 0;
            0 0 1 0;
            0 0 0 0];

    dR_s = [0 0 1 0;
            0 0 0 0;
           -1 0 0 0;
            0 0 0 0];
end

function [R, dR_c, dR_s] = rz(c,s)
    if isa(c, 'double')
        R = [c -s 0  0;
             s  c 0  0; 
             0  0 1  0;
             0  0 0  1];
    elseif isa(c,'interval')
        R = interval(eye(4));
        R(1,1) = c;
        R(1,2) = -s;
        R(2,1) = s;
        R(2,2) = c;
    end

    dR_c = [1 0 0 0;
            0 1 0 0;
            0 0 0 0;
            0 0 0 0];

    dR_s = [0 -1 0 0;
            1 0 0 0;
            0 0 0 0;
            0 0 0 0];
end