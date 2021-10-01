function joint_reachable_set = RobotForwardKinematics(num_joints,motors,BaseTM,T,axes)
% modified from Jon's forward kinematics code
% perform forward kinematics according to motor parameterized trajectories 
% num_joints is the number of joints you want to configure
% motors is a polynomial zonotope that contain parameterization of
% reference trajectories of motors
% T, axes, are the robot forward kinematics configuration information

if isa(motors, 'polyZonotope')
    FK = reshape(BaseTM(1:3,:),[12,1]);
else
    if isa(motors, 'interval')
        FK = interval(BaseTM);
    else
        if isa(motors, 'double')
            FK = BaseTM;
        else
            error('Unknown data type');
        end
    end
end
joint_reachable_set = cell(num_joints,1);

% For Digit, we don't have to worry about the last joints (swing toe pitch and
% roll) since we assume that the swing foot is always flat over the ground
for i = 1:num_joints
    dim = find(axes(:,i) ~=0);
    
    if isa(motors, 'polyZonotope')
        angle = getDim(motors,i);
    else
        if isa(motors, 'interval') || isa(motors, 'double')
            angle = motors(i);
        else
        
        end
    end
    
    if dim == 1
        R = rx(axes(dim,i) * angle);
    elseif dim == 2
        R = ry(axes(dim,i) * angle);
    else
        R = rz(axes(dim,i) * angle);
    end
    
    if isa(motors, 'polyZonotope')
        if i == 1
            joint_reachable_set{i} = HMmultiplication(HMmultiplication(FK, reshape(T(1:3,:,i),[12,1])), R);
        else
            joint_reachable_set{i} = HMmultiplication(HMmultiplication(joint_reachable_set{i-1}, reshape(T(1:3,:,i),[12,1])), R);
        end
    else
        if isa(motors, 'interval') || isa(motors, 'double')
            if i == 1
                joint_reachable_set{i} = FK * T(:,:,i) * R;
            else
                joint_reachable_set{i} = joint_reachable_set{i-1} * T(:,:,i) * R;
            end
        else
            
        end
    end
end

end

%% helper functions
function R = rx(th)
    c = cos(th);
    s = sin(th);
    if isa(th, 'double')
        R = [1  0  0  0;
             0  c -s  0;
             0  s  c  0; 
             0  0  0  1];
    else
    if isa(th,'interval')
        R = interval(eye(4));
        R(2,2) = c;
        R(2,3) = -s;
        R(3,2) = s;
        R(3,3) = c;
    else
    if isa(th, 'polyZonotope')
        R = exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd([1;0;0;0], c), s), 0), -s), c), [0;0;0]);
    else
        error('Unknown data type');
    end
    end
    end
end

function R = ry(th)
    c = cos(th);
    s = sin(th);
    if isa(th, 'double')
        R = [c  0  s  0;
             0  1  0  0;
            -s  0  c  0; 
             0  0  0  1];
    else
    if isa(th,'interval')
        R = interval(eye(4));
        R(1,1) = c;
        R(1,3) = s;
        R(3,1) = -s;
        R(3,3) = c;
    else
    if isa(th, 'polyZonotope')
        R = exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd(c, 0), -s), [0;1;0]), s), 0), c), [0;0;0]);
    else
        error('Unknown data type');
    end
    end
    end
end

function R = rz(th)
    c = cos(th);
    s = sin(th);
    if isa(th, 'double')
        R = [c -s 0  0;
             s  c 0  0; 
             0  0 1  0;
             0  0 0  1];
    else
    if isa(th,'interval')
        R = interval(eye(4));
        R(1,1) = c;
        R(1,2) = -s;
        R(2,1) = s;
        R(2,2) = c;
    else
    if isa(th, 'polyZonotope')
        R = exactCartProd(exactCartProd(exactCartProd(exactCartProd(exactCartProd(c, s), 0), -s), c), [0;0;0;1;0;0;0]);
    else
        error('Unknown data type');
    end
    end
    end
end