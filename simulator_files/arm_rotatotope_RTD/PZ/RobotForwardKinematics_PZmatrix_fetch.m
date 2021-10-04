function joint_reachable_set = RobotForwardKinematics_PZmatrix_fetch(num_joints,JRS,T,axes)
% modified from Jon's forward kinematics code
% perform forward kinematics according to motor parameterized trajectories 
% num_joints is the number of joints you want to configure
% motors is a polynomial zonotope that contain parameterization of
% reference trajectories of motors
% T, axes, are the robot forward kinematics configuration information

% represent a 4x4 homogeinous transformation matrix as a 12x1 vector,
% ignore the last row since they are always [0 0 0 1].
FK = reshape([1,0,0,0;0,1,0,0;0,0,1,0],[12,1]);

joint_reachable_set = cell(num_joints,1);

for i = 1:num_joints-1
    dim = find(axes(:,i) ~=0);
    
%     motors = reduceFactorsFull(JRS, 1);
    motors = JRS;
    motors.Grest = diag(sum(abs(motors.Grest),2));
    motors.id = i;
    
    cosangle = getDim(motors,1);
    sinangle = getDim(motors,2);
    
    if dim == 1
        R = rx(axes(dim,i) * cosangle, axes(dim,i) * sinangle);
    elseif dim == 2
        R = ry(axes(dim,i) * cosangle, axes(dim,i) * sinangle);
    else
        R = rz(axes(dim,i) * cosangle, axes(dim,i) * sinangle);
    end
    
    if i == 1
        joint_reachable_set{i} = HMmultiplication(HMmultiplication(FK, reshape(T(1:3,:,i),[12,1])), R);
    else
        joint_reachable_set{i} = HMmultiplication(HMmultiplication(joint_reachable_set{i-1}, reshape(T(1:3,:,i),[12,1])), R);
    end
end

% position of the end effector, ignore orientation for now
joint_reachable_set{num_joints} = HMmultiplication(joint_reachable_set{num_joints-1}, reshape(T(1:3,:,num_joints),[12,1]));

end

%% helper functions
function R = rx(th, th2)
    if nargin == 1
        c = reduceFactorsFull(cos(th));
        s = reduceFactorsFull(sin(th));
    else
        c = th;
        s = th2;
    end
    
    if isa(th, 'double')
        R = [1  0  0  0;
             0  c -s  0;
             0  s  c  0; 
             0  0  0  1];
    else
        R = cell(12,1);
        R{1} = 1;
        R{2} = 0;
        R{3} = 0;
        R{4} = 0;
        R{5} = PZmatrix(c);
        R{6} = PZmatrix(s);
        R{7} = 0;
        R{8} = PZmatrix(-s);
        R{9} = PZmatrix(c);
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
    end
end

function R = ry(th, th2)
    if nargin == 1
        c = reduceFactorsFull(cos(th));
        s = reduceFactorsFull(sin(th));
    else
        c = th;
        s = th2;
    end
    
    if isa(th, 'double')
        R = [c  0  s  0;
             0  1  0  0;
            -s  0  c  0; 
             0  0  0  1];
    else
        R = cell(12,1);
        R{1} = PZmatrix(c);
        R{2} = 0;
        R{3} = PZmatrix(-s);
        R{4} = 0;
        R{5} = 1;
        R{6} = 0;
        R{7} = PZmatrix(s);
        R{8} = 0;
        R{9} = PZmatrix(c);
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
    end
end

function R = rz(th, th2)
    if nargin == 1
        c = reduceFactorsFull(cos(th));
        s = reduceFactorsFull(sin(th));
    else
        c = th;
        s = th2;
    end
    
    if isa(th, 'double')
        R = [c -s 0  0;
             s  c 0  0; 
             0  0 1  0;
             0  0 0  1];
    else
        R = cell(12,1);
        R{1} = PZmatrix(c);
        R{2} = PZmatrix(s);
        R{3} = 0;
        R{4} = PZmatrix(-s);
        R{5} = PZmatrix(c);
        R{6} = 0;
        R{7} = 0;
        R{8} = 0;
        R{9} = 1;
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
    end
end