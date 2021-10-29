function joint_reachable_set_error = RobotForwardKinematics_Ees_fetch(num_joints,JRS,T,axes)
% modified from Jon's forward kinematics code
% perform forward kinematics according to motor parameterized trajectories 
% num_joints is the number of joints you want to configure
% motors is a polynomial zonotope that contain parameterization of
% reference trajectories of motors
% T, axes, are the robot forward kinematics configuration information

% represent a 4x4 homogeinous transformation matrix as a 12x1 vector,
% ignore the last row since they are always [0 0 0 1].

FK = reshape([1,0,0,0;0,1,0,0;0,0,1,0],[12,1]);

joint_reachable_set_error = cell(num_joints,1);

for i = 1:num_joints-1
    dim = find(axes(:,i) ~=0);
    
%     motors = JRS{i};
    motors = JRS;
    motors.Grest = diag(sum(abs(motors.Grest),2));
    motors.id = i;
    
    cosangle = getDim(motors,1);
    sinangle = getDim(motors,2);
    
    ce = interval(-cosangle.Grest, cosangle.Grest);
    se = interval(-sinangle.Grest, sinangle.Grest);
    cosangle.Grest = [];
    cE = interval(cosangle);
    sinangle.Grest = [];
    sE = interval(sinangle);
    
    disp([cE.rad,sE.rad])
    
    if dim == 1
        R = rx(cE, ce, sE, se);
    elseif dim == 2
        R = ry(cE, ce, sE, se);
    else
        R = rz(cE, ce, sE, se);
    end
    
    if i == 1
        joint_reachable_set_error{i} = HMmultiplication(HMmultiplication(FK, reshape(T(1:3,:,i),[12,1])), R);
    else
        joint_reachable_set_error{i} = HMmultiplication(HMmultiplication(joint_reachable_set_error{i-1}, reshape(T(1:3,:,i),[12,1])), R);
    end
end

% position of the end effector, ignore orientation for now
joint_reachable_set_error{num_joints} = HMmultiplication(joint_reachable_set_error{num_joints-1}, reshape(T(1:3,:,num_joints),[12,1]));

% %% verification
% for num = 1:1000
% factor = 2 * rand(6,1) - 1;
% motor = zeros(7,1);
% motor_sample = motor;
% for i = 1:6
%     JRS{i}.id = 1;
%     slice_motor = sliceFRS(getDim(JRS{i},[1:2]), factor(i));
%     cen = center(slice_motor);
%     motor(i) = atan2(cen(2), cen(1));
%     ran = randPoint(slice_motor);
%     motor_sample(i) = atan2(ran(2), ran(1));
% end
% 
% joint_reachable_set_center = RobotForwardKinematics(num_joints,motor,eye(4),T,axes);
% joint_reachable_set_sample = RobotForwardKinematics(num_joints,motor_sample,eye(4),T,axes);
% 
% for i = 3:7
% if ~(joint_reachable_set_center{i}(1,4) + joint_reachable_set_error{i}{10}.e.rad >= joint_reachable_set_sample{i}(1,4) && ...
%         joint_reachable_set_center{i}(1,4) - joint_reachable_set_error{i}{10}.e.rad <= joint_reachable_set_sample{i}(1,4))
%     disp('error')
% end
% if ~(joint_reachable_set_center{i}(2,4) + joint_reachable_set_error{i}{11}.e.rad >= joint_reachable_set_sample{i}(2,4) && ...
%         joint_reachable_set_center{i}(2,4) - joint_reachable_set_error{i}{11}.e.rad <= joint_reachable_set_sample{i}(2,4))
%     disp('error')
% end
% if ~(joint_reachable_set_center{i}(3,4) + joint_reachable_set_error{i}{12}.e.rad >= joint_reachable_set_sample{i}(3,4) && ...
%         joint_reachable_set_center{i}(3,4) - joint_reachable_set_error{i}{12}.e.rad <= joint_reachable_set_sample{i}(3,4))
%     disp('error')
% end
% end
% end
end

%% helper functions
function R = rx(cE, ce, sE, se)
        R = cell(12,1);
        R{1} = 1;
        R{2} = 0;
        R{3} = 0;
        R{4} = 0;
        R{5} = Ees(cE, ce);
        R{6} = Ees(sE, se);
        R{7} = 0;
        R{8} = Ees(-sE, -se);
        R{9} = Ees(cE, ce);
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
end

function R = ry(cE, ce, sE, se)
        R = cell(12,1);
        R{1} = Ees(cE, ce);
        R{2} = 0;
        R{3} = Ees(-sE, -se);
        R{4} = 0;
        R{5} = 1;
        R{6} = 0;
        R{7} = Ees(sE, se);
        R{8} = 0;
        R{9} = Ees(cE, ce);
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
end

function R = rz(cE, ce, sE, se)
        R = cell(12,1);
        R{1} = Ees(cE, ce);
        R{2} = Ees(sE, se);
        R{3} = 0;
        R{4} = Ees(-sE, -se);
        R{5} = Ees(cE, ce);
        R{6} = 0;
        R{7} = 0;
        R{8} = 0;
        R{9} = 1;
        R{10} = 0;
        R{11} = 0;
        R{12} = 0;
end