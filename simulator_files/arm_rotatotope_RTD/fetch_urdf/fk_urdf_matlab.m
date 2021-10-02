close all; clear; clc;

%% load robot
% add one more end effector link to compute the position of the end
% effector
robot_name = 'fetch';
robot_filename = 'fetch_arm_reduced.urdf';

% load robot
robot = importrobot(robot_filename);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];

% robot parameters
use_random = false;
num_joints = robot.NumBodies;

%% frame transformation
T = zeros(4,4,num_joints);
axes = zeros(3, num_joints);

for i = 1:num_joints
    if i == 1
        T(:,:,i) = getTransform(robot, zeros(num_joints,1), robot.Bodies{i}.Name, robot.Base.Name);
    else
        T(:,:,i) = getTransform(robot, zeros(num_joints,1), robot.Bodies{i}.Name, robot.Bodies{i-1}.Name);
    end
    axes(:,i) = robot.Bodies{i}.Joint.JointAxis';
end

save([robot_name, '_FK_info.mat'], 'T', 'axes');
% load([robot_name, '_FK_info_right_pinned.mat'])

%% validation over a random configuration
q = randomConfiguration(robot);

FK_true = getTransform(robot, q, robot.Bodies{num_joints}.Name, robot.Base.Name);

FK = cell(num_joints,1);

for i = 1:num_joints
    dim = find(axes(:,i) ~=0);
    
    if dim == 1
        R = rx(axes(dim,i) * q(i));
    elseif dim == 2
        R = ry(axes(dim,i) * q(i));
    else
        R = rz(axes(dim,i) * q(i));
    end
    
    if i == 1
        FK{i} = T(:,:,i) * R;
    else
        FK{i} = FK{i-1} * T(:,:,i) * R;
    end
end

disp(FK{end} - FK_true)

%% helper functions
function R = rx(th)

    c = cos(th);
    s = sin(th);

    R = [1  0  0  0;
         0  c -s  0;
         0  s  c  0; 
         0  0  0  1];
  
end

function R = ry(th)

    c = cos(th);
    s = sin(th);

    R = [c  0  s  0;
         0  1  0  0;
        -s  0  c  0; 
         0  0  0  1];
         
end

function R = rz(th)

    c = cos(th);
    s = sin(th);

    R = [c -s 0  0;
         s  c 0  0; 
         0  0 1  0;
         0  0 0  1];

end