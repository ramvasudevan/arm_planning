close all; clear; clc;

%% initialization
dt = 0.01;
g_t = dt / 2;

timeids = 80:80;

load('fetch_FK_info.mat');

num_joints = size(axes,2);

%% read in polynomial zonotopes in offline reachability analysis
PZ_reachset = load('C:\Users\RoahmLab\Documents\arm_planning\simulator_files\arm_rotatotope_RTD\FRS_trig_PZ\trig_FRS_-2.875.mat');

g_k = max(pi/24, abs(PZ_reachset.my_c_IC/3));

%% compose FRS through kinematics chain
joint_reachable_set = cell(length(PZ_reachset.Rcont),1);
tic;
% for i = 1:length(PZ_reachset.Rcont)
for i = timeids
    disp(i)
    joint_reachable_set{i} = RobotForwardKinematics_PZmatrix_fetch(num_joints,PZ_reachset.Rcont{i},T,axes);
end
toc;

%% simplify FRS
joint_pos = cell(length(PZ_reachset.Rcont),1);
% for i = 1:length(PZ_reachset.Rcont)
for i = timeids
    joint_pos{i} = cell(num_joints,1);
    for j = 2:num_joints
        joint_pos{i}{j} = joint_reachable_set{i}{j}{1}.toPolyZonotope;
        
        for k = 2:12
            if isa(joint_reachable_set{i}{j}{k}, 'PZmatrix')
                joint_pos{i}{j} = exactCartProd(joint_pos{i}{j}, joint_reachable_set{i}{j}{k}.toPolyZonotope);
            else
                joint_pos{i}{j} = exactCartProd(joint_pos{i}{j}, joint_reachable_set{i}{j}{k});
            end
        end
        
%         joint_pos{i}{j} = reduceFactorsFull(joint_pos{i}{j}, 1);
    end
end

%% plot the result
figure; view(3); grid on; hold on; axis equal;

factor = 2*g_k*rand(6,1)-g_k;

FRS_colors = [ones(length(PZ_reachset.Rcont),1) * [1,0,0], linspace(0.5,0.1,length(PZ_reachset.Rcont))'];  

% for timeid = 1:length(PZ_reachset.Rcont)
for timeid = timeids
    joint_pos_zono = cell(num_joints,1);
    for j = 1:(num_joints-1)
        if j == 1
            joint_pos_zono1 = interval(zeros(3,1));
        else
            joint_pos_zono1 = interval(sliceFRS(getDim(joint_pos{timeid}{j}, 10:12), factor));
        end
        
        joint_pos_zono2 = interval(sliceFRS(getDim(joint_pos{timeid}{j+1}, 10:12), factor));
        link_vertices = [vertices(joint_pos_zono1), vertices(joint_pos_zono2)]';
        
        link_conv_hull = convhulln(link_vertices);
        trisurf(link_conv_hull,link_vertices(:,1),link_vertices(:,2),link_vertices(:,3),'FaceColor',[1,0,0],'FaceAlpha',0.1,'EdgeColor',[1,0,0],'EdgeAlpha',0.8);
    end
end

%% ARMTD
tic;
R = robot_arm_FRS_rotatotope_fetch(zeros(6,1), PZ_reachset.my_c_IC*ones(6,1));
toc;

%% ARMTD plotting
for timeid = timeids
    plot_slice(R.link_FRS{1}{timeid}, factor(1:2));
    plot_slice(R.link_FRS{2}{timeid}, factor(1:4));
    plot_slice(R.link_FRS{3}{timeid}, factor(1:6));
end

%% ground truth
% A = robot_arm_3D_fetch('verbose',0, 'animation_set_axes_flag',0,...
%                         'animation_set_view_flag',0,'use_CAD_flag',0,...
%                         'move_mode','direct');

% for timeid = 1:length(PZ_reachset.Rcont)
for timeid = timeids
    timeInterval = interval((timeid-1)*dt, timeid*dt);
    
    for i = 1:50
        samplet = randPoint(timeInterval);
        
        if timeid <= 50
            traj = PZ_reachset.my_c_IC * samplet + 0.5 * factor * samplet.^2;
        else
            traj = -0.25 * PZ_reachset.my_c_IC - 0.25 * factor + (2 * PZ_reachset.my_c_IC + factor) * samplet - 0.5 * (2 * PZ_reachset.my_c_IC + factor) * samplet.^2;
        end

        joint_reachable_set_groundtruth = RobotForwardKinematics(num_joints,[traj;rand],eye(4),T,axes);

        for j = 1:num_joints-1
            plot3([joint_reachable_set_groundtruth{j}(1,4),joint_reachable_set_groundtruth{j+1}(1,4)],...
                  [joint_reachable_set_groundtruth{j}(2,4),joint_reachable_set_groundtruth{j+1}(2,4)],...
                  [joint_reachable_set_groundtruth{j}(3,4),joint_reachable_set_groundtruth{j+1}(3,4)],'k');
        end

%         [R,T,J] = A.forward_kinematics(traj);
%         
%         J = J - J(:,1);
%         
%         for j = 1:size(J,2)-1
%             plot3([J(1,j),J(1,j+1)], [J(2,j),J(2,j+1)], [J(3,j),J(3,j+1)], 'g');
%         end
    end
end

title(['sliced reachable set, t = ', num2str(timeids(1)*dt), 's to ', num2str(timeids(end)*dt), 's'])



