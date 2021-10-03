close all; clear; clc;

%% initialization
% read in polynomial zonotopes in offline reachability analysis
PZ_reachset = load('C:\Users\RoahmLab\Documents\arm_planning\simulator_files\arm_rotatotope_RTD\FRS_trig_PZ\trig_FRS_-2.875.mat');
g_k = max(pi/24, abs(PZ_reachset.my_c_IC/3));

% time information
timeids = 80:80;
dt = 0.01;

% forward kinematics of the robot
load('fetch_FK_info.mat');
num_joints = size(axes,2);

% obstacles (represented in zonotopes)
O{1} = zonotope([0.5;0.5;0.5;],diag([0.05,0.075,0.1]));

%% compose joint FRS
[joint_pos, composeJointFRS_timing] = composeFRS_PZmatrix_fetch(timeids, PZ_reachset, T, axes);

%% optimization formalization
factor = 2*g_k*rand(6,1)-g_k;

[c,ceq,gc,gceq] = linkFRSconstraints(factor, timeids, joint_pos, num_joints, O);

%% run optimization problem
options = optimoptions(@fmincon,'CheckGradients',true,'SpecifyObjectiveGradient',false,'SpecifyConstraintGradient',true);

sol = fmincon(@(x)norm(x),rand(6,1),[],[],[],[],-ones(6,1),ones(6,1),@(x)linkFRSconstraints(x, timeids, joint_pos, num_joints, O),options);

%% plot the result
% figure; view(3); grid on; hold on; axis equal;
% 
% factor = 2*g_k*rand(6,1)-g_k;
% 
% FRS_colors = [ones(length(PZ_reachset.Rcont),1) * [1,0,0], linspace(0.5,0.1,length(PZ_reachset.Rcont))'];  

% for timeid = timeids
%     for j = 1:(num_joints-1)
%         if j == 1
%             joint_pos_zono1 = interval(zeros(3,1));
%         else
%             joint_pos_zono1 = interval(sliceFRS(getDim(joint_pos{timeid}{j}, 10:12), factor));
%         end
%         
%         joint_pos_zono2 = interval(sliceFRS(getDim(joint_pos{timeid}{j+1}, 10:12), factor));
%         link_vertices = [vertices(joint_pos_zono1), vertices(joint_pos_zono2)]';
%         
%         link_conv_hull = convhulln(link_vertices);
%         trisurf(link_conv_hull,link_vertices(:,1),link_vertices(:,2),link_vertices(:,3),'FaceColor',[1,0,0],'FaceAlpha',0.1,'EdgeColor',[1,0,0],'EdgeAlpha',0.8);
%     end
% end

%% ARMTD
% tic;
% R = robot_arm_FRS_rotatotope_fetch(zeros(6,1), PZ_reachset.my_c_IC*ones(6,1));
% toc;
% 
% %% ARMTD plotting
% for timeid = timeids
%     plot_slice(R.link_FRS{1}{timeid}, factor(1:2));
%     plot_slice(R.link_FRS{2}{timeid}, factor(1:4));
%     plot_slice(R.link_FRS{3}{timeid}, factor(1:6));
% end

%% ground truth
% for timeid = timeids
%     timeInterval = interval((timeid-1)*dt, timeid*dt);
%     
%     for i = 1:50
%         samplet = randPoint(timeInterval);
%         
%         if timeid <= 50
%             traj = PZ_reachset.my_c_IC * samplet + 0.5 * factor * samplet.^2;
%         else
%             traj = -0.25 * PZ_reachset.my_c_IC - 0.25 * factor + (2 * PZ_reachset.my_c_IC + factor) * samplet - 0.5 * (2 * PZ_reachset.my_c_IC + factor) * samplet.^2;
%         end
% 
%         joint_reachable_set_groundtruth = RobotForwardKinematics(num_joints,[traj;rand],eye(4),T,axes);
% 
%         for j = 1:num_joints-1
%             plot3([joint_reachable_set_groundtruth{j}(1,4),joint_reachable_set_groundtruth{j+1}(1,4)],...
%                   [joint_reachable_set_groundtruth{j}(2,4),joint_reachable_set_groundtruth{j+1}(2,4)],...
%                   [joint_reachable_set_groundtruth{j}(3,4),joint_reachable_set_groundtruth{j+1}(3,4)],'k');
%         end
%     end
% end
% 
% title(['sliced reachable set, t = ', num2str(timeids(1)*dt), 's to ', num2str(timeids(end)*dt), 's'])



