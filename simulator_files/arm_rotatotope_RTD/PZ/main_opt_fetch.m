close all; clear; clc;

%% initialization
% read in polynomial zonotopes in offline reachability analysis
PZ_reachset = load('..\FRS_trig_PZ\trig_FRS_-2.875.mat');
g_k = max(pi/24, abs(PZ_reachset.my_c_IC/3));

% time information
% timeids = 10:10:length(PZ_reachset.Rcont);
timeids = 200;
dt = 0.005;

% forward kinematics of the robot
load('fetch_FK_info.mat');
num_joints = size(axes,2);

% goal
goal = [1;1;1];

% obstacles (represented in zonotopes)
O{1} = zonotope([0.5;0.5;0.5],diag([0.05,0.075,0.1]));
O{2} = zonotope([0;0;0],diag([0.05,0.075,0.1]));

%% compose joint FRS
[joint_pos, composeJointFRS_timing] = composeFRS_PZmatrix_fetch(timeids, PZ_reachset.Rcont, T, axes);

%% run optimization problem
x0 = rand(6,1);

options = optimoptions(@fmincon,'CheckGradients',true,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);

sol = fmincon(@(x)jointFRScost(x, 100, joint_pos, goal), ...
              rand(6,1), ...
              [],[],[],[], ...
              -ones(6,1), ...
              ones(6,1), ...
              @(x)linkFRSconstraints(x, timeids, joint_pos, num_joints, O), ...
              options);

%% plot the result
% figure; view(3); grid on; hold on; axis equal;

for i = 1:length(O)
    obs_vertices = vertices(O{i});
    Digit.animation.PlotZonotope(O{i}, [1,0,0]);
    shp = alphaShape(obs_vertices(1,:)',obs_vertices(2,:)',obs_vertices(3,:)',inf);
    plot(shp,'FaceColor',[1,0,0],'FaceAlpha',0.1,'EdgeColor',[1,0,0],'EdgeAlpha',0.8);
end

for timeid = timeids
    for j = 1:(num_joints-1)
        if j == 1
            joint_pos_zono1 = interval(zeros(3,1));
        else
            joint_pos_zono1 = interval(sliceFRS(getDim(joint_pos{timeid}{j}, 10:12), sol));
        end
        
        joint_pos_zono2 = interval(sliceFRS(getDim(joint_pos{timeid}{j+1}, 10:12), sol));
        link_vertices = [vertices(joint_pos_zono1), vertices(joint_pos_zono2)]';
        
        link_conv_hull = convhulln(link_vertices);
        trisurf(link_conv_hull,link_vertices(:,1),link_vertices(:,2),link_vertices(:,3),'FaceColor',[0,0,0.6],'FaceAlpha',0.1,'EdgeColor',[0,0,1],'EdgeAlpha',0.8);
    end
end

% %% ARMTD
% tic;
% R = robot_arm_FRS_rotatotope_fetch(zeros(6,1), PZ_reachset.my_c_IC*ones(6,1));
% toc;
% 
% %% ARMTD plotting
% for timeid = timeids
%     plot_slice(R.link_FRS{1}{timeid}, g_k * sol(1:2));
%     plot_slice(R.link_FRS{2}{timeid}, g_k * sol(1:4));
%     plot_slice(R.link_FRS{3}{timeid}, g_k * sol(1:6));
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



