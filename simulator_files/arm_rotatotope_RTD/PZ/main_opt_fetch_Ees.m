close all; clear; clc;

%% initialization
% read in polynomial zonotopes in offline reachability analysis
PZ_reachset = load('..\FRS_trig_PZ_improved\trig_FRS_-2.875.mat');
g_k = max(pi/24, abs(PZ_reachset.my_c_IC/3));

% time information
% timeids = 10:10:length(PZ_reachset.Rcont);
timeids = 128;
dt = 0.005;

% forward kinematics of the robot
load('fetch_FK_info.mat');
num_joints = size(axes,2);

% goal
goal = [1;1;1];

% obstacles (represented in zonotopes)
O{1} = zonotope([0.5;0.5;0.5],diag([0.05,0.075,0.1]));
O{2} = zonotope([0;0;0],diag([0.05,0.075,0.1]));

%% construct reachable set error bound
[joint_pos_error, duration_t] = composeFRS_Ees_fetch(timeids, PZ_reachset.Rcont, T, axes);

%% run optimization problem
x0 = rand(6,1);

options = optimoptions(@fmincon,'CheckGradients',true,'SpecifyObjectiveGradient',false,'SpecifyConstraintGradient',true);

sol = fmincon(@(x)norm(x), ...
              rand(6,1), ...
              [],[],[],[], ...
              -ones(6,1), ...
              ones(6,1), ...
              @(x)linkFRSconstraints_Ees(x, PZ_reachset.Rcont, joint_pos_error, timeids, num_joints, T, axes, O), ...
              options);
error('eee');
%% plot the result
figure; view(3); grid on; hold on; axis equal;

for i = 1:length(O)
    obs_vertices = vertices(O{i});
    Digit.animation.PlotZonotope(O{i}, [1,0,0]);
    shp = alphaShape(obs_vertices(1,:)',obs_vertices(2,:)',obs_vertices(3,:)',inf);
    plot(shp,'FaceColor',[1,0,0],'FaceAlpha',0.1,'EdgeColor',[1,0,0],'EdgeAlpha',0.8);
end

for timeid = timeids
    c_inp = zeros(length(sol),1);
    s_inp = zeros(length(sol),1);
    dc_inp = zeros(length(sol),length(sol));
    ds_inp = zeros(length(sol),length(sol));
    for j = 1:length(sol)
        PZ_reachset.Rcont{timeid}.id = j;
        [temp_cs, temp_dcs] = sliceFRS(getDim(PZ_reachset.Rcont{timeid}, 1:2), sol);
        temp_cs = center(temp_cs);
        c_inp(j) = temp_cs(1);
        s_inp(j) = temp_cs(2);
        dc_inp(j,:) = temp_dcs(1,:);
        ds_inp(j,:) = temp_dcs(2,:);
    end

    [joint_reachable_set, ~] = RobotForwardKinematics_diff(num_joints,c_inp,s_inp,dc_inp,ds_inp,eye(4),T,axes);

    for j = 1:(num_joints-1)
        if j == 1
            joint_pos1 = zeros(3,1);
            joint_rad = zeros(3,1);
        else
            joint_pos1 = joint_reachable_set{j}(1:3,4);
            joint_rad = [joint_pos_error{timeid}{j}{10}.e.rad; joint_pos_error{timeid}{j}{11}.e.rad; joint_pos_error{timeid}{j}{12}.e.rad];
        end
        
        joint_pos_zono1 = joint_pos1 + interval(-joint_rad, joint_rad);

        joint_pos2 = joint_reachable_set{j+1}(1:3,4);
        
        joint_rad = [joint_pos_error{timeid}{j+1}{10}.e.rad; joint_pos_error{timeid}{j+1}{11}.e.rad; joint_pos_error{timeid}{j+1}{12}.e.rad];
        joint_pos_zono2 =  joint_pos2 + interval(-joint_rad, joint_rad);
        link_vertices = [vertices(joint_pos_zono1), vertices(joint_pos_zono2)]';
        
        link_conv_hull = convhulln(link_vertices);
        trisurf(link_conv_hull,link_vertices(:,1),link_vertices(:,2),link_vertices(:,3),'FaceColor',[0,0,1],'FaceAlpha',0.2,'EdgeColor',[0,0,1],'EdgeAlpha',0.8);
    end
end