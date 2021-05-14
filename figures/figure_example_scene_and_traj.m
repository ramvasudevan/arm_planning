% patrick holmes
% 2020 01 31 plotting three trajectories for the ARMTD paper
% ARTMD + SLP on shelf example (scenario 4)
% ARMTD + RRT*
% CHOMP crashing.

% do the ARMTD + SLP first...
clear;

filename = 'arm_planning/simulator_files/testing/trial_data/20200127/trial_scene_008_009.mat';
% filename = 'arm_planning/simulator_files/testing/trial_data/20200130_SLP/trial_scene_004_001.mat';
% filename = 'arm_planning/simulator_files/testing/trial_data/20200130_RRTstar/trial_scene_007_001.mat';

figure(1) ; clf ; axis equal ; hold on ; grid off; box on;
set(gca,'Projection','Perspective')
set(gca, 'XLim', [-1 1], 'YLim', [-1, 1], 'ZLim', [-0.05 1.5]);
set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% campos([   16.4817   -7.5082   20.9115]);
campos([    13.5308   -7.0115    5.9994]);
% campos([10.1429   -6.3185   11.5536]);



blues = 1/256*[222,235,247
158,202,225
49,130,189];

lightpurple = 1/256*[117,107,177];
purple = 1/256*[84,39,143];

verbosity = 0 ;
dimension = 3;
goal_type = 'configuration';
goal_radius = pi/30;

load(filename)

agent_info = summary(1).agent_info ;
bounds = summary(1).bounds ;
obstacles = summary(1).obstacles ;
planner_info = summary(1).planner_info ;
start = summary(1).start ;
goal = summary(1).goal ;

plot_CAD_flag = false;
A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'plot_CAD_flag', plot_CAD_flag);
A_CHOMP = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'plot_CAD_flag', plot_CAD_flag);

A_CHOMP = reset_agent_with_chomp_traj(A_CHOMP, 'chomp_trajectory_scene_008_009.txt');


A.link_plot_face_opacity = 0.5;
A.link_plot_edge_opacity = 0.2;
% A.link_plot_edge_color = blues(3, :);
A.link_plot_face_color = blues(3, :);

W = fetch_base_world_static('create_random_obstacles_flag', false, 'include_base_obstacle', false, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type) ;

for i = 1:length(W.obstacles)
    if W.obstacles{i}.is_base_obstacle
        W.obstacles{i}.plot_face_opacity = 0.2;
%         W.obstacles{i}.plot_edge_opacity = 1;
%         W.obstacles{i}.plot_face_opacity = 0.2;
        if ~(W.obstacles{i}.plot_face_color(1) == 0.9)
            W.obstacles{i}.plot_face_color = [0.7 0.7 0.7];
        else
%             W.obstacles{i}.plot_face_color = [0 0 0];
        end
    else
        W.obstacles{i}.plot_face_opacity = 0.75;
%         W.obstacles{i}.plot_edge_opacity = 1;
        W.obstacles{i}.plot_face_color = [1 0.7 0.7];
    end
end

% fill in agent state
A.time = agent_info.time ;
A.state = agent_info.state ;

% set up world using arm
W.setup(agent_info)
% clear_plot_data(W);

plot(W)
nTimes = 50;
% if dimension == 3
%     view(3)
% end
% animate(A)

chomploton = 1;

if chomploton
    poop = A_CHOMP;
    bleh = find(all(A_CHOMP.state(A_CHOMP.joint_state_indices, :) == W.start), 1, 'last');
    start_time = A_CHOMP.time(bleh);
    mytimes = linspace(start_time, A_CHOMP.time(end), nTimes);
    filename = 'figure_example_scene_chomp';
else
    poop = A;
    mytimes = linspace(A.time(1), A.time(end), nTimes);
    filename = 'figure_example_scene_armtd';
end

AI = poop.get_agent_info;
for i = 1:length(mytimes)
    [~, idx] = min(abs(poop.time - mytimes(i)));
    if i == 1
        poop.link_plot_face_opacity = 1;
        poop.link_plot_edge_opacity = 0.2;
        poop.link_plot_edge_color = lightpurple;
        poop.link_plot_face_color = purple;
    else
        poop.link_plot_face_opacity = 0.25;
        poop.link_plot_edge_opacity = 0.1;
        poop.link_plot_edge_color = [0.2 0.2 0.2];
        poop.link_plot_face_color = [0.9 0.9 0.9];
%         poop.link_plot_edge_color = blues(3, :);
        poop.link_plot_face_color = blues(1, :);
    end
    collision_check = W.collision_check_single_state(AI, poop.state(poop.joint_state_indices, idx));
    if collision_check
        poop.link_plot_edge_color = [0.2 0.2 0.2];
        poop.link_plot_face_color = 'r';
    end
    
    poop.plot_at_time(mytimes(i));
end

print(filename, '-dpng', '-r300');
print(filename, '-depsc', '-r300');
fig = gcf;
save_figure_to_pdf(fig, filename);
