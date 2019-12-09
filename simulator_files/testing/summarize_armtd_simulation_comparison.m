%% description
% This script iterates through the saved armtd trials and summarizes
% information about each (ie crash check, goal check, planning time, etc.)
%
% Author: Patrick Holmes
% Created 06 December 2019

clear; clc;

trial_file_header = 'trial_scene_' ;
trial_file_folder = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/20191205/' ;
trial_file_location = sprintf('%s*%s*', trial_file_folder, trial_file_header);
trial_file_list = dir(trial_file_location);

results = struct();
results.scene = {};
results.crash = [];
results.goal = [];
results.avg_planning_time = [];

for idx = 1:length(trial_file_list)
    trial_filename = trial_file_list(idx).name;
    mytrial = load([trial_file_folder trial_filename]);
    summary = mytrial.summary;
    
    results.scene{end+1, 1} = mytrial.world_filename;
    results.crash(end+1, 1) = summary.crash_check;
    results.goal(end+1, 1) = summary.goal_check;
    results.avg_planning_time(end+1, 1) = mean(summary.planning_time(~isnan(summary.planning_time)));
    
end

disp(struct2table(results));

disp('Crashed?');
disp([num2str(sum(results.crash)) ' / 100' ]);
disp('Reached Goal?');
disp([num2str(sum(results.goal)) ' / 100' ]);




% % filename = 'trial_20190410_0262.mat' ;
% clear; clc; 
% filename = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/20191205/trial_scene_007_005.mat';
% 
% % obs_color = [1 0.7 0.7] ;
% % obs_opacity = 0.5 ;
% % 
% % agent_camera_distance = 3 ; % default is 3
% % agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
% % plot_agent_view = 'behind' ; % none, behind, above, or onboard
% 
% verbosity = 10 ;
% dimension = 3;
% 
% 
% %% automated from here
% load(filename)
% 
% agent_info = summary(1).agent_info ;
% bounds = summary(1).bounds ;
% obstacles = summary(1).obstacles ;
% planner_info = summary(1).planner_info ;
% start = summary(1).start ;
% goal = summary(1).goal ;
% 
% % create agent
% % A = quadrotor_agent('verbose',verbose_level,...
% %     'plot_view_style',plot_agent_view,...
% %     'camera_direction_offset',agent_camera_position,...
% %     'camera_follow_dist',agent_camera_distance) ;
% 
% A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);
% 
% % create world
% % W = zonotope_box_world('verbose',verbose_level,...
% %     'obs_color',obs_color,...
% %     'obs_opacity',obs_opacity,...
% %     'bounds',bounds,...
% %     'obstacles',obstacles,...
% %     'goal',goal) ;
% 
% W = fetch_base_world_static('create_random_obstacles', 0, 'include_base_obstacle', 0, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
%     'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles) ;
% 
% % fill in agent state
% A.time = agent_info.time ;
% A.state = agent_info.state ;
% % A.attitude = agent_info.attitude ;
% 
% % set up world using arm
% W.setup(agent_info)
% clear_plot_data(W);
%     
% 
% %% plot
% % figure(1)
% % clf
% % hold on
% % plot(W)
% % axis equal
% % plot_at_time(A)
% % camlight
% % lighting flat
% % material dull
% % lightangle(10,70)
% %% plotting
% figure(1) ; clf ; axis equal ; hold on ; grid on
% 
% plot(W)
% 
% if dimension == 3
%     view(3)
% end
% animate(A)