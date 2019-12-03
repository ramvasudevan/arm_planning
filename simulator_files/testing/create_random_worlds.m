%% description
% This script generates random worlds to be used for evaluating aRmTD
% against other motion planners
%
% Authors: Patrick Holmes
% Created: 11 November 2019

clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
world_save_dir = 'arm_planning/simulator_files/testing/saved_worlds/';
if ~exist(world_save_dir, 'dir')
    mkdir(world_save_dir);
end

N_obstacle_min = 1 ;
N_obstacle_max = 10 ;
N_worlds_per_obstacle = 1;
dimension = 3 ;
nLinks = 3 ;
verbosity = 6 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
floor_normal_axis = 1;

A = robot_arm_3D_fetch('verbose', verbosity, 'floor_normal_axis', floor_normal_axis, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);

%% automated from here

for i = N_obstacle_min:N_obstacle_max
    for j = 1:N_worlds_per_obstacle
    
        % use this to start from random start config:
        % W = arm_world_static('floor_normal_axis', floor_normal_axis, 'include_base_obstacle', 0, 'goal_radius', 0.03, 'N_obstacles',i,'dimension',dimension,'workspace_goal_check', 0,...
        %     'verbose',verbosity) ;
        W = arm_world_static('floor_normal_axis', floor_normal_axis, 'include_base_obstacle', 0, 'goal_radius', 0.03, 'N_obstacles',i,'dimension',dimension,'workspace_goal_check', 0,...
            'verbose',verbosity, 'start', [0;0;0;0;0;0]) ;

        % set up world using arm
        I = A.get_agent_info ;
        W.setup(I)

        % place arm at starting configuration
        A.state(A.joint_state_indices) = W.start ;
        
        filename = sprintf('%s20191125_obstacles_%s_world_%s.csv', world_save_dir, num2str(i), num2str(j));

        % create .csv file
        write_fetch_scene_to_csv(W, filename);

    end
end