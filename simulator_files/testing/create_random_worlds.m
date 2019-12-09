%% description
% This script generates random worlds to be used for evaluating aRmTD
% against other motion planners
%
% Authors: Patrick Holmes
% Created: 11 November 2019

clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
world_save_dir = 'arm_planning/simulator_files/testing/saved_worlds/20191205';
if ~exist(world_save_dir, 'dir')
    mkdir(world_save_dir);
end

N_obstacle_min = 1 ;
N_obstacle_max = 10 ;
N_worlds_per_obstacle = 10;
dimension = 3 ;
nLinks = 3 ;
verbosity = 6 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
floor_normal_axis = 1;

A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);

%% automated from here

for i = N_obstacle_min:N_obstacle_max
    for j = 1:N_worlds_per_obstacle
    
        % use this to start from random start config:
        W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03, 'N_obstacles',i,'dimension',dimension,'workspace_goal_check', 0,...
            'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025) ;
        % W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03, 'N_obstacles',N_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
        %     'verbose',verbosity,'start', [0;0;0;0;0;0], 'goal', [pi;0;0;0;0;0], 'creation_buffer', 0.05) ;


        % set up world using arm
        I = A.get_agent_info ;
        W.setup(I)

        % place arm at starting configuration
        A.state(A.joint_state_indices) = W.start ;
        
        filename = sprintf('%s/scene_%03d_%03d.csv', world_save_dir, i, j);

        % create .csv file
        write_fetch_scene_to_csv(W, filename);

    end
end