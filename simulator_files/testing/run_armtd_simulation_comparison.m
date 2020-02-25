%% description
% This script iterates through a list of presaved random worlds and runs
% the aRmTD planner on them. It then saves information on how well each the
% planner performed in each trial.
%
% Authors: Patrick Holmes (adapted from Shreyas Kousik code)
% Created 25 November 2019
% Edited 16 January 2020

clear; clc;

%% user parameters
% goal_type = 'end_effector_location';
goal_type = 'configuration';
goal_radius = pi/30;
dimension = 3 ;
nLinks = 3 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
use_cuda_flag = true;
agent_move_mode = 'direct';

create_random_obstacles_flag = false ;
verbosity = 6 ;
actual_t_plan = 10 ;
simulated_t_plan = 0.5 ;
HLP_timeout = 2 ; 
HLP_grow_tree_mode = 'new' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.2 ;
use_end_effector_for_cost_flag = true ;
plot_CAD_flag = false ; % plot the faaaaancy arm :)

% plotting
plot_while_running = false ;
% agent_camera_distance = 3 ; % default is 3
% agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
% plot_agent_view = 'behind' ; % none, behind, above, or onboard
% plot_zonotopes = true ;

% simulation
% start_idx = 5 ;
% end_idx = 500 ;
verbosity = 10 ;
max_sim_time = 600 ;
max_sim_iter = 1000 ;
first_iter_pause_flag = false;

% file handling
save_file_header = 'trial_' ;
file_location = 'arm_planning/simulator_files/testing/trial_data/20200127' ;
if ~exist(file_location, 'dir')
    mkdir(file_location);
end

% world file
world_file_header = 'scene';
world_file_folder = 'arm_planning/simulator_files/testing/saved_worlds/20200127/';
world_file_location = sprintf('%s*%s*', world_file_folder, world_file_header);
world_file_list = dir(world_file_location);


%% automated from here
% run loop
tic
% for idx = 68
for idx = 1:length(world_file_list)
    % read world CSV to get start and goal, populate obstacles:
    world_filename = world_file_list(idx).name;
    [start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);
    
%     figure(1); clf; view(3); grid on;
    
    W = fetch_base_world_static('create_random_obstacles_flag', false, 'include_base_obstacle', true, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type) ;
    
    % create arm agent
    A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'move_mode', agent_move_mode);
    
    % can adjust LLC gains here
    A.LLC.K_p = 1*A.LLC.K_p;
    A.LLC.K_i = 1*A.LLC.K_i;
    A.LLC.K_d = 1*A.LLC.K_d;
    A.joint_input_limits = 1*A.joint_input_limits;
    
    % options for FRS
    FRS_options = struct();
    FRS_options.t_plan = t_plan;
    FRS_options.origin_shift = A.joint_locations(1:3, 1);
    FRS_options.T = T;
    FRS_options.buffer_dist = A.buffer_dist;
    FRS_options.combs = generate_combinations_upto(200);
    FRS_options.maxcombs = 200;
    
    % create planner
    P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options,...
        'verbose', verbosity, 't_plan', actual_t_plan,...
        't_move',simulated_t_plan,...
        'time_discretization', time_discretization,...
        'first_iter_pause_flag',first_iter_pause_flag,...
        'plot_HLP_flag',plot_HLP_flag,...
        'lookahead_distance',lookahead_distance,...
        'use_end_effector_for_cost_flag',use_end_effector_for_cost_flag,...
        'use_cuda_flag', use_cuda_flag) ;
    
%     P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
%         'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
%         'grow_tree_mode',HLP_grow_tree_mode,...
%         'buffer',0.1) ;
    
    % set up world using arm
    I = A.get_agent_info ;
    W.setup(I)
    
    % place arm at starting configuration
    % W.start = zeros(6, 1); % put in "home" config
    A.state(A.joint_state_indices) = W.start ;
    
    % create simulator
    S = simulator(A,W,P, 'plot_while_running', plot_while_running, 'allow_replan_errors',allow_replan_errors,'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter) ; 
    
    % %% plotting
    if plot_while_running
        figure(1) ; clf ; axis equal ; hold on ; grid on
        
        plot(A)
        plot(W)
        
        if dimension == 3
            view(3)
        end
    end
    
%     animate(A)
    
    % run simulation
    summary = S.run() ;
    
    % save summary
%     filename = [file_location,save_file_header,num2str(idx,'%04.f'),'.mat'] ;
    filename = [file_location,'/',save_file_header,world_filename(1:end-4),'.mat'] ;
    save(filename, 'world_filename', 'summary')
    
    toc
end

