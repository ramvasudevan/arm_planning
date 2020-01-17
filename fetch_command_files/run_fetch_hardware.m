%% description
% This script sets up a Fetch robot in an environment with various user
% specified scenarios:
% 1. Table
% 2. Wall/doorway
% 3. Posts
% 4. Shelves
% 5. Reach inside box
% 6. Sink to Cupboard?
% 7. Reach through window
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 14 Jan 2019
% Updated: ---
%
clear; clc;
%% user parameters
%%% CHOOSE SCENARIO %%%
scenario = 6; % 1 2 3 4 5 6 or 7
sub_scenario = 1; % sub_scenario only changes for scenario 4

%%% EXPORTING TO CSV %%%
write_to_csv = false;
run_simulation_flag = true ;
world_save_dir = 'arm_planning/simulator_files/testing/saved_worlds/20200116_scenarios';
if ~exist(world_save_dir, 'dir')
    mkdir(world_save_dir);
end
csv_filename = sprintf('%s/scene_%03d_%03d.csv', world_save_dir, scenario, sub_scenario);

%%% PLANNING PARAMETERS %%%
use_cuda_flag = true ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'
t_plan = 0.5 ;


% add more obstacles
create_random_obstacles_flag = false ; % in addition to the shelf
verbosity = 6 ;
allow_replan_errors = true ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = true ; 
HLP_timeout = 2 ; 
HLP_grow_tree_mode = 'seed' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.3 ;
use_end_effector_for_cost_flag = true ;
plot_CAD_flag = false ;

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0,'plot_CAD_flag',plot_CAD_flag,...
    'move_mode',agent_move_mode);
%% make world
[scenario_obstacles,start,goal_radius,goal_type,goal] = get_fetch_scenario_info(scenario) ;

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius',goal_radius,...
    'N_random_obstacles',0,'dimension',3,'workspace_goal_check',1,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'start',start,'goal',goal,...
    'create_random_obstacles_flag',create_random_obstacles_flag,...
    'goal_type',goal_type) ;

%% make shelf obstacle
W.add_obstacle(scenario_obstacles) ;

%% make planner
FRS_options = struct() ;
FRS_options.t_plan = t_plan;
FRS_options.origin_shift = A.joint_locations(1:3, 1);
FRS_options.T = T ;
FRS_options.L = 0.33 ;
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200) ;
FRS_options.maxcombs = 200;

P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options,...
    'verbose', verbosity, 't_plan', t_plan,...
    't_move',t_plan,...
    'time_discretization', time_discretization,...
    'first_iter_pause_flag',first_iter_pause_flag,...
    'plot_HLP_flag',plot_HLP_flag,...
    'lookahead_distance',lookahead_distance,...
    'use_end_effector_for_cost_flag',use_end_effector_for_cost_flag,...
    'use_cuda_flag', use_cuda_flag) ;

% P.HLP = robot_arm_RRT_HLP('sampling_timeout',HLP_timeout,...
%     'plot_while_sampling_flag',plot_while_sampling_flag,...
%     'plot_HLP_flag',plot_HLP_flag,...
%     'make_new_graph_every_iteration_flag',make_new_graph_every_iteration,...
%     'new_node_growth_distance',0.5,...
%     'choose_goal_as_new_node_frequency',0.5) ;

% P.HLP = robot_arm_optimization_HLP() ;

% P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
%     'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
%     'grow_tree_mode',HLP_grow_tree_mode,...
%     'buffer',0.1) ;

%% set up for running hardware
% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

%% ROS setup

%% run hardware
start_tic = tic ;
iter_cur = 1 ;
t_agent = 0 ;
t_cur = toc(start_tic) ;
d_to_goal = inf ;

while iter_cur < max_iterations && t_cur < max_time && d_to_goal > W.goal_radius
    % get the agent info
    
    % get the world info
    
    % call replan
    
    % process the traj if needed
    
    % send traj to robit
    
    % pause however long is left over
end