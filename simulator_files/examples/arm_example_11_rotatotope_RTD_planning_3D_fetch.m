%% description
% This script generates an agent, world, and planner. It plugs them in to
% the simulator framework, and then runs a single simulation.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 29 July 2019
% Updated: 3 August 2019
%
% Updated: 19 August 2019 - moving RTD to 3D and using a 1 link model
% Updated: 22 August 2019 - moving RTD to 3D and using Fetch model
% Updated: 18 September 2019 - using rotatotope RTD planner now
clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
N_random_obstacles = 20;
dimension = 3 ;
nLinks = 3 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
use_cuda_flag = false;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'

first_iter_pause_flag = true;
create_random_obstacles_flag = false ;
verbosity = 6 ;
actual_t_plan = 10 ;
simulated_t_plan = 0.5 ;
HLP_timeout = 0.2 ; 
HLP_grow_tree_mode = 'new' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.2 ;
use_end_effector_for_cost_flag = true ;
plot_CAD_flag = false ; % plot the faaaaancy arm :)

A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'move_mode', agent_move_mode);

%% automated from here

% can adjust LLC gains here
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', pi/30, 'N_random_obstacles',N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.05, 'base_creation_buffer', 0.05) ;
% W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03, 'N_random_obstacles',N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
%     'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025, 'start', [0 0 0 0 0 0]') ;

FRS_options = struct();
% FRS_options.position_dimensions = [1;2;3];
% FRS_options.extra_position_dimensions = [4;5];
% FRS_options.IC_dimensions = [6;7];
% FRS_options.param_dimensions = [8;9];
% FRS_options.nLinks = nLinks;
% FRS_options.time_discretization = 0.01;

FRS_options.t_plan = t_plan;
FRS_options.origin_shift = A.joint_locations(1:3, 1);
FRS_options.T = T;
FRS_options.L = 0.33;
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;
P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options,...
    'verbose', verbosity, 't_plan', actual_t_plan,...
    't_move',simulated_t_plan,...
    'time_discretization', time_discretization,...
    'first_iter_pause_flag',first_iter_pause_flag,...
    'plot_HLP_flag',plot_HLP_flag,...
    'lookahead_distance',lookahead_distance,...
    'use_end_effector_for_cost_flag',use_end_effector_for_cost_flag,...
    'use_cuda_flag', use_cuda_flag) ;
    
% P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
%     'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
%     'grow_tree_mode',HLP_grow_tree_mode,...
%     'buffer',0.1) ;

% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
% W.start = zeros(6, 1); % put in "home" config
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',1000,'max_sim_iterations',1000) ;

% create .csv file
% write_fetch_scene_to_csv(W);


%% run simulation
S.run()

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(W)

if dimension == 3
    view(3)
end

animate(A)
