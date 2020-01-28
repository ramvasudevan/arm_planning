%% description
% This script sets up a Fetch robot in an environment with a cabinet like
% Cabinet 3 in ROAHM Lab
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 23 Jan 2019
% Updated: not yet
%
%% user parameters
%%% WORLD PARAMETERS %%%
% manually create start
% start = [0; 0 ; 0 ; pi/2 ; 0 ; -pi/2.2 ] ; % on shelf 5
start = [0; -pi/4; pi ; pi/7 ; 0 ; -pi/2.7 ] ; % on shelf 2

% start = [pi/3; 0;0;0;0;0] ; % heckin outside the shelves

% manually create goal
goal_radius = pi/30 ;
% goal_type = 'configuration' ; % 'configuration' or 'end_effector_location'
goal_type = 'configuration' ;

% configuration goals (set goal_type to 'configuration')
% goal = [0; -pi/4; pi ; pi/7 ; 0 ; -pi/2.7 ] ; % on shelf 2
% goal = [-pi/5; -pi/3.7; pi/5 ; pi/3.7 ; 0 ; 0.1 ] ; % on shelf 3
goal = [0; -pi/4.2; 0 ; pi/1.7 ; 0 ; -pi/3 ] ; % on shelf 4
% goal = [0; 0 ; 0 ; pi/2 ; 0 ; -pi/2.2 ] ; % on shelf 5

% shelf parameters
shelf_center_xy = [0.8;0] ; % this is the default

% add more obstacles
N_random_obstacles = 6 ; % NOTE we should no longer set W.N_obstacles
create_random_obstacles_flag = false ; % in addition to the shelf
%%% END WORLD PARAMETERS %%%

%%% MANUAL WAYPOINTS IF YA WANT 'EM %%%
% manual_waypoints = [start(:), [0; 0 ; 0 ; pi/2 ; -pi/3 ; -pi/2.2],...
%     [0; 0 ; 3*pi/4 ; pi/2 ; -pi/2 ; -pi/2.2], [0; -pi/4 ; pi ; pi/4 ; -pi/2 ; -pi/2], ...
%     [0; -pi/4 ; pi ; pi/4 ; -pi/3 ; -pi/2], goal(:)] ; % this for shelf 5 to shelf 2
manual_waypoints = [start(:), [0; -pi/4 ; pi ; pi/4 ; -pi/3 ; -pi/2],...
    [0; -pi/4 ; pi/2 ; pi/2 ; -pi/2 ; -pi/2], ...
    [0; -pi/4 ;0 ; pi/1.5 ; -pi/2 ; -pi/2],...
    [0; -pi/4 ;0 ; pi/1.5 ; -pi/3 ; -pi/2.2], ...
    goal(:)] ;  % this for shelf 2 to shelf 4
%     [0; -pi/4 ; pi ; pi/4 ; -pi/2 ; -pi/2], ...


%%% OTHER PARAMETERS %%%
use_cuda_flag = true ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'
verbosity = 6 ;
allow_replan_errors = true ;
actual_t_plan = 10 ;
simulated_t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = false ; 
run_simulation_flag = true ;
HLP_timeout = 2 ; 
HLP_grow_tree_mode = 'new' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.3 ;
use_end_effector_for_cost_flag = false ;
csv_filename = 'fetch_shelf_scene_example.csv' ;
plot_CAD_flag = false ; % plot the faaaaancy arm :)
%%% END OTHER PARAMETERS %%%

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0,'plot_CAD_flag',plot_CAD_flag,...
    'move_mode',agent_move_mode);

% can adjust tracking controller gains here
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;

%% make world
W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius',goal_radius,...
    'N_random_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check',1,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'start',start,'goal',goal,...
    'create_random_obstacles_flag',create_random_obstacles_flag,...
    'goal_type',goal_type) ;

%% make shelf obstacle
C = make_roahmlab_cabinet_obstacle(shelf_center_xy) ;

W.add_obstacle(C) ;

%% make planner
FRS_options = struct() ;
FRS_options.t_plan = actual_t_plan;
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

% P.HLP = robot_arm_RRT_HLP('sampling_timeout',HLP_timeout,...
%     'plot_while_sampling_flag',plot_while_sampling_flag,...
%     'plot_HLP_flag',plot_HLP_flag,...
%     'make_new_graph_every_iteration_flag',make_new_graph_every_iteration,...
%     'new_node_growth_distance',0.5,...
%     'choose_goal_as_new_node_frequency',0.5) ;

% P.HLP = robot_arm_optimization_HLP() ;

P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
    'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
    'grow_tree_mode',HLP_grow_tree_mode,...
    'buffer',0.1) ;

% P.HLP = arm_manual_waypoint_HLP('manual_waypoints',manual_waypoints, 'interp_type', 'linear') ;

%% set up simulator
% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,...
    'max_sim_time',1000,'max_sim_iterations',1000) ;

%% initial plot
figure(1); clf; axis equal ; hold on ; grid on ;
view(3) ;
% campos([-10.9451    2.8278   16.5867]) ;
% campos([-0.0361  -25.5052    0.7394])

plot(A)
plot(W)

%% create .csv file for MoveIt!
write_fetch_scene_to_csv(W,csv_filename);

%% run simulation
if run_simulation_flag
    S.run()
end