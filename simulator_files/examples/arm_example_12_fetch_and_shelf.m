%% description
% This script sets up a Fetch robot in an environment with a shelf-like
% obstacle.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 23 Dec 2019
% Updated: 23 Dec 2019
%
%% user parameters
%%% WORLD PARAMETERS %%%
% manually create start
start = [0;-0.5;0;0.5;0;0] ; % on top shelf

% manually create goal
goal = [0;+1;0;-1;0;0] ; % reach to bottom of shelf 1
% goal = [0.5;-0.5;0;0.5;0;0] ; % reach to the left of shelf 1
% goal = [0.5;+1;0;-1;0;0] ; % reach down and to the left of shelf 1
goal = [pi/2 ; -0.5;0;0.5;0;0] ; % on top of shelf 2

% shelf parameters
shelf_center_1 = [1.2 ; 0 ; 0.6] ;
shelf_center_2 = [0 ; 1.2 ; 0.6] ;
shelf_height = 1.2 ; % m
shelf_width = 1.2 ; % m 
shelf_depth = 0.8 ; % m
N_shelves = 3 ;
min_shelf_height = 0.3 ;
max_shelf_height = 1.2 ;

% add more obstacles
N_random_obstacles = 2 ;
create_random_obstacles_flag = false ; % in addition to the shelf
%%% END WORLD PARAMETERS %%%

%%% OTHER PARAMETERS %%%
nLinks = 3 ;
verbosity = 6 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = false ; 
run_simulation_flag = true ;
HLP_timeout = 2 ; 
plot_while_sampling_flag = true ;
make_new_graph_every_iteration = false ;
plot_waypoint_flag = true ;
%%% END OTHER PARAMETERS %%%

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);

% can adjust LLC gains here
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;

%% make world
W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03,...
    'N_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'start',start,'goal',goal,...
    'create_random_obstacles_flag',create_random_obstacles_flag) ;

%% make shelf obstacle
shelf_1 = make_shelf_obstacle(shelf_center_1,shelf_height,shelf_width,...
    shelf_depth,N_shelves,min_shelf_height,max_shelf_height,1) ;

shelf_2 = make_shelf_obstacle(shelf_center_2,shelf_height,shelf_width,...
    shelf_depth,N_shelves,min_shelf_height,max_shelf_height,2) ;

W.obstacles = [shelf_1, shelf_2] ;

%% make planner
FRS_options = struct() ;
FRS_options.t_plan = t_plan;
FRS_options.origin_shift = A.joint_locations(1:3, 1);
FRS_options.T = T;
FRS_options.L = 0.33;
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;
P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options,...
    'verbose', verbosity, 't_plan', t_plan,...
    'time_discretization', time_discretization,...
    'first_iter_pause_flag',first_iter_pause_flag) ;

P.HLP = robot_arm_RRT_HLP('sampling_timeout',HLP_timeout,...
    'plot_while_sampling_flag',plot_while_sampling_flag,...
    'plot_waypoint_flag',plot_waypoint_flag,...
    'make_new_graph_every_iteration_flag',make_new_graph_every_iteration,...
    'new_node_growth_distance',0.5,...
    'choose_goal_as_new_node_frequency',0.5) ;

%% set up simulator
% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% % place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',1000,'max_sim_iterations',1000) ;

%% initial plot
figure(1); clf; view(3); axis equal ; hold on ; grid on ;
plot(S)

%% create .csv file for MoveIt!
% write_fetch_scene_to_csv(W);

%% run simulation
if run_simulation_flag
    S.run()
end