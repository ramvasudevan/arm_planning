clear; clc; close all;

% set up subscribers
sub_fetch = rossubscriber('/mocap_fetch') ;
try
    msg_fetch = sub_fetch.receive(3) ;
    p_fetch = get_position_from_message(msg_fetch) ;
catch
    disp('fetch data not received!')
    p_fetch = [0;0;0.5] ;
end
sub_obs = rossubscriber('/mocap_obs') ;

%% obs data
% obs_box_size = [0.4,0.4,0.4] ;
% o = box_obstacle_zonotope('side_lengths',obs_box_size,'center',zeros(3,1)) ;

mocap_fetch.sub = sub_fetch;
mocap_obs{1}.obs_box_size =  [0.4,0.4,0.4];
mocap_obs{1}.obstacle_object = box_obstacle_zonotope('side_lengths',mocap_obs{1}.obs_box_size,'center',zeros(3,1)) ;
mocap_obs{1}.sub = sub_obs;
mocap_obs{1}.T_obs = [-0.15 ; -0.01 ; 0.8] ; %%% SET THIS TRANSFORM MANUALLY FOR EACH OBSTACLE
mocap_obs{1}.R_obs = eul2rotm([pi/2 0 0],'xyz') ;

%% user params
start = [pi/3;pi/6;0;0;0;0];
goal = [-pi/3;-pi/6;0;0;0;0];
goal_radius = 0.1;
goal_type = 'configuration';
max_iterations = 1000;
max_time = 20 ;
run_simulation_flag = true ;
use_cuda_flag = true ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'
t_plan = 0.5 ;
t_overrun = 0.005 ; % typical amount of time that the loop overruns t_plan
plot_while_running_flag = true ;
N_random_obstacles = 0 ;
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
% use_end_effector_for_cost_flag = true ;
plot_CAD_flag = false ;
collision_check_time_discretization = 0.01 ;

% important!:
use_mocap_flag = true;

%% fetch wrrrrld stuff
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0,'plot_CAD_flag',plot_CAD_flag,...
    'move_mode',agent_move_mode);

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius',goal_radius,...
    'N_random_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check',1,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'create_random_obstacles_flag',create_random_obstacles_flag,...
    'goal_type',goal_type,...
    'collision_check_time_discretization',collision_check_time_discretization,...
    'start',start,'goal',goal) ;

W.setup(A.get_agent_info)

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
    'use_cuda_flag', use_cuda_flag, ...
    'use_mocap_flag', use_mocap_flag, ...
    'mocap_fetch', mocap_fetch, ...
    'mocap_obs', mocap_obs) ;

% set up world using arm
AI = A.get_agent_info ;
W.setup(AI)

% place arm at starting configuration
A.reset(W.start) ;

% get world info initially
WI = W.get_world_info(AI) ;

% set up planner
P.setup(AI, WI);

%% set up simulator
% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,...
    'max_sim_time',1000,'max_sim_iterations',1000) ;

%% plotting setup
disp('Initial plot!')
figure(1) ; clf ; axis equal ; grid on ; view(3) ; hold on ;
plot(W) ;
plot(A);
for i = 1:length(mocap_obs)
    plot(mocap_obs{i}.obstacle_object);
end

%% run simulation
if run_simulation_flag
    S.run()
end
