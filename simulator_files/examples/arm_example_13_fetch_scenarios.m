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
scenario = 7; % 1 2 3 4 5 6 or 7
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
use_cuda_flag = false ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'

% add more obstacles
N_random_obstacles = 6 ; % NOTE we should no longer set W.N_obstacles
create_random_obstacles_flag = false ; % in addition to the shelf
verbosity = 6 ;
allow_replan_errors = true ;
actual_t_plan = 10 ;
simulated_t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = true ; 
HLP_timeout = 2 ; 
HLP_grow_tree_mode = 'new' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.3 ;
use_end_effector_for_cost_flag = true ;
plot_CAD_flag = false ; % plot the faaaaancy arm :)

%%% WORLD PARAMETERS %%%
scenario_obstacles = {};
switch scenario
    case 1 % table
        start = [0; 0.5; 0; -0.5; 0; 0];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [0;-0.5;0; 0.5;0;0];
        obs_center = [1.1; 0; 0.8];
        obs_side_lengths = [1, 4, 0.01];
        % create obstacle
        O = box_obstacle_zonotope('center',obs_center(:),...
                'side_lengths',obs_side_lengths) ;
            
        scenario_obstacles = {O};
        
    case 2 % wall/doorway
        start = [-0.5; 0; 0; 0; 0; 0];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [0.5;0;0; 0.5;0;0];
        obs_center = [1.1; 0; 0.8];
        obs_side_lengths = [1, 0.01, 4];
        % create obstacle
        O = box_obstacle_zonotope('center',obs_center(:),...
                'side_lengths',obs_side_lengths) ;
            
        scenario_obstacles = {O};        
    case 3 % posts
        start = [-0.6; 0; 0; 0; 0; 0];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [0.15;-0.75;0.2; 0.4;0.3;0.2];
        obs_center = [0.8; -0.25; 2];
        obs_side_lengths = [0.05, 0.05, 4];
        % create obstacle
        O1 = box_obstacle_zonotope('center',obs_center(:),...
                'side_lengths',obs_side_lengths) ;
        obs_center = [0.4; 0.25; 2];
        obs_side_lengths = [0.05, 0.05, 4];
        % create obstacle
        O2 = box_obstacle_zonotope('center',obs_center(:),...
                'side_lengths',obs_side_lengths) ;
            
        scenario_obstacles = {O1, O2};        
        
    case 4 % shelves
        % manually create start
        start = [0;-0.5;0;0.5;0;0] ; % on top of shelf 1 (in front of robot)

        % manually create goal
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'

        % configuration goals (set goal_type to 'configuration')
        % goal = [0;+1;0;-1;0;0] ; % bottom of front shelf
        % goal = [0.5;-0.5;0;0.5;0;0] ; % left of front shelf
        % goal = [0.5;+1;0;-1;0;0] ; bottom left of front shelf
        goal = [pi/2 ; -0.5;0;0.5;0;0] ; % top of left shelf
        % goal = [pi/2 ;+1;0;-1;0;0] ; % bottom of left shelf

        % end effector location goals (set goal_type to 'end_effector_location')
        % goal = [0; 1 ; 0.4] ; bottom middle of left shelf
        % goal = [0.25 ; 1 ; 1] ; top right of left shelf
        % goal = [1 ; -0.4 ; 0.6] ; % bottom right of front shelf
        % goal = [-0.3 ; 1 ; 0.6] ; % bottom of left shelf

        % shelf parameters
        shelf_center_1 = [1.1 ; 0 ; 0.7] ;
        shelf_center_2 = [0 ; 1.1 ; 0.7] ;
        shelf_height = 1.4 ; % m
        shelf_width = 1.2 ; % m 
        shelf_depth = 0.8 ; % m
        N_shelves = 3 ;
        min_shelf_height = 0.3 ;
        max_shelf_height = 1.3 ;
        
        % create shelves
        shelf_1 = make_shelf_obstacle(shelf_center_1,shelf_height,shelf_width,...
            shelf_depth,N_shelves,min_shelf_height,max_shelf_height,1) ;
        
        shelf_2 = make_shelf_obstacle(shelf_center_2,shelf_height,shelf_width,...
            shelf_depth,N_shelves,min_shelf_height,max_shelf_height,2) ;
        
        scenario_obstacles = [shelf_1, shelf_2];
        
    case 5 % inside box
%         start = [0; -pi/12; 0; pi/3; 0; pi/3];
        start = [0; 0; 0; pi/2; 0; 0];
%         start = [0.2; 0.2; 0.2; 0.2; 0.2; 0.2];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [0.15;-0.75;0.2; 0.4;0.3;0.2];
%         goal = [0.5; 0.2; -0.3; 0.4; -0.2; 0.7];
        
        total_box_side_lengths = [0.4, 0.4, 0.66];
        total_box_center = [0.45, 0, total_box_side_lengths(3)/2];
%         total_box_center = [0.7, 0, total_box_side_lengths(3)/2];

        
        obs_center = [total_box_center(1); total_box_center(2) + total_box_side_lengths(2)/2; total_box_center(3)];
        obs_side_lengths = [total_box_side_lengths(1), 0.01, total_box_side_lengths(3)];
        O1 = box_obstacle_zonotope('center',obs_center(:),'side_lengths',obs_side_lengths) ;
        
        obs_center = [total_box_center(1) - total_box_side_lengths(1)/2; total_box_center(2); total_box_center(3)];
        obs_side_lengths = [0.01, total_box_side_lengths(2), total_box_side_lengths(3)];
        O2 = box_obstacle_zonotope('center',obs_center(:),'side_lengths',obs_side_lengths) ;
        
        obs_center = [total_box_center(1); total_box_center(2) - total_box_side_lengths(2)/2; total_box_center(3)];
        obs_side_lengths = [total_box_side_lengths(1), 0.01, total_box_side_lengths(3)];
        O3 = box_obstacle_zonotope('center',obs_center(:),'side_lengths',obs_side_lengths) ;
        
        obs_center = [total_box_center(1) + total_box_side_lengths(1)/2; total_box_center(2); total_box_center(3)];
        obs_side_lengths = [0.01, total_box_side_lengths(2), total_box_side_lengths(3)];
        O4 = box_obstacle_zonotope('center',obs_center(:),'side_lengths',obs_side_lengths) ;
        
        scenario_obstacles = {O1, O2, O3, O4};
    case 6 % sink to cupboard
        start = [0; -pi/6; 0; pi/3+0.15; 0; pi/3];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [-pi/3;-pi/3;pi/6; pi/6;+pi/3;pi/6];
        
        % counter size
        counter_center = [0.6; 0; 0.6];
        counter_length = 0.5;
        counter_width = 2;
        
        sink_width = 0.5;
        sink_depth = 0.3;
        
        cupboard_center = [0.6; -0.55; 1.4];
        cupboard_length = counter_length;
        cupboard_width = 0.5;
        cupboard_depth = 0.5;
        
        % create counter
        obs_center = counter_center + [0; sink_width/2 + counter_width/2; 0];
        obs_side_lengths = [counter_length, counter_width, 0.01];
        O1 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = counter_center + [0; -sink_width/2 - counter_width/2; 0];
        obs_side_lengths = [counter_length, counter_width, 0.01];
        O2 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
                
        % create sink
        obs_center = counter_center + [0; sink_width/2; -sink_depth/2];
        obs_side_lengths = [sink_width, 0.01, sink_depth];
        O3 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = counter_center + [0; -sink_width/2; -sink_depth/2];
        obs_side_lengths = [sink_width, 0.01, sink_depth];
        O4 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = counter_center + [sink_width/2; 0; -sink_depth/2];
        obs_side_lengths = [0.01, sink_width, sink_depth];
        O5 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = counter_center + [-sink_width/2; 0; -sink_depth/2];
        obs_side_lengths = [0.01, sink_width, sink_depth];
        O6 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = counter_center + [0; 0; -sink_depth];
        obs_side_lengths = [sink_width, sink_width, 0.01];
        O7 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        % create cupboard
        obs_center = cupboard_center + [0; cupboard_width/2; 0];
        obs_side_lengths = [cupboard_length, 0.01, cupboard_depth];
        O8 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = cupboard_center + [0; -cupboard_width/2; 0];
        obs_side_lengths = [cupboard_length, 0.01, cupboard_depth];
        O9 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = cupboard_center + [0; 0; cupboard_depth/2];
        obs_side_lengths = [cupboard_length, cupboard_width, 0.01];
        O10 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = cupboard_center + [0; 0; -cupboard_depth/2];
        obs_side_lengths = [cupboard_length, cupboard_width, 0.01];
        O11 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        obs_center = cupboard_center + [cupboard_length/2; 0; 0];
        obs_side_lengths = [0.01, cupboard_width, cupboard_depth];
        O12 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        
        scenario_obstacles = {O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11, O12};
        
        
    case 7 % reach through window
        start = [-pi/2; 0; pi/2; pi/4; 0; 0];
        goal_radius = 0.05 ;
        goal_type = 'end_effector_location' ; % 'configuration' or 'end_effector_location'
        goal = [0;0;0; 0;pi/3;pi/3];
        
        % window size
        window_center = [0.6; 0; 0.8];
        window_side_length = 0.5;
        
        obs_height = 1.5;
        obs_width = 1.5;
        
        % create obstacle
        obs_center = window_center + [0; 0; -window_side_length/2 - obs_height/2];
        obs_side_lengths = [0.01, 4, obs_height];
        O1 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        % create obstacle
        obs_center = window_center + [0; 0; +window_side_length/2 + obs_height/2];
        obs_side_lengths = [0.01, 4, obs_height];
        O2 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        % create obstacle
        obs_center = window_center + [0; -window_side_length/2 - obs_width/2; 0];
        obs_side_lengths = [0.01, obs_width, 4];
        O3 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        % create obstacle
        obs_center = window_center + [0; +window_side_length/2 + obs_width/2; 0];
        obs_side_lengths = [0.01, obs_width, 4];
        O4 = box_obstacle_zonotope('center',obs_center(:),...
            'side_lengths',obs_side_lengths) ;
        
        scenario_obstacles = {O1, O2, O3, O4};
        
        
    otherwise error('scenario not recognized');
end
%%% END WORLD PARAMETERS %%%

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
% % check for translating configuration into EE goal (edit: unnecessary)
% if strcmp(goal_type, 'end_effector_location') && length(goal) == 6
%     [~, ~, J] = A.forward_kinematics(goal);
%     goal = J(:, end);
% if strcmp(goal_type, 'configuration') && length(goal) == 3
%     Q = A.inverse_kinematics_end_effector(goal);
%     goal = Q;
% end

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius',goal_radius,...
    'N_random_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check',1,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'start',start,'goal',goal,...
    'create_random_obstacles_flag',create_random_obstacles_flag,...
    'goal_type',goal_type) ;

%% make shelf obstacle

W.add_obstacle(scenario_obstacles) ;

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
plot(A)
plot(W)

%% create .csv file for MoveIt!
if write_to_csv
    write_fetch_scene_to_csv(W,csv_filename);
end

%% run simulation
if run_simulation_flag
    S.run()
end