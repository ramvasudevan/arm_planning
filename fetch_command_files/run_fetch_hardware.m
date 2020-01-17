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
start = [0;0;0;pi/4;0;0];
goal = [0;0;0;0;0;0];
goal_radius = 0.1;
goal_type = 'configuration';
max_iterations = 1000;
max_time = 20 ;

% scenario = 6; % 1 2 3 4 5 6 or 7
% sub_scenario = 1; % sub_scenario only changes for scenario 4

%%% EXPORTING TO CSV %%%
run_simulation_flag = true ;

%%% PLANNING PARAMETERS %%%
use_cuda_flag = true ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'
t_plan = 0.5 ;

%%% REAL LIFE PARAMETERS %%%
t_overrun = 0.005 ; % typical amount of time that the loop overruns t_plan
plot_while_running_flag = true ;

%%% OTHER PARAMS %%%
% add more obstacles
N_random_obstacles = 20 ;
create_random_obstacles_flag = true ; % in addition to the shelf
verbosity = 6 ;
allow_replan_errors = true ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = false ;
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
collision_check_time_discretization = 0.01 ;

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0,'plot_CAD_flag',plot_CAD_flag,...
    'move_mode',agent_move_mode);

%% make world
% [scenario_obstacles,start,goal_radius,goal_type,goal] = get_fetch_scenario_info(scenario) ;

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius',goal_radius,...
    'N_random_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check',1,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'create_random_obstacles_flag',create_random_obstacles_flag,...
    'goal_type',goal_type,...
    'collision_check_time_discretization',collision_check_time_discretization) ;
%     'start',start,'goal',goal) ;

%% make scenario obstacle
% W.add_obstacle(scenario_obstacles) ;

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
AI = A.get_agent_info ;
W.setup(AI)

% place arm at starting configuration
A.reset(W.start) ;

% get world info initially
WI = W.get_world_info(AI) ;

% set up planner
P.setup(AI, WI);

%% ROS setup
global ROS_t ROS_q ROS_q_dot ROS_saving_data_flag
ROS_t = []; ROS_q = []; ROS_q_dot = []; ROS_saving_data_flag = false;

rosinit('192.168.1.100', 11311);
ROS_sub = rossubscriber('/joint_states',@ROS_sub_callback) ;
[ROS_pub, ROS_msg] = rospublisher('/fetch/des_states','trajectory_msgs/JointTrajectory') ;
ROS_msg.JointNames = ["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
pause(0.1);

% ROS_saving_data_flag = true;

%% initial plop
disp('Initial plot!')
figure(1) ; clf ; hold on ; axis equal ; grid on ;
set(gca,'Projection','Perspective')
campos([16 -13 14])
plot(W)
plot(A)

%% send robit to start position
p = rosmessage('trajectory_msgs/JointTrajectoryPoint') ;
p.Positions = [W.start; 0];
p.Velocities = zeros(7, 1);
p.TimeFromStart = rosduration(8);
ROS_msg.Points = p;
send(ROS_pub, ROS_msg);

disp('Moving robot to start position!');
pause(10);

%% run hardware
start_tic = tic ;
iter_cur = 1 ;
t_agent = 0 ;
t_cur = toc(start_tic) ;
goal_reached_flag = false ;
ROS_saving_data_flag = true ;

while iter_cur < max_iterations && t_cur < max_time && (~goal_reached_flag)
    %% start timing
    in_loop_start_tic = tic ;
    
    position = [];
    velocity = [];
    time_vector = [];
    
    %% get the agent info
    AI = A.get_agent_info() ;
    
    % note that we don't need to get the world info again, since the
    % obsicles are all statique
    
    %% check world goal reached thing
    goal_reached_flag = W.goal_check(AI) ;
    
    %% call replan
    [T_ref,U_ref,Z_ref] = P.replan(AI,WI) ;
    
    %% process the traj if needed
    position = Z_ref(1:2:end, :)';
    velocity = Z_ref(2:2:end, :)';
    position = [position, zeros(size(position, 1), 1)];
    velocity = [velocity, zeros(size(velocity, 1), 1)];
    time_vector = T_ref - T_ref(1);
    
    %% make array for message
    N_pts = size(position, 1) ;
    
    % preallocate array for joint trajs
    p_array = [] ;
    for idx = 1:N_pts
        % make a joint trajectory point
        p = rosmessage('trajectory_msgs/JointTrajectoryPoint') ;
        
        % fill in the position and velocity
        p.Positions = position(idx,:) ;
        p.Velocities = velocity(idx,:) ;
        
        % fill in the duration
        p.TimeFromStart = rosduration(time_vector(idx)) ;
        
        % fill in p_array
        p_array = [p_array, p] ;
    end
    
    ROS_msg.Points = p_array;
    
    %% send traj to robit
    send(ROS_pub, ROS_msg);
    disp('Sent trajectory to ROS!!');
    
    %% update agent's state with latest info
%     A.reset    
%     if ~isempty(ROS_q)
%         A.state(A.joint_state_indices,end) = ROS_q(:,end) ;
%         A.state(A.joint_speed_indices,end) = ROS_q_dot(:,end) ; 
%     end
    
    %% move agent
    A.move(t_plan,T_ref,U_ref,Z_ref) ;
    
    in_loop_time = toc(in_loop_start_tic) ;
    
    %% plot if ya wanna
    if plot_while_running_flag
        plot(A)
    end
    
    %% pause however long is left over
    if in_loop_time < t_plan
        pause(t_plan - in_loop_time - t_overrun) ;
    end
    
    %% update iteration and timing
    t_cur = toc(start_tic) ;
    iter_cur = iter_cur + 1 ;    
end

%% pause to save the last little bit of ROS data
pause(t_plan) ;

%% yay!
if goal_reached_flag
    disp('GOAL REACHED! Hurray!!!') ;
end

rosshutdown;

%% update agent's state with the ROS
% extract planned trajectory to a variable
T_planned = A.time ;
Z_planned = A.state ;

% convert ROS time to seconds
ROS_t = ROS_t - ROS_t(1) ;

% only keep the ros time up to the last planned time, plus t_plan
ROS_t_log = ROS_t <= (T_planned(end) + t_plan) ;
ROS_t = ROS_t(ROS_t_log) ;
ROS_q = ROS_q(:,ROS_t_log) ;
ROS_q_dot = ROS_q_dot(:,ROS_t_log) ;

N_ROS_data = size(ROS_t,2) ;

% plug in state
A.time = ROS_t ;
A.state = nan(A.n_states,N_ROS_data) ;
A.state(A.joint_state_indices,:) = ROS_q ;
A.state(A.joint_speed_indices,:) = ROS_q_dot ;

%% plot stuff
figure(1) ; clf ; hold on ; axis equal ; grid on ; view(3) ;
plot(W) ;
plot(A) ;

figure(2) ; clf ;
subplot(2,1,1) ; hold on ;
h_plan = plot(T_planned, Z_planned(A.joint_state_indices,:)', 'b') ;
h_exec = plot(A.time, A.state(A.joint_state_indices,:)', 'r') ;
title('joint angles')
xlabel('time [s]')
legend([h_plan(1), h_exec(1)],'planned','executed')

subplot(2,1,2) ; hold on ;
plot(T_planned, Z_planned(A.joint_speed_indices,:)', 'b')
plot(A.time, A.state(A.joint_speed_indices,:)', 'r')
title('joint speeds')
xlabel('time [s]')

drawnow() ;

%% animate agent :)
figure(1) ;
animate(A)

%% check for collisions with virtual obstacles
tic
W.collision_check(A.get_agent_info, true);
toc

%% hlepper function
function [] = ROS_sub_callback(src, msg)
    global ROS_t ROS_q ROS_q_dot ROS_saving_data_flag

    % get the joint states out of the message
    if size(msg.Position,1) == 13 && ROS_saving_data_flag
        q = msg.Position(7:12) ;
        q_dot = msg.Velocity(7:12);

        % get the time stamp
        t_stamp = msg.Header.Stamp ;
        t = t_stamp.Sec + (t_stamp.Nsec*(1e-9)) ;

        ROS_t = [ROS_t, t];
        ROS_q = [ROS_q, q];
        ROS_q_dot = [ROS_q_dot, q_dot];

    end

% save
% get the reference trajectory up to time t_move
%                 T = 0:A.integrator_time_discretization:t_move ;
%                 [U,Z] = match_trajectories(T,T_ref,U_ref,T_ref,Z_ref) ;

% append the reference trajectory to the agent's
% current trajectory
%     z = [q(1:end-1, 1)'; q_dot(1:end-1, 1)'];
%     z = z(:);
%     u = zeros(6, 1);
%     A.commit_move_data(t,z,t,u) ;
%                 A.joint_pos_data = [A.joint_pos_data, [t;q]] ;
end