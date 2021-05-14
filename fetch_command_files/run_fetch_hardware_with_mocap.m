%% description
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 14 Jan 2019
% Updated: ---
%
clear; clc;
use_mocap_flag = true;

%% user parameters

%%% CHOOSE SCENARIO %%%
start = [1.5793;
    0.2709;
    0.0869;
    0.2470;
   -0.6233;
    0.3989];

goal = [-0.8872;
   -0.5027;
   -2.0383;
    1.7616;
    0.4742;
   -0.8090];
% start = [pi/3;pi/6;0;pi/12;0;0];
% goal = [-pi/3;-pi/6;0;-pi/12;0;0];
% start = [-1.4300; -0.6806 ;1.7076 ;-1.0997 ;-0.7937 ;1.1532];
% goal = [1.5705 ;-0.7206 ;2.2161;-1.0843;2.3771;0.2447];
% start = [-pi/2; 0; 0; 0; 0; 0];
% goal = [pi/2; 0; 0; 0; 0; 0];
goal_radius = 0.1;
goal_type = 'configuration';
max_iterations = 1000;
max_time = 100 ;

%%%%TABLE
% start = [0; pi/3; 0; -pi/3; 0; 0]; % under table
% start = [0;0.125;0;-0.125;0;0];
start = [-pi/3;0;0;0;0;0];
goal_radius = pi/30 ;
goal_type = 'configuration' ;
% goal = [pi/4;-0.5;0; 0.5;0;0];
% goal = [0;0.0;0;-0.0;0;0];
goal = [pi/3;0.0;0;-0.0;0;0];
shelf_center_xy = [0.8;0] ; % this is the default
% add more obstacles
manual_waypoints = [start(:),...
    [0; pi/3; 0; -pi/3; pi/3; pi/3], ...
    [-pi/3; pi/3; 0; -pi/3; pi/2; pi/2], ...
    [-pi/2; 0; 0; 0; pi/2; pi/2],...
    [-pi/3; -pi/8; 0; pi/8; pi/2; pi/2], ...
    goal(:)] ;
%%%

% manual_waypoints = fliplr(manual_waypoints);
% bleh = start;
% start = goal;
% goal = bleh;

% %%%%% CABINET
% % manually create start
% % start = [0; 0 ; 0 ; pi/2 ; 0 ; -pi/2.2 ] ; % on shelf 5
% start = [0; -pi/4; pi ; pi/7 ; 0 ; -pi/2.7 ] ; % on shelf 2
% % start = [0.6382;
% %     0.1977;
% %     4.3612;
% %     0.7286;
% %    -2.6181;
% %    -1.1285]; % random one on shelf 4
% % manually create goal
% goal_radius = pi/30 ;
% goal_type = 'configuration' ;
% % configuration goals (set goal_type to 'configuration')
% goal = [0; -pi/4; pi ; pi/7 ; 0 ; -pi/2.7 ] ; % on shelf 2
% % goal = [-pi/5; -pi/3.7; pi/5 ; pi/3.7 ; 0 ; 0.1 ] ; % on shelf 3
% % goal = [0; -pi/4.2; 0 ; pi/1.7 ; 0 ; -pi/3 ] ; % on shelf 4
% goal = [0; 0 ; 0 ; pi/2 ; 0 ; -pi/2.2 ] ; % on shelf 5
% % shelf parameters
% shelf_center_xy = [0.8;0] ; % this is the default
% 
% % add more obstacles
% %%% MANUAL WAYPOINTS IF YA WANT 'EM %%%
% manual_waypoints = [start(:), [0; 0 ; 0 ; pi/2 ; -pi/3 ; -pi/2.2],...
%     [0; 0 ; 3*pi/4 ; pi/2 ; -pi/2 ; -pi/2.2], [0; -pi/4 ; pi ; pi/4 ; -pi/2 ; -pi/2], ...
%     [0; -pi/4 ; pi ; pi/4 ; -pi/3 ; -pi/2], goal(:)] ; % this for shelf 5 to shelf 2
% % manual_waypoints = [start(:), [0; -pi/4 ; pi ; pi/4 ; -pi/3 ; -pi/2],...
% %     [0; -pi/4 ; pi/2 ; pi/2 ; -pi/2 ; -pi/2], ...
% %     [0; -pi/4 ;0 ; pi/1.5 ; -pi/2 ; -pi/2],...
% %     [0; -pi/4 ;0 ; pi/1.5 ; -pi/3 ; -pi/2.2], ...
% %     goal(:)] ;  % this for shelf 2 to shelf 4 %edit doesn't work!
% %%%%
run_simulation_flag = true ;

use_cuda_flag = true ;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'
t_plan = 0.5 ;

%%% REAL LIFE PARAMETERS %%%
t_overrun = 0.005 ; % typical amount of time that the loop overruns t_plan
plot_while_running_flag = true ;
save_filename = 'arm_planning/fetch_command_files/fetch_hardware_demo_20200128/fetch_hardware_tablebox_3.mat' ;

%%% OTHER PARAMS %%%
% add more obstacles
N_random_obstacles = 0 ;
create_random_obstacles_flag = false ; % in addition to the shelf
verbosity = 6 ;
allow_replan_errors = true ;
time_discretization = 0.01 ;
T = 1 ;
first_iter_pause_flag = false ;
HLP_timeout = 0.2 ;
HLP_grow_tree_mode = 'new' ;
plot_while_sampling_flag = false ;
make_new_graph_every_iteration = false ;
plot_HLP_flag = true ; % for planner
plot_waypoint_flag = true ; % for HLP
plot_waypoint_arm_flag  = true ; % for HLP
lookahead_distance = 0.5 ;
use_end_effector_for_cost_flag = false ;
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
    'collision_check_time_discretization',collision_check_time_discretization,...
    'start',start,'goal',goal) ;

%% make scenario obstacle
% W.add_obstacle(scenario_obstacles) ;

C = make_roahmlab_table_obstacle(shelf_center_xy) ;
W.add_obstacle(C)

% mybox = box_obstacle_zonotope('center', [0.7; 0; 0.8], 'side_lengths', [0.3 0.3 0.3]);
% mybox2 = box_obstacle_zonotope('center', [0.7; 0; 1.1], 'side_lengths', [0.3 0.3 0.3]);
% W.add_obstacle({mybox, mybox2});

% C = make_roahmlab_cabinet_obstacle(shelf_center_xy) ;
% W.add_obstacle(C)

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
% 
P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
    'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
    'grow_tree_mode',HLP_grow_tree_mode,...
    'buffer',0.1) ;

% P.HLP = arm_manual_waypoint_HLP('manual_waypoints',manual_waypoints, 'interp_type', 'linear') ;

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

fetch_ros_init
ROS_sub = rossubscriber('/joint_states',@ROS_sub_callback) ;
[ROS_pub, ROS_msg] = rospublisher('/fetch/des_states','trajectory_msgs/JointTrajectory') ;
ROS_msg.JointNames = ["shoulder_pan_joint" "shoulder_lift_joint" "upperarm_roll_joint" "elbow_flex_joint"...
    "forearm_roll_joint" "wrist_flex_joint" "wrist_roll_joint"];
pause(0.1);

%% MOCAP setup
% set up subscribers
if use_mocap_flag
    MOCAP_fetch_sub = rossubscriber('/mocap_fetch') ;
    default_p_fetch = zeros(3, 1);
    try
        MOCAP_fetch_msg = MOCAP_fetch_sub.receive(3) ;
        p_fetch = get_mocap_position_from_message(MOCAP_fetch_msg) ;
    catch
        disp('fetch data not received!')
        p_fetch = default_p_fetch ;
    end

    fetch_origin_to_markers = [-0.25;0;0.95];
    R_mocap = eul2rotm([pi/2 0 0],'xyz') ;
    MOCAP_obs{1}.sub = rossubscriber('/mocap_obs') ;
    % MOCAP_obs{1}.obs_box_size =  [0.32,0.32,0.32];

    % box obstacle!
    % MOCAP_obs{1}.obs_box_size =  [0.32,0.32,0.32];
    % MOCAP_obs{1}.obstacle_object = box_obstacle_zonotope('side_lengths',MOCAP_obs{1}.obs_box_size,'center',zeros(3,1)) ;
    % MOCAP_obs{1}.T_obs = [0;-0.05;-0.13] ; %%% SET THIS TRANSFORM MANUALLY FOR EACH OBSTACLE
    % MOCAP_obs{1}.R_obs = eul2rotm([pi/2 0 0],'xyz') ;

    % quadrotor obstacle!
    % MOCAP_obs{1}.obs_box_size = [0.23;0.23;0.12] ; % for quadrotor
    % MOCAP_obs{1}.obstacle_object = box_obstacle_zonotope('side_lengths',MOCAP_obs{1}.obs_box_size,'center',zeros(3,1)) ;
    % MOCAP_obs{1}.T_obs = [0;+0.01;0];
    % MOCAP_obs{1}.R_obs = eul2rotm([pi/2 0 0],'xyz') ;

    % vase obstacle!
    MOCAP_obs{1}.obs_box_size = [0.2; 0.2; 0.3]; % for vase
    MOCAP_obs{1}.obstacle_object = box_obstacle_zonotope('side_lengths',MOCAP_obs{1}.obs_box_size,'center',zeros(3,1)) ;
    MOCAP_obs{1}.T_obs = [0;0;0];
    MOCAP_obs{1}.R_obs = eul2rotm([pi/2 0 0],'xyz') ;
end


%% initial plop
disp('Initial plot!')
figure(1) ; clf ; hold on ; axis equal ; grid on ;
set(gca,'Projection','Perspective')
campos([16 -13 14])
plot(W)
plot(A)
if use_mocap_flag
    for i = 1:length(MOCAP_obs)
        plot(MOCAP_obs{i}.obstacle_object);
    end
end

disp('Send robot to start position?');
pause();

%% send robit to start position
p = rosmessage('trajectory_msgs/JointTrajectoryPoint') ;
p.Positions = [W.start; 0];
p.Velocities = zeros(7, 1);
p.TimeFromStart = rosduration(8);
ROS_msg.Points = p;
send(ROS_pub, ROS_msg);

disp('Moving robot to start position!');
pause(10);

disp('Send robot to goal position?');
pause();

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
    
    %% get permanent world info
    WI_tmp = WI;
    
    %% get the agent info
    AI = A.get_agent_info() ;
    
    % note that we don't need to get the world info again, since the
    % obsicles are all statique
    
    %% check world goal reached thing
    goal_reached_flag = W.goal_check(AI) ;
    
    %% update obstacles from motion capture
%     try
    if use_mocap_flag
        MOCAP_fetch_msg = MOCAP_fetch_sub.receive(3) ;
        mocap_origin_to_markers = R_mocap*get_mocap_position_from_message(MOCAP_fetch_msg);
        origin_shift = mocap_origin_to_markers - fetch_origin_to_markers;
        %     catch
        %         disp('fetch data not received!')
        %         p_fetch = default_p_fetch ;
        %     end
        for i = 1:length(MOCAP_obs)
            disp('getting obs mocap data')
            msg_obs = MOCAP_obs{i}.sub.receive() ;
            % get the difference between the two positions
            d_obs = R_mocap*get_mocap_position_from_message(msg_obs);
            % disp('transforming obs to wrrrld frame')
            p_obs = d_obs + MOCAP_obs{i}.T_obs - origin_shift; % new center of obsicle
            
            % fix the obstacle center
            MOCAP_obs{i}.obstacle_object.change_center(p_obs) ;
            
            %disp('updating plot')
            V = MOCAP_obs{i}.obstacle_object.plot_data.body.Vertices ;
            mV = mean(V,1) ;
            MOCAP_obs{i}.obstacle_object.plot_data.body.Vertices = V - mV + repmat(p_obs(:)',size(V,1),1) ;
            
            % update obstacle list
            WI_tmp.obstacles{end+1} = MOCAP_obs{i}.obstacle_object;
            pause(0.005)
        end
    end
    
    %% call replan
    [T_ref,U_ref,Z_ref] = P.replan(AI,WI_tmp) ;
    
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
% T_planned = A.time ;
% Z_planned = A.state ;
% 
% % convert ROS time to seconds
% ROS_t = ROS_t - ROS_t(1) ;
% 
% % only keep the ros time up to the last planned time, plus t_plan
% ROS_t_log = ROS_t <= (T_planned(end) + t_plan) ;
% ROS_t = ROS_t(ROS_t_log) ;
% ROS_q = ROS_q(:,ROS_t_log) ;
% ROS_q_dot = ROS_q_dot(:,ROS_t_log) ;
% 
% N_ROS_data = size(ROS_t,2) ;
% 
% % plug in state
% A.time = ROS_t ;
% A.state = nan(A.n_states,N_ROS_data) ;
% A.state(A.joint_state_indices,:) = ROS_q ;
% A.state(A.joint_speed_indices,:) = ROS_q_dot ;

%% plot stuff
figure(1) ; clf ; hold on ; axis equal ; grid on ; view(3) ;
plot(W) ;
plot(A) ;

% figure(2) ; clf ;
% subplot(2,1,1) ; hold on ;
% h_plan = plot(T_planned, Z_planned(A.joint_state_indices,:)', 'b') ;
% h_exec = plot(A.time, A.state(A.joint_state_indices,:)', 'r') ;
% title('joint angles')
% xlabel('time [s]')
% legend([h_plan(1), h_exec(1)],'planned','executed')
% 
% subplot(2,1,2) ; hold on ;
% plot(T_planned, Z_planned(A.joint_speed_indices,:)', 'b')
% plot(A.time, A.state(A.joint_speed_indices,:)', 'r')
% title('joint speeds')
% xlabel('time [s]')
% 
% drawnow() ;

%% animate agent :)
figure(1) ;
% animate(A)

%% check for collisions with virtual obstacles
tic
% W.collision_check(A.get_agent_info, true);
toc

%% save things
save_fetch_hardware_data

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