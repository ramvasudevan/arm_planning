%% user parameters
%%% WORLD PARAMETERS %%%
% manually create start
start = [0;-0.5;0;0.5;0;0] ; % on top of shelf 1 (in front of robot)

% manually create goal
% goal = [0;+1;0;-1;0;0] ; % reach to bottom of shelf 1
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

%%% IK PATH GENERATION PARAMETERS %%%
N_states = 5 ;
%%% IK PATH GENERATION PARAMETERS %%%

%%% OTHER PARAMETERS %%%
verbosity = 6 ;
%%% END OTHER PARAMETERS %%%

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0) ;

%% make world
W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03,...
    'N_obstacles',N_random_obstacles,'dimension',3,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'start',start,'goal',goal,...
    'create_random_obstacles_flag',create_random_obstacles_flag) ;

% make shelf obstacle
shelf_1 = make_shelf_obstacle(shelf_center_1,shelf_height,shelf_width,...
    shelf_depth,N_shelves,min_shelf_height,max_shelf_height,1) ;

shelf_2 = make_shelf_obstacle(shelf_center_2,shelf_height,shelf_width,...
    shelf_depth,N_shelves,min_shelf_height,max_shelf_height,2) ;

W.obstacles = [shelf_1, shelf_2] ;

%% set up arm and world
% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% set agent
A.reset(W.start)

%% get path between start and goal as end-effector positions
% get end effector at start and goal
ee_start = A.forward_kinematics_end_effector(W.start)  ;
ee_goal = A.forward_kinematics_end_effector(W.goal) ;

% generate straight-line path between start and goal
t_path = linspace(0,1,N_states+2) ; % don't include the start and goal
ee_path = match_trajectories(t_path,[0 1],[ee_start ee_goal]) ;
ee_path = ee_path(:,2:(end-1)) ;

%% run IK for each ee path position
% preallocate poses to fill in
q_path = nan(A.n_links_and_joints,N_states) ;
V_path = cell(1,N_states) ;

% run IK for each ee point
for idx = 1:N_states
    if idx == 1
        q0 = A.state(A.joint_state_indices,end) ;
    else
        q0 = q_path(:,idx-1) ;
    end
    q_path(:,idx) = A.inverse_kinematics_end_effector(ee_path(:,idx),q0) ;
    V_path{idx} = A.get_collision_check_volume(q_path(:,idx)) ;
end

%% plot
figure(1); clf; view(3); axis equal ; hold on ; grid on ;
plot(A)
plot(W)

plot_path(ee_start,'bp')
plot_path(ee_goal,'gp')
plot_path(ee_path,'kp')

% plot path
for idx = 1:N_states
    patch(V_path{idx},'FaceColor','g','FaceAlpha',0.1)
end