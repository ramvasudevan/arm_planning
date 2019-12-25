%% description
% This script tests generating collision-free configurations
%
% Authors: Shreyas Kousik
% Created: 24 Dec 2019
% Updated: -
%
%% user parameters

%%% WORLD PARAMETERS %%%
% manually create start
start = [0;-0.5;0;0.5;0;0] ; % on top of shelf 1 (in front of robot)
% start = [-0.1783
%    -0.2498
%     1.1342
%     0.2476
%    -0.6828
%     0.4835] ;

% manually create goal
% goal = [0;+0.5;0;-0.5;0;0] ; % reach to bottom of shelf 1
% goal = [0.5;-0.5;0;0.5;0;0] ; % reach to the left of shelf 1
% goal = [0.5;+1;0;-1;0;0] ; % reach down and to the left of shelf 1
% goal = [pi/2 ; -0.5;0;0.5;0;0] ; % on top of shelf 2
goal = [pi/2 ;+1;0;-1;0;0] ; % bottom of shelf 2

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


%%% OPTIMIZATION PARAMETERS
buffer_dist = 0.1 ; % for obstacles
start_pct = 0 ; % straight line between 0 (W.start) and 1 (W.goal)
goal_pct = 1 ;
run_style = 'single' ; % choose 'single' or 'multi'
N_states = 8 ; % number of states along planned path for 'multi' opt
%%% END OPTIMIZATION PARAMETERS

%%% OTHER PARAMETERS %%%
verbosity = 6 ;
%%% END OTHER PARAMETERS %%%

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0);

% can adjust tracking controller gains here
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;

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

% set up world using arm
I = A.get_agent_info ;
W.setup(I)
O = W.obstacles ;

% % place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

%% run single state optimization
if strcmp(run_style,'single')
    % start and goal poses
    x_start_and_goal = match_trajectories([start_pct goal_pct],[0 1],[W.start W.goal]) ;
    
    x_start = x_start_and_goal (:,1) ;
    x_goal = x_start_and_goal (:,2) ;
    
%     x_start = [-0.1759
%         -0.7665
%         1.3631
%         1.5136
%         0.6364
%         0.2228 ];
%     
%     x_goal = [-0.4437
%         -0.5744
%         1.3425
%         1.1253
%         0.1235
%         0.4543] ;
    
    % get start and goal end effector locations
    J_start = get_collision_check_locations(A,x_start) ;
    J_start = J_start(:,end) ;
    
    J_goal = get_collision_check_locations(A,x_goal) ;
    J_goal = J_goal(:,end) ;
    
    % make cost and constraints
    cost = @(x) single_state_cost(x,A,J_start,J_goal) ;
    nonlcon = @(x) single_state_nonlcon(x,A,W.obstacles,0.1) ;
    
    % make initial guess
    x_0 = 0.5.*(x_start + x_goal) ;
    
    tic
    x_opt = fmincon(cost,x_0,[],[],[],[],A.joint_state_limits(1,:)',A.joint_state_limits(2,:)',nonlcon) ;
    toc
    
    V_opt_single_state = A.get_collision_check_volume(x_opt) ;
    V_start = A.get_collision_check_volume(x_start) ;
    V_goal = A.get_collision_check_volume(x_goal) ;
end

%% run multi-state optimization
if strcmp(run_style,'multi')
    % get info about obstacles and links
    N_obs = length(O) ;
    N_links = A.n_links_and_joints ;
    
    % make straight line guess
    t = linspace(0,1,N_states) ;
    x_line = match_trajectories(t,[0 1],[W.start W.goal]) ;
    
    % set up fmincon problem
    goal_weight = 1 ;
    traj_weight = sqrt(N_links)/N_states ; % normalize for "size" of config space and length of traj
    
    cost = @(x) trajopt_cost(x,x_line,N_states,N_links) ;
    nonlcon = @(x) trajopt_nonlcon(x,A,O,buffer_dist,N_states,N_links,N_obs) ;
    
    % make lower and upper bounds
    lb = repmat(A.joint_state_limits(1,:)',N_states,1) ;
    ub = repmat(A.joint_state_limits(2,:)',N_states,1) ;
    
    % set up options
    options = optimoptions('fmincon','MaxFunctionEvaluations',3e4,...
        'OptimalityTolerance',1e-3,'ConstraintTolerance',1e-5) ;
    
    % call fmincon
    tic
    x_opt = fmincon(cost,x_line(:),[],[],[],[],lb,ub,nonlcon,options) ;
    toc
    
    % generate volumes for plotting solution
    x = reshape(x_opt,N_links,N_states) ;
    V_opt = cell(1,N_states) ;
    for idx = 1:N_states
        x_opt = x(:,idx) ;
        V_opt{idx} = A.get_collision_check_volume(x_opt) ;
    end
end

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3)
plot(W)
plot(A)

switch run_style
    case 'single'
        % plot single state optimization result
        patch(V_opt_single_state,'facecolor','g','facealpha',0.2)
        patch(V_start,'facecolor',[0.5 0.5 0.5],'facealpha',0.2)
        patch(V_goal,'facecolor',[0.5 0.5 0.5],'facealpha',0.2)
        
    case 'multi'
        % % plot multi-state optimization result
        for idx = 1:N_states
            patch(V_opt{idx},'facecolor',[0 (idx/N_states) (1 - (idx/N_states))],'facealpha',0.2)
        end
end

%% helper functions: single state cost and constraints
function c = single_state_cost(x,A,J_start,J_goal)
J = get_collision_check_locations(A,x) ;
c = vecnorm(J_goal - J(:,end)).^2 + vecnorm(J_start - J(:,end)).^2 ;
end

function [n,neq] = single_state_nonlcon(x,A,O,buffer_dist)
J = get_collision_check_locations(A,x) ;
n = -(cell2mat(cellfun(@(o) dist_point_to_box(J,...
    o.side_lengths(1),o.side_lengths(2),o.side_lengths(3),o.center),...
    O,'UniformOutput',false)) - buffer_dist);
n = n(:) ;
neq = [] ;
end

%% helper functions: multi-state cost and constraints
function c = trajopt_cost(x,x_line,N_states,N_links)
% get cost of distance from each state to the line between start and goal
x = reshape(x,N_links,N_states) ;
c = sum(vecnorm(x - x_line).^2) ;
end

function [n,neq] = trajopt_nonlcon(x,A,O,buffer_dist,N_states,N_links,N_obs)
% get the states
x = reshape(x,N_links,N_states) ;

% preallocate n
n = zeros(N_links*N_obs,N_states) ;


% get n for obstacle avoidance and neq for start and goal end-effector
for idx = 1:N_states
    % get joint locations for the current state
    J = get_collision_check_locations(A,x(:,idx)) ;
    
    % check collision for the current state
    n(:,idx) = -(cell2mat(cellfun(@(o) dist_point_to_box(J,...
        o.side_lengths(1),o.side_lengths(2),o.side_lengths(3),o.center),...
        O,'UniformOutput',false)) - buffer_dist) ;
end
% make into a column vector
n = n(:) ;

% assign neq
neq = [] ;
end

%% link joint locations (helper function)
function J = get_collision_check_locations(A,q)
j_vals = q ; % joint angles
j_locs = A.joint_locations ; % joint locations

% extract dimensions
n = A.n_links_and_joints ;
d = A.dimension ;

% set up translations and rotations
R_pred = eye(d) ;
T_pred = zeros(d,1) ;

% allocate array for the joint locations
J = nan(d,n) ;

% move through the kinematic chain and get the rotations and
% translation of each link
for idx = 1:n
    % get the value and location of the current joint
    j_idx = j_vals(idx) ;
    j_loc = j_locs(:,idx) ;
    
    % rotation matrix of current joint
    axis_pred = R_pred*A.joint_axes(:,idx) ;
    R_succ = axis_angle_to_rotation_matrix_3D([axis_pred', j_idx])*R_pred ;
    
    % create translation
    T_succ = T_pred + R_pred*j_loc(1:d) - R_succ*j_loc(d+1:end) ;
    
    % fill in the joint location
    j_loc_local = j_locs((d+1):end,idx) ;
    J(:,idx) = -R_succ*j_loc_local + T_succ ;
    
    % update predecessors for next iteration
    R_pred = R_succ ;
    T_pred = T_succ ;
end

% add more points
t = 1:n ;
t2 = linspace(1,n,3*n) ;
J = match_trajectories(t2,t,J) ;
end