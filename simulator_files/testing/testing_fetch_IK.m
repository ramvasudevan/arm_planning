%% description
% This script tries to solve IK for the Fetch to reach a particular
% end-effector position.
%
% Author: Shreyas Kousik
% Created: 25 Dec 2019
% Updated: not yet!
%
%% user parameters
% desired end effector location
p_des = [2*rand(2,1) - 1 ; 1.5*rand(1)] ;

% agent
verbosity = 0 ;

% test a bunch of end-effector positions throughout the workspace
try_a_bunch_of_ee_positions_flag = false ;

%% automated from here
% make agent
A = robot_arm_3D_fetch('verbose',verbosity, 'animation_set_axes_flag',0,...
    'animation_set_view_flag',0);

% make world
W = fetch_base_world_static('include_base_obstacle',1, 'goal_radius', 0.03,...
    'N_obstacles',0,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025,...
    'create_random_obstacles_flag',false) ;

%% run IK
tic
q = A.inverse_kinematics(p_des) ;
toc

A.reset(q) ;

%% plot
figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3)
plot(W)
plot(A) ;
plot_path(p_des,'gp','LineWidth',2)
campos([13.9632  -12.7029   18.1674])

%% set up a bunch of end-effector locations to try
if try_a_bunch_of_ee_positions_flag
    [X,Y,Z] = sphere(20) ;
    X_log = X > 0 ;
    X = X(X_log) ; Y = Y(X_log) ; Z = Z(X_log) ;
    
    % create initial set
    P_des = [X(:) Y(:) Z(:)]' ;
    
    % duplicate at smaller scales
    P_des = [1.2.*P_des, 0.8*P_des(:,1:2:end), 1.4*P_des] ;
    
    % shift Z
    P_des(3,:) = P_des(3,:) + 0.6 ;
    
    % remove stuff under the ground
    Z_log = P_des(3,:) >= 0 ;
    P_des = P_des(:,Z_log)  ;
    
    %% run IK
    % save timing vector
    N_p = size(P_des,2) ;
    t_all = nan(1,N_p) ;
    
    for idx = 1:N_p
        p_des = P_des(:,idx) ;
        
        q_0 = rand(A.n_links_and_joints,1) ;
        
        t_start = tic ;
        q = A.inverse_kinematics(p_des,q_0) ;
        t_all(idx) = toc(t_start) ;
        
        A.reset(q) ;
        
        plot_path(p_des,'gp','LineWidth',2)
        plot(A)
        drawnow
    end
    
    disp(['Average time to find solution: ',num2str(mean(t_all))])
end