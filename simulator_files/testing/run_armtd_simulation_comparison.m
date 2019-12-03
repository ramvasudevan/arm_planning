%% description
% This script iterates through a list of presaved random worlds and runs
% the aRmTD planner on them. It then saves information on how well each the
% planner performed in each trial.
%
% Authors: Patrick Holmes (adapted from Shreyas Kousik code)
% Created 25 November 2019

clear; clc;

%% user parameters
goal_radius = 0.1;
dimension = 3 ;
nLinks = 3 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
floor_normal_axis = 1;

% plotting
plot_during_sim = true ;
% agent_camera_distance = 3 ; % default is 3
% agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
% plot_agent_view = 'behind' ; % none, behind, above, or onboard
% plot_zonotopes = true ;

% simulation
% start_idx = 5 ;
% end_idx = 500 ;
verbosity = 10 ;
max_sim_time = 300 ;
max_sim_iter = 50 ;

% file handling
% if ~exist('arm_planning/simulator_files/testing/trial_data', 'dir')
%     mkdir('arm_planning/simulator_files/testing/trial_data');
% end
save_file_header = 'trial_20191125_' ;
file_location = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/' ;

% world file
world_file_header = '20191111';
world_file_folder = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/saved_worlds/';
world_file_location = sprintf('%s*%s*', world_file_folder, world_file_header);
world_file_list = dir(world_file_location);


%% automated from here
% run loop
tic
for idx = 1:length(world_file_list)
    % read world CSV to get start and goal, populate obstacles:
    world_filename = world_file_list(idx).name;
    [start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);
    
    W = arm_world_static('floor_normal_axis', floor_normal_axis, 'include_base_obstacle', 0, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles) ;
    
    % create arm agent
    A = robot_arm_3D_fetch('verbose', verbosity, 'floor_normal_axis', floor_normal_axis, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);
    
    % can adjust LLC gains here
    A.LLC.K_p = 1*A.LLC.K_p;
    A.LLC.K_i = 1*A.LLC.K_i;
    A.LLC.K_d = 1*A.LLC.K_d;
    A.joint_input_limits = 1*A.joint_input_limits;
    
    % options for FRS
    FRS_options = struct();
    FRS_options.t_plan = t_plan;
    FRS_options.origin_shift = A.joint_locations(1:3, 1);
    FRS_options.T = T;
    FRS_options.L = 0.33;
    FRS_options.buffer_dist = A.buffer_dist;
    FRS_options.combs = generate_combinations_upto(200);
    FRS_options.maxcombs = 200;
    
    % create planner
    P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options, 'first_iter_pause', 0, 'verbose', verbosity, 't_plan', t_plan, 'time_discretization', time_discretization) ;
    
    % set up world using arm
    I = A.get_agent_info ;
    W.setup(I)
    
    % place arm at starting configuration
    % W.start = zeros(6, 1); % put in "home" config
    A.state(A.joint_state_indices) = W.start ;
    
    % create simulator
    S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter) ; 
    
    % run simulation
    summary = S.run() ;
    
    % save summary
    filename = [file_location,save_file_header,num2str(idx,'%04.f'),'.mat'] ;
    save(filename, 'world_filename', 'summary')
    
    toc
end

% %% plotting
% figure(1) ; clf ; axis equal ; hold on ; grid on
% 
% plot(W)
% 
% if dimension == 3
%     view(3)
% end
% 
% animate(A)