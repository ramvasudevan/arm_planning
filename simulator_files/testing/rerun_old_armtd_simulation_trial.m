%% user parameters
% filename = 'trial_20190410_0262.mat' ;
clear; clc; 
filename = '/Users/pdholmes/Documents/MATLAB/arm_planning/simulator_files/testing/trial_data/20191205/trial_scene_007_008.mat';

% obs_color = [1 0.7 0.7] ;
% obs_opacity = 0.5 ;
% 
% agent_camera_distance = 3 ; % default is 3
% agent_camera_position = [-3;0;1] ; % default is [-3;0;1.5]
% plot_agent_view = 'behind' ; % none, behind, above, or onboard

verbosity = 10 ;
dimension = 3;
t_plan = 0.5;
T = 1;
time_discretization = 0.01;
plot_while_running = true;
allow_replan_errors = true;
max_sim_time = 300;
max_sim_iter = 50;


%% automated from here
load(filename)

agent_info = summary(1).agent_info ;
bounds = summary(1).bounds ;
obstacles = summary(1).obstacles ;
obstacles = {obstacles{1:10}};
% obstacles = {obstacles{1:7}, obstacles{9:end}};
planner_info = summary(1).planner_info ;
start = summary(1).start ;
goal = summary(1).goal ;

% create agent
% A = quadrotor_agent('verbose',verbose_level,...
%     'plot_view_style',plot_agent_view,...
%     'camera_direction_offset',agent_camera_position,...
%     'camera_follow_dist',agent_camera_distance) ;

A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0);

% create world
% W = zonotope_box_world('verbose',verbose_level,...
%     'obs_color',obs_color,...
%     'obs_opacity',obs_opacity,...
%     'bounds',bounds,...
%     'obstacles',obstacles,...
%     'goal',goal) ;

W = fetch_base_world_static('create_random_obstacles', 0, 'include_base_obstacle', 0, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles) ;

% fill in agent state
% A.time = agent_info.time ;
% A.state = agent_info.state ;

% set up world using arm
W.setup(agent_info)
clear_plot_data(W);

% place arm at starting configuration
% W.start = zeros(6, 1); % put in "home" config
A.state(A.joint_state_indices) = W.start ;

% options for FRS

FRS_options = struct();
FRS_options.t_plan = t_plan;
FRS_options.origin_shift = A.joint_locations(1:3, 1);
FRS_options.T = T;
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;

% create planner
P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options, 'first_iter_pause', 0, 'verbose', verbosity, 't_plan', t_plan, 'time_discretization', time_discretization) ;


% create simulator
S = simulator(A,W,P, 'plot_while_running', plot_while_running, 'allow_replan_errors',allow_replan_errors,'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iter) ;

% run simulation
summary = S.run() ;
    

%% plot
% figure(1)
% clf
% hold on
% plot(W)
% axis equal
% plot_at_time(A)
% camlight
% lighting flat
% material dull
% lightangle(10,70)
%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(W)

if dimension == 3
    view(3)
end
animate(A)