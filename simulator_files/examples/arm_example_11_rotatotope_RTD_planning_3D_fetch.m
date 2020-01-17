%% description
% This script generates an agent, world, and planner. It plugs them in to
% the simulator framework, and then runs a single simulation.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 29 July 2019
% Updated: 3 August 2019
%
% Updated: 19 August 2019 - moving RTD to 3D and using a 1 link model
% Updated: 22 August 2019 - moving RTD to 3D and using Fetch model
% Updated: 18 September 2019 - using rotatotope RTD planner now
clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
N_random_obstacles = 10;
dimension = 3 ;
nLinks = 3 ;
verbosity = 10 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
use_cuda_flag = true;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'

A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'move_mode', agent_move_mode);

%% automated from here

% can adjust LLC gains here
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;

W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', pi/30, 'N_random_obstacles',N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.05, 'base_creation_buffer', 0.05) ;
% W = fetch_base_world_static('include_base_obstacle', 1, 'goal_radius', 0.03, 'N_random_obstacles',N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
%     'verbose',verbosity, 'creation_buffer', 0.1, 'base_creation_buffer', 0.025, 'start', [0 0 0 0 0 0]') ;

FRS_options = struct();
% FRS_options.position_dimensions = [1;2;3];
% FRS_options.extra_position_dimensions = [4;5];
% FRS_options.IC_dimensions = [6;7];
% FRS_options.param_dimensions = [8;9];
% FRS_options.nLinks = nLinks;
% FRS_options.time_discretization = 0.01;

FRS_options.t_plan = t_plan;
FRS_options.origin_shift = A.joint_locations(1:3, 1);
FRS_options.T = T;
FRS_options.L = 0.33;
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;
P = robot_arm_rotatotope_RTD_planner_3D_fetch(FRS_options, 'verbose', verbosity, 't_plan', t_plan, 'time_discretization', time_discretization, 'use_cuda_flag', use_cuda_flag) ;

% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
% W.start = zeros(6, 1); % put in "home" config
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',1000,'max_sim_iterations',1000) ;

% create .csv file
% write_fetch_scene_to_csv(W);


%% run simulation
S.run()

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(W)

if dimension == 3
    view(3)
end

animate(A, true)
