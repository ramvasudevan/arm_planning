%% description
% 2D, 2 link rotatotope example
%
% This script generates an agent, world, and planner. It plugs them in to
% the simulator framework, and then runs a single simulation.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 19 March 2020

clear; clc; figure(1); clf;

%% user parameters
N_obstacles = 5;
dimension = 2 ;
nLinks = 2 ;
verbosity = 10 ;
allow_replan_errors = true ;
t_plan = 0.5 ;
time_discretization = 0.01 ;
T = 1 ;
use_cuda_flag = false;
agent_move_mode = 'direct' ; % pick 'direct' or 'integrator'

% for HLP:
% sampling_timeout = 0.1 ; % seconds to run sampling
% new_node_growth_distance = 0.5 ;

A = robot_arm_2D_2link_2DOF_thin('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'move_mode', agent_move_mode);

%% automated from here

W = arm_world_static('include_base_obstacle', 0, 'goal_radius', pi/30, 'N_obstacles',N_obstacles,'dimension',dimension,...
    'verbose',verbosity) ;

FRS_options = struct();
FRS_options.t_plan = t_plan;
FRS_options.origin_shift = zeros(2, 1);
FRS_options.buffer_dist = A.buffer_dist;
FRS_options.combs = generate_combinations_upto(200);
FRS_options.maxcombs = 200;
P = robot_arm_rotatotope_RTD_planner_2D_2link(FRS_options, 'verbose', verbosity, 't_plan', t_plan, 'time_discretization', time_discretization) ;

% P.HLP = robot_arm_RRT_HLP('verbose',verbosity,...
%     'sampling_timeout',sampling_timeout,...
%     'new_node_growth_distance',new_node_growth_distance) ;

% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors) ;

%% run simulation
S.run()

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(W)

if dimension == 3
    view(3)
end

animate(A)
