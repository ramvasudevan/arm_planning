%% description
% This script generates an agent, world, and planner. It plugs them in to
% the simulator framework, and then runs a single simulation.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 29 July 2019
% Updated: 3 August 2019
clear ; clc ; close all

%% user parameters
N_obstacles = 5 ;
dimension = 2 ;
verbosity = 6 ;
allow_replan_errors = true ;
t_plan = 0.5 ;

%% automated from here
switch dimension
    case 2
        A = robot_arm_2D_3link_3DOF_thin('verbose',verbosity) ;
    case 3
        A = robot_arm_3D_2link_4DOF('verbose',verbosity) ;
end

% adjust LLC gains
A.LLC.K_p = 1*A.LLC.K_p;
A.LLC.K_i = 1*A.LLC.K_i;
A.LLC.K_d = 1*A.LLC.K_d;
A.joint_input_limits = 1*A.joint_input_limits;


W = arm_world_static('include_base_obstacle', 1, 'goal_radius', 0.0025, 'N_obstacles',N_obstacles,'dimension',dimension,...
    'verbose',verbosity) ;

P = robot_arm_RTD_planner_3link2d('verbose', verbosity, 't_plan', t_plan) ;

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
