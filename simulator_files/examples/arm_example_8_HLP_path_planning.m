%% description
% This script demonstrates how to set up and use the high-level planner for
% the robot arm. Currently, there are PRM and RRT planners available.
%
% Author: Shreyas Kousik
% Created: 2 Aug 2019
% Updated: 2 Aug 2019
clear ; clc ;

%% user parameters
% arm and world
dimension = 3 ; % 2 or 3
N_obstacles = 4 ;

% high-level planner
HLP_type = 'RRT' ; % choose 'PRM' or 'RRT'
sampling_timeout = 0.1 ; % seconds to run sampling
new_node_growth_distance = 0.5 ;

% user-friendly params
verbosity = 6 ;

%% automated from here
% make arm
switch dimension
    case 2
        A = robot_arm_2D_3link_3DOF('verbose',verbosity) ;
    case 3
        A = robot_arm_3D_2link_4DOF('verbose',verbosity) ;
    otherwise
        error('Pick dimension = 2 or dimension = 3.')
end

% make world
W = arm_world_static('N_obstacles',N_obstacles,'verbose',verbosity) ;
agent_info = A.get_agent_info ;
W.setup(agent_info) ;

% get world info
world_info = W.get_world_info() ;

% set agent to world state
A.reset(W.start)

%% make high-level planner
switch HLP_type
    case 'PRM'
        HLP = robot_arm_PRM_HLP('verbose',verbosity,...
            'sampling_timeout',sampling_timeout,...
            'new_node_growth_distance',new_node_growth_distance) ;
    case 'RRT'
        HLP = robot_arm_RRT_HLP('verbose',verbosity,...
            'sampling_timeout',sampling_timeout,...
            'new_node_growth_distance',new_node_growth_distance) ;
    otherwise
        error('Ope.')
end
agent_info = A.get_agent_info ;
HLP.setup(agent_info,world_info)

%% run sampling algorithm
% call HLP sample method
HLP.sample(agent_info,world_info)

% get HLP's best path
HLP.find_best_path(agent_info,world_info) ;

%% plotting
figure(1) ; clf ; hold on ; axis equal ;

plot(W) ;

% plot the agent along the best path
P = HLP.best_path_nodes ;
N_P = size(P,2) ;
disp(['Number of nodes on best path: ',num2str(N_P)])
if N_P > 0
    for idx = 1:N_P
        % set the agent's state
        A.state(A.joint_state_indices) = P(:,idx) ;

        % plot the agent
        plot(A) ;

        % pause to appreciate
        pause(0.25)
    end
end