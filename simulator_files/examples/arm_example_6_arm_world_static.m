%% description
% This script demonstrates how to set up an arm_world_static class with 2D
% or 3D obstacles.
%
% Author: Shreyas Kousik
% Created: 23 July 2019
%
clear ; clc ;

%% user parameters
% number of obstacles
N_obstacles = 5 ;

% dimension of world
dimension = 2 ; % 2 or 3

% world verbosity
world_verbosity = 10 ;

%% automated from here
% make arm
switch dimension
    case 2
        A = robot_arm_agent ;
    case 3
        A = robot_arm_agent_3D ;
    otherwise
        error('Pick dimension = 2 or dimension = 3.')
end

% make world
W = arm_world_static('N_obstacles',N_obstacles,'dimension',dimension,...
    'verbose',world_verbosity) ;

% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration
A.state(A.joint_state_indices) = W.start ;

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(A)
plot(W)

if dimension == 3
    view(3)
end