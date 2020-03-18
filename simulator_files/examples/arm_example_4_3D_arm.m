%% description
% This script shows how to use the robot_arm_agent_3D class.
%
% Author: Shreyas Kousik
% Created: 22 Jul 2019
% Updated: 1 Sept 2019
%
clear ; clf ; clc ;

%% user parameters
% joint angles in \R^4 for desired reference point
reference_point = 2*rand(4,1) - 1 ;

% amount of time to move arm (i.e., amount of trajectory to execute)
t_move = 2 ;

% total duration of trajectory (must be >= t_move)
t_total = 2 ;

%% automated from here
% create arm
A = robot_arm_3D_2link_4DOF('verbose',10) ;
A.integrator_time_discretization = 0.005 ;

%% create reference trajectory and input
% reference time and input:
T = [0 t_total] ;
U = zeros(A.n_inputs, length(T)) ; % no feedforward input this time

% reference trajectory
Z = zeros(A.n_states, size(T,2)) ;

% fill in desired point
Z(A.joint_state_indices,end) = reference_point ;

% generate desired speeds
q0 = A.state(A.joint_state_indices,1) ;
qf = reference_point ;
reference_speed = (qf - q0)/t_total ;
Z(A.joint_speed_indices,1) = reference_speed ;

%% move the agent
A.reset() ;
tic
A.move(t_move, T, U, Z)
toc

%% plot states vs. reference
figure(1) ; clf ;

% get states and reference matched in time
Z_j = A.state(A.joint_state_indices,:) ;
Z_d = A.state(A.joint_speed_indices,:) ;
Z_ref_j = match_trajectories(A.time, T, Z(A.joint_state_indices,:), A.LLC.interp_method) ;
Z_ref_d = match_trajectories(A.time, T, Z(A.joint_speed_indices,:), A.LLC.interp_method) ;

% position
subplot(2,1,1) ;
plot(A.time',Z_ref_j','b:',A.time',Z_j','b')
title('position')
xlabel('rad')

% speed
subplot(2,1,2) ;
plot(A.time',Z_ref_d','b:',A.time',Z_d','b')
title('speed')
xlabel('time')
ylabel('rad/s')

%% animate result
figure(2) ; pause(0.5) ; 
A.animate()