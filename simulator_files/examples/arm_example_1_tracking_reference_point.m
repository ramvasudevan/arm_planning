%% DESCRIPTION %%
% This script is an example of how to make and use a generic robot arm
% agent in the simulator framework.
%
% Author: Shreyas Kousik
% Created: 20 Jul 2019
% Updated: 20 Jul 2019
clear ; clf ; clc ;

%% USER PARAMETERS %%
% amount of time to move
t_move = 4 ;

% total duration of trajectory
t_total = 4 ;

%% AUTOMATED FROM HERE %%
A = robot_arm_agent() ;

%% create reference trajectory
% reference time and input:
T = [0 t_total] ;
U = zeros(A.n_inputs, length(T)) ; % no feedforward input this time

% reference joint position:
joint_1 = [pi/2, pi/2] ;
joint_2 = [0, 0] ;

% reference joint speeds, which try to make the arm stop at the end configuration
joint_spd_1 = [(pi/2)/t_total, 0] ;
joint_spd_2 = [0, 0] ;

% concatenate joint and speed references in to a reference trajectory:
Z = [joint_1 ;
     joint_spd_1 ;
     joint_2 ;
     joint_spd_2 ] ;
     
%% move the agent
A.reset() ;
A.move(t_move, T, U, Z)

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
figure(2) ; clf ; pause(0.01)
A.animate()