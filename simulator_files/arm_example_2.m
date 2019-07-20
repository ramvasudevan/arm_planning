%% DESCRIPTION %%
% This script is an example of how to make the arm track a sinusoidal
% reference signal in the simulator framework. In this script, one can play
% with the control gains used by the arm's low-level PID controller.
%
% Author: Shreyas Kousik
% Created: 20 Jul 2019
% Updated: 20 Jul 2019
clear ; clc ; clf ; 

%% user params
% control gains
k_p = 20 ;
k_d = 100 ; 
k_i = 0.1 ;

% low-level controller interpolation method (use 'previous' for zero order
% hold and 'linear' for linear interpolation)
interpolation_method = 'linear' ;

% timing
t_move = 10 ;
t_total = 10 ;

%% make a new arm
A = robot_arm_agent() ; 

%% set control gains and interpolation method
A.LLC.set_gains('P',k_p,'I',k_i,'D',k_d) ;
A.LLC.interp_method = interpolation_method ;

%% make trajectory
T = linspace(0,t_total) ;
U = zeros(A.n_inputs, size(T,2)) ;
Z = [1 - cos(T) ; sin(T) ; sin(T) ; cos(T)] ;

%% move arm
A.move(t_move,T,U,Z)

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

%% animate
figure(2) ; clf ; pause(0.01) % pause to make sure we don't plot in fig 1
animate(A)