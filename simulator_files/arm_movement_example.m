%% DESCRIPTION %%
% This script is an example of how to make and use a generic robot arm
% agent in the simulator framework.
%
% Author: Shreyas Kousik
% Created: 20 Jul 2019
% Updated: 20 Jul 2019
clear ; clf ; clc ;

%% USER PARAMETERS %%
% control gains
k_p = 0.1 ;
k_d = 0.01 ; 
k_i = 0.0001 ;

% trajectory total time
t_total = 4 ;

%% AUTOMATED FROM HERE %%
%% create arm
if ~exist('A','var')
    A = robot_arm_agent() ;
end

%% create gain matrices
K_p = k_p.*[eye(A.LLC.n_agent_inputs) ; zeros(A.LLC.n_agent_inputs)] ;
K_p = reshape(K_p,A.LLC.n_agent_inputs,[]) ;
                
K_d = k_d.*[zeros(A.LLC.n_agent_inputs) ; eye(A.LLC.n_agent_inputs)] ;
K_d = reshape(K_d,A.LLC.n_agent_inputs,[]) ;
                
K_i = k_i.*eye(A.LLC.n_agent_inputs) ;

A.LLC.K_p = K_p ;
A.LLC.K_d = K_d ;
A.LLC.K_i = K_i ;

%% create reference trajectory
% reference time and input:
T = [0 t_total/2 t_total] ;
U = zeros(A.n_inputs, length(T)) ; % no feedforward input this time

% reference joint position:
joint_1 = [0, -pi/4, -pi/2] ;
joint_2 = [0, 0, 0] ;

% reference joint speeds, which try to make the arm stop at the end configuration
joint_spd_1 = [0, (-pi/2), 0] ;
joint_spd_2 = [0, 0, 0] ;

% concatenate joint and speed references in to a reference trajectory:
Z = [joint_1 ;
     joint_spd_1 ;
     joint_2 ;
     joint_spd_2 ] ;
     
%% move the agent
A.reset() ;
A.move(T(end), T, U, Z)

%% animate result
A.animate()