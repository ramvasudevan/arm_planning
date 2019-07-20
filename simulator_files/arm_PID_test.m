clear ; clc ; clf ; 

%% user params
t_move = 10 ;
k_p = 100 ;
k_d = 10 ; 
k_i = 0.01 ;

%% make a new arm
A = robot_arm_agent() ; 

K_p = k_p.*[eye(A.LLC.n_agent_inputs) ; zeros(A.LLC.n_agent_inputs)] ;
K_p = reshape(K_p,A.LLC.n_agent_inputs,[]) ;
                
K_d = k_d.*[zeros(A.LLC.n_agent_inputs) ; eye(A.LLC.n_agent_inputs)] ;
K_d = reshape(K_d,A.LLC.n_agent_inputs,[]) ;
                
K_i = k_i.*eye(A.LLC.n_agent_inputs) ;

A.LLC.K_p = K_p ;
A.LLC.K_d = K_d ;
A.LLC.K_i = K_i ;

%% make trajectory
T = linspace(0,10) ;
U = zeros(A.n_inputs, size(T,2)) ;
Z = [1 - cos(T) ; sin(T) ; sin(T) ; cos(T)] ;
% Z = [4*T ; 4*ones(size(T)) ; zeros(2,length(T))] ;


%% move arm
A.move(t_move,T,U,Z)

%% plot states vs. reference
figure(1) ; clf ;


Z_j = A.state(A.joint_state_indices,:) ;
Z_d = A.state(A.joint_speed_indices,:) ;
Z_ref_j = match_trajectories(A.time, T, Z(A.joint_state_indices,:)) ;
Z_ref_d = match_trajectories(A.time, T, Z(A.joint_speed_indices,:)) ;

% position
subplot(2,1,1) ;
plot(A.time',Z_ref_j','b:',A.time',Z_j','b')
title('position')

% speed
subplot(2,1,2) ;
plot(A.time',Z_ref_d','b:',A.time',Z_d','b')
title('speed')

%% animate
% figure(2) ; clf
% animate(A)