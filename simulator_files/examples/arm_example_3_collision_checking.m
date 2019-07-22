%% description
% This script demonstrates how to use the output of A.get_agent_info as a
% volume for collision checking.
%
% Author: Shreyas Kousik
% Created: 22 July 2019
%
%% user parameters
% configuration for the 2-link, 2-DOF, 2-D arm
q = rand(2,1) ;

% obstacle location
obstacle_size = 0.3 ; % this is a scalar value
obstacle_offset = [0.5 ; 0.5] ;

%% automated from here
% create arm
A = robot_arm_agent() ;

% set arm's state
A.state(A.joint_state_indices) = q ;

% get info object
I = A.get_agent_info() ;

% create obstacle
O = obstacle_size.*make_random_polygon(round(rand_range(4,10))) + obstacle_offset ;

% get collision check volume
V = I.get_collision_check_volume(q) ;

% check for intersection
[x_int,y_int] = polyxpoly(V(1,:),V(2,:),O(1,:),O(2,:)) ;

if ~isempty(x_int)
    disp('Collision detected!')
end

%% inspect timing
average_volume_time = timeit(@() I.get_collision_check_volume(q)) ;
disp(['Average time to get volume: ',num2str(average_volume_time),' seconds'])

average_collision_check_time = timeit(@() polyxpoly(V(1,:),V(2,:),O(1,:),O(2,:))) ;
disp(['Average time for collision check: ',num2str(average_collision_check_time),' seconds'])

%% plotting
figure(1) ; clf ; hold on ; axis equal

plot(A)
plot(V(1,:),V(2,:),'r--','LineWidth',1.5)
patch('Vertices',O','Faces',[1:size(O,2), 1],...
    'FaceColor',[1 0 0],'FaceAlpha',0.3,...
    'EdgeColor',[0.5 0 0],'LineWidth',1.5)

if ~isempty(x_int)
    plot(x_int,y_int,'kx')
end