%% description
% This script shows how to perform a collision check for an arbitrary
% configuration of the robot_arm_3D class.
%
% Author: Shreyas Kousik
% Created: 22 Jul 2019
%
clear ; clc ;

%% user parameters
% configuration for the 2-link, 4-DOF, 3-D arm
q = 2*rand(4,1) - 1 ;

% obstacle
obstacle_size = [0.1, 0.1, 0.1] ; % [length, width, height]
obstacle_offset = [0.2 ; 0 ; 0] ; % center

%% automated from here
% create arm
A = robot_arm_3D_2link_4DOF() ;

% set arm's state
A.state(A.joint_state_indices) = q ;

% get info object
I = A.get_agent_info() ;

% create obstacle
[~,vertices] = make_cuboid_for_patch(obstacle_size(1),...
    obstacle_size(2),obstacle_size(3),obstacle_offset) ;
faces = convhull(vertices) ; % this returns a triangulated surface
O.faces = faces ;
O.vertices = vertices ;

% get collision check volume
V = I.get_collision_check_volume(q) ;

% check for intersection
check = SurfaceIntersection(O,V) ;
check = any(check(:)) ;

if check
    disp('Collision detected!')
else
    disp('No collision detected.')
end

%% inspect timing
average_volume_time = timeit(@() I.get_collision_check_volume(q)) ;
disp(['Average time to get volume: ',num2str(average_volume_time*1000,'%0.2f'),' ms'])

average_collision_check_time = timeit(@() SurfaceIntersection(O,V)) ;
disp(['Average time for collision check: ',num2str(average_collision_check_time*1000,'%0.2f'),' ms'])

%% plotting
figure(1) ; clf ; hold on ; axis equal ; view(3) ; grid on ;

plot(A)
patch(O,'facealpha',0.3,'facecolor','r','edgecolor',[0.5 0 0])